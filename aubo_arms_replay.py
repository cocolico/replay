#!/usr/bin/env python3
"""Checked camera-trajectory to real Aubo arms. Default: read-only preparation."""
import argparse
import concurrent.futures
import os
from pathlib import Path
import time
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
from replay_pipeline import HERE, SIDES, config, read_json, write_json, prepare_camera, provenance, hand_contract
from device_backends import Aubo
from read_hands import Hands,check_current_limit
from replay_control import acquire_replay_lock, read_arms, execute, require_start
from runtime_plan import check_controller, sample_replay, transition, dynamics, split, joint_spline
from collision_scene import Scene, DEFAULT_CLEARANCE_M
from dh_kinematics import fk
from aubo_continuous_ik import cartesian_targets
from diagnose_ik import diagnose
from solve_controller_ik import solve_side as sdk_solve_side

SETTINGS = dict(rate_hz=100, speed=.2, minimum_transition_s=10., arm_velocity_fraction=.2,
                arm_acceleration_fraction=.15, feedback_timeout_s=.25, maximum_tick_lateness_s=.03,
                arm_tracking_error_rad=.2, fixed_hand_position_tolerance_rad=.03, warmup_s=2., mesh_clearance_m=DEFAULT_CLEARANCE_M,
                table_clearance_m=0., maximum_eef_position_error_m=.002,maximum_hand_current_a=1.5)


def solve_side(c, side, arrays, controllers, hands, out):
    """Use the same per-source-frame SDK solver as successful episode594."""
    report = {'backend':'episode594_sdk', 'execution_enabled':False,
              'collision_checked':False, 'runtime_acceptance':False}
    try:
        qs, details = sdk_solve_side(c, side, arrays)
        for key in ('robot_type', 'kinematics_real', 'joint_min_rad', 'joint_max_rad'):
            if details['snapshot'][key] != controllers[side][key]:
                raise ValueError('Controller model/limits changed during IK: '+side)
        path = out/f'{side}_sdk_offline.npz'
        np.savez_compressed(path, time_s=arrays['time_s'], joint_positions_rad=qs,
                            execution_enabled=False)
        report.update(details, passed=True, whole_trajectory_solutions=1)
        return [(arrays['time_s'].copy(), qs, path)]
    except (ValueError, RuntimeError) as error:
        report.update(passed=False, whole_trajectory_solutions=0, error=str(error))
        raise
    finally:
        write_json(out/f'{side}_ik.json', report)


def smooth_servo_waypoints(waypoints, first_end, rate, subdivisions, vlim, alim):
    knots = np.arange(len(waypoints))
    # Each feedback checkpoint may become a stationary hold.
    stops = [*range(first_end+1), len(knots)-1]
    curve = joint_spline(knots, waypoints, stops)
    stride = subdivisions
    for _ in range(8):
        ticks = np.arange(knots[-1]*stride+1)
        sampled = curve(ticks/stride)
        sampled[::stride] = waypoints
        # Include the deceleration into a hold and the restart from rest.
        checked = np.insert(sampled, np.asarray(stops)*stride, sampled[np.asarray(stops)*stride], axis=0)
        v, a = dynamics(checked, rate)
        moving_v, moving_a = dynamics(sampled, rate)
        scale = max(np.max(np.maximum(v,moving_v)/vlim), np.sqrt(np.max(np.maximum(a,moving_a)/alim)))
        if scale <= 1:
            return sampled, stride
        stride = max(stride+1, int(np.ceil(stride*scale)))
    raise ValueError('Cannot smooth Servo waypoints within configured dynamics')


def runtime_pair(c, controllers, hands, left, right, settings, arrays):
    times, lq, _ = left;rt, rq, _ = right
    np.testing.assert_array_equal(times, rt)
    arms_only = settings.get('arms_only', True)
    hand_path = {}
    for side in SIDES:
        hand_path[side] = (np.tile(hands[side]['positions_rad'], (len(times), 1)) if arms_only
                           else np.asarray(arrays[side+'_wuji_joint_positions_rad']))
        if hand_path[side].shape != (len(times),20):raise ValueError('Hand/source frame count mismatch: '+side)
    source = np.concatenate([lq, hand_path['left'], rq, hand_path['right']], axis=1)
    start = np.concatenate([controllers['left']['joint_positions_rad'], hands['left']['positions_rad'],
                            controllers['right']['joint_positions_rad'], hands['right']['positions_rad']])
    lo = [];hi = [];vlim = [];alim = []
    for side in SIDES:
        s = controllers[side]
        if arms_only:hl,hu=[-np.inf]*20,[np.inf]*20
        else:
            _,_,hl,hu=hand_contract(Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf',side)
        lo.extend(s['joint_min_rad']);lo.extend(hl)
        hi.extend(s['joint_max_rad']);hi.extend(hu)
        vlim.extend(np.asarray(s['joint_max_speed_rad_s'])*settings['arm_velocity_fraction']);vlim.extend([1. if arms_only else settings['hand_velocity_rad_s']]*20)
        alim.extend(np.asarray(s['joint_max_acceleration_rad_s2'])*settings['arm_acceleration_fraction']);alim.extend([1. if arms_only else settings['hand_acceleration_rad_s2']]*20)
    lo, hi, vlim, alim = map(np.asarray, (lo, hi, vlim, alim));speed = settings['speed'];rate = settings['rate_hz']
    sampling = settings.get('trajectory_sampling')
    if sampling in ('source_waypoints', 'source_waypoints_servoj'):
        intervals = np.diff(times)/speed
        if len(intervals)<2 or not np.allclose(intervals,intervals[0],rtol=1e-5,atol=1e-8):
            raise ValueError('Source waypoint mode requires uniform timestamps')
        rate = 1/float(intervals[0]);settings['rate_hz'] = rate
        if not np.isfinite(source).all() or np.any(source<lo-1e-6) or np.any(source>hi+1e-6):
            raise ValueError('Source waypoint position limit violation')
        trajectory = source
        first_step_end = 0
        feedback_stride = 1
        if sampling == 'source_waypoints_servoj':
            # AUBO servoJoint rejects adjacent arm commands over 15 degrees.
            # Keep the same clock for both arms and both hands, adding ticks
            # only on segments that would violate the controller contract.
            rows = [source[0]]
            first_limit = settings.get('first_step_arm_deg')
            if first_limit is not None and not (0 < settings['first_step_right_tolerance_deg'] < first_limit <= 15):
                raise ValueError('Invalid first-step angle or arrival tolerance')
            for i, (previous, target) in enumerate(zip(source[:-1], source[1:])):
                delta = target - previous
                arm_step = max(abs(delta[:6]).max(), abs(delta[26:32]).max())
                limit = first_limit if i == 0 and first_limit is not None else 15.
                parts = max(1, int(np.ceil(arm_step / np.deg2rad(limit))))
                if first_limit is not None:
                    if i == 0:
                        first_step_end = parts
                    elif parts > 1:
                        raise ValueError(f'Source step {i+1} exceeds 15 degrees at original cadence')
                for part in range(1, parts):
                    rows.append(previous + delta * (part / parts))
                rows.append(target)
            trajectory = np.asarray(rows)
            subdivisions = settings.get('servo_subdivisions',1)
            if type(subdivisions) is not int or not 1 <= subdivisions <= 10:
                raise ValueError('Invalid Servo subdivisions')
            if subdivisions > 1:
                rate *= subdivisions
                settings['rate_hz'] = rate
                trajectory, feedback_stride = smooth_servo_waypoints(
                    trajectory, first_step_end, rate, subdivisions, vlim, alim)
                first_step_end *= feedback_stride
                if not np.isfinite(trajectory).all() or np.any(trajectory<lo-1e-6) or np.any(trajectory>hi+1e-6):
                    raise ValueError('Smoothed waypoint position limit violation')
        v,a = dynamics(trajectory,rate)
        if np.any(v>vlim+1e-6) or np.any(a>alim+1e-6):
            raise ValueError('Source waypoint velocity/acceleration bounds exceeded at requested speed')
        initial_target_warnings=[]
        for side in SIDES:
            arm0,hand0=split(source[0]);arm_start,hand_start=split(start)
            arm_delta=float(np.max(abs(arm0[side]-arm_start[side])))
            hand_delta=float(np.max(abs(hand0[side]-hand_start[side])))
            if arm_delta>.002 or hand_delta>.03:
                warning={'type':'source_waypoint_initial_target_discontinuity','side':side,
                         'max_arm_delta_rad':arm_delta,'arm_threshold_rad':.002,
                         'max_hand_delta_rad':hand_delta,'hand_threshold_rad':.03,
                         'action':'warn_only_continue'}
                initial_target_warnings.append(warning);print('WARNING',warning,flush=True)
        if settings.get('collision_check_policy') != 'disabled_by_operator':
            raise ValueError('Source waypoint mode requires explicit collision policy')
        return trajectory,dict(ready_for_execution=True,collision_checked=False,collision_passed=None,
            collision_check_policy='disabled_by_operator',collision_failures=[],checked_mesh_samples=0,
            hand_self_collision_policy='disabled',hand_self_collision_warning_frames=0,hand_self_collision_warnings=[],
            effective_speed=float((times[-1]-times[0])*rate/(len(trajectory)-1)),duration_s=(len(trajectory)-1)/rate,transition_s=0.,
            max_velocity_rad_s=v.tolist(),max_acceleration_rad_s2=a.tolist(),
            initial_target_discontinuity_warnings=initial_target_warnings,
            trajectory_sampling=sampling,predicted_waypoints=len(source)-1,
            added_servo_waypoints=len(trajectory)-len(source),
            first_step_end_index=first_step_end,
            first_step_feedback_stride=feedback_stride,
            includes_measured_start=True,command_interval_s=1/rate,
            eef_checked=False,eef_check_note='Independent SDK FK check covers the original source waypoints.')
    for _ in range(16):
        ticks, replay = sample_replay(times, source, rate, speed);v, a = dynamics(replay, rate)
        if np.all(v<=vlim+1e-6) and np.all(a<=alim+1e-6):break
        speed *= .8
    else:raise ValueError('Unable to retime replay within arm velocity/acceleration limits')
    duration = max(2/rate, settings['minimum_transition_s'], float(np.max(1.9*abs(source[0]-start)/vlim)),
                   float(np.max(np.sqrt(6*abs(source[0]-start)/alim))))
    for _ in range(8):
        ramp_t, ramp = transition(start, source[0], rate, duration)
        v, a = dynamics(ramp, rate)
        if np.all(v<=vlim+1e-6) and np.all(a<=alim+1e-6):break
        duration *= 1.3
    else:raise ValueError('Transition dynamic limit violation')
    hold_ticks = round(settings.get('entry_hold_s',1.)*rate)
    trajectory = np.concatenate([ramp, np.tile(source[0], (hold_ticks, 1)), replay[1:]])
    v, a = dynamics(trajectory, rate)
    if not np.isfinite(trajectory).all() or np.any(trajectory<lo-1e-6) or np.any(trajectory>hi+1e-6):raise ValueError('Runtime position limit violation')
    if np.any(v>vlim+1e-6) or np.any(a>alim+1e-6):raise ValueError('Combined runtime dynamic limit violation')
    collision_disabled = settings.get('collision_check_policy') == 'disabled_by_operator'
    bad = [];mesh_checked=0;hand_self_warnings=[];hand_self_warning_frames=0;source_hand_self_warning_frames=0
    if not collision_disabled:
        scene = Scene(c, controllers, settings['mesh_clearance_m'], table_clearance=settings['table_clearance_m'])
        if not arms_only:
            for frame,row in enumerate(source):
                arm_state,finger_state=split(row);findings=scene.check_hand_self(arm_state,finger_state)
                if findings:
                    source_hand_self_warning_frames+=1
                    if len(hand_self_warnings)<100:
                        hand_self_warnings.append({'source_frame':frame,'source_time_s':float(times[frame]),'findings':findings})
        for i, row in enumerate(trajectory):
            mesh_checked+=1
            arms, fingers = split(row)
            if not arms_only:
                hand_findings=scene.check_hand_self(arms,fingers)
                if hand_findings:
                    hand_self_warning_frames+=1
                    if len(hand_self_warnings)<100:
                        hand_self_warnings.append({'runtime_frame':i,'runtime_time_s':i/rate,'findings':hand_findings})
                    if hand_self_warning_frames==1:print('WARNING hand self-collision detected; continuing because policy is warn',flush=True)
            findings = scene.check(arms, fingers,ignore_hand_self=not arms_only)
            if findings:
                bad.append({'frame':i, 'runtime_time_s':i/rate, 'phase':'transition' if i<len(ramp) else 'replay', 'findings':findings})
                if len(bad)>=20:break
            if i%1000 == 0:print('full assembly mesh', i, '/', len(trajectory), flush=True)
    max_pe = None;max_ae = None;eef_deviation_warnings=[]
    if not bad:
        # Check interpolated joint commands against the requested Cartesian curve as well.
        query = np.minimum(times[-1], times[0]+ticks*speed)
        max_pe = 0.;max_ae = 0.
        for side in SIDES:
            desired = cartesian_targets(arrays['time_s'], arrays[side+'_aubo_base_T_flange'], query)
            for i, target in enumerate(desired):
                arms, _ = split(replay[i]);actual = fk(arms[side], controllers[side]['kinematics_real'])
                pe = np.linalg.norm(actual[:3, 3]-target[:3, 3]);ae = Rotation.from_matrix(actual[:3, :3].T@target[:3, :3]).magnitude()
                max_pe = max(max_pe, float(pe));max_ae = max(max_ae, float(ae))
        if max_pe>settings['maximum_eef_position_error_m'] or max_ae>.01:
            warning={'type':'interpolated_eef_path_deviation',
                     'max_position_error_m':max_pe,'position_threshold_m':settings['maximum_eef_position_error_m'],
                     'max_rotation_error_rad':max_ae,'rotation_threshold_rad':.01,
                     'action':'warn_only_continue'}
            eef_deviation_warnings.append(warning);print('WARNING',warning,flush=True)
    return trajectory, {'ready_for_execution':not bad, 'collision_checked':not collision_disabled, 'collision_passed':None if collision_disabled else not bad,
                        'collision_check_policy':'disabled_by_operator' if collision_disabled else 'required',
                        'collision_failures':bad, 'hand_self_collision_policy':'disabled' if collision_disabled else ('warn' if not arms_only else 'block'),
                        'hand_self_collision_warning_frames':hand_self_warning_frames,'hand_self_collision_warnings':hand_self_warnings,
                        'source_hand_self_collision_warning_frames':source_hand_self_warning_frames,
                        'mesh_clearance_m':settings['mesh_clearance_m'], 'table_clearance_m':settings['table_clearance_m'], 'maximum_eef_position_error_m':settings['maximum_eef_position_error_m'], 'checked_mesh_samples':mesh_checked, 'effective_speed':speed,
                        'duration_s':(len(trajectory)-1)/rate, 'transition_s':float(ramp_t[-1]),
                        'max_velocity_rad_s':v.tolist(), 'max_acceleration_rad_s2':a.tolist(),
                        'trajectory_sampling':'cubic_hermite_100hz',
                        'source_waypoints':len(source), 'runtime_waypoints':len(trajectory),
                        'includes_measured_start':True, 'command_interval_s':1/rate,
                        'max_replay_eef_position_error_m':max_pe, 'max_replay_eef_rotation_error_rad':max_ae,
                        'eef_deviation_warnings':eef_deviation_warnings,'eef_checked':not bad}


def require_plan(c, plan, trajectory, reader):
    if not all(plan.get(k) is True for k in ('ready_for_execution', 'collision_checked', 'collision_passed')):
        raise ValueError('Preflight has not passed')
    if not isinstance(plan.get('arms_only'),bool):raise ValueError('Missing explicit arm/hand execution mode')
    if plan['arms_only'] != plan.get('settings',{}).get('arms_only',True):raise ValueError('Arm/hand execution mode changed')
    if provenance(c['_config_path']) != plan['configuration']:raise ValueError('Configuration changed')
    for item in plan['sources']:
        if provenance(item['path']) != item:raise ValueError('Source or implementation changed: '+item['path'])
    if provenance(plan['trajectory']['path']) != plan['trajectory']:raise ValueError('Trajectory changed')
    with np.load(plan['trajectory']['path'], allow_pickle=False) as z:
        if not np.array_equal(z['joint_positions_left_right_rad'], trajectory):raise ValueError('Trajectory in memory differs')
    live=read_arms(c);fresh_hands=reader.snapshot();check_current_limit(fresh_hands,plan['settings']['maximum_hand_current_a'])
    require_start(plan,live,fresh_hands)
    for side in SIDES:
        if np.max(abs(np.asarray(live[side]['joint_positions_rad'])-np.asarray(plan['controllers'][side]['joint_positions_rad'])))>.002:
            raise ValueError('Arm start drifted after planning; rerun preflight: '+side)
        if np.max(abs(np.asarray(fresh_hands[side]['positions_rad'])-np.asarray(plan['hand_start'][side]['positions_rad'])))>.005:
            raise ValueError('Hand start drifted after planning; rerun preflight: '+side)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('--config', type=Path, default=HERE/'config/human_replay.json')
    parser.add_argument('--speed', type=float, default=.2)
    parser.add_argument('--arms-only', action='store_true', help='Keep fingers fixed; default replays both arms and fingers')
    parser.add_argument('--first-frame-only', action='store_true', help='Move from the measured pose to frame zero, then stop')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not 0<args.speed<=.2:parser.error('speed must be (0,0.2]')
    if not args.trajectory.is_dir():parser.error('trajectory must be a camera trajectory folder')
    lock = acquire_replay_lock();reader = None
    name = 'aubo_arms_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    out = HERE/'output'/name;out.mkdir();settings = dict(read_json(HERE/'config/runtime.json'), **SETTINGS)
    settings.update(speed=args.speed, ik_method='episode594_sdk', arms_only=args.arms_only)
    summary = {'ready_for_execution':False, 'execution_attempted':False, 'arms_only':args.arms_only, 'name':name, 'ik_method':'episode594_sdk'}
    def session(status, **extra):write_json(HERE/'output/replay_session.json', dict(pid=os.getpid(), name=name, status=status, updated_unix_s=time.time(), **extra))
    session('preflight');print('OUTPUT', out, flush=True)
    try:
        c = config(args.config)
        controllers = read_arms(c)
        for side in SIDES:check_controller(c, side, controllers[side])
        reader = Hands(read_json(HERE/'config/runtime.json')['hands']);hands = reader.snapshot()
        current_report=check_current_limit(hands,settings['maximum_hand_current_a'])
        settings['hands'] = read_json(HERE/'config/runtime.json')['hands']
        c.pop('_config_path');c['camera_trajectory_root'] = str(args.trajectory.resolve())
        for side in SIDES:
            c['sides'][side]['model_confirmed'] = True
            c['sides'][side]['model_confirmation_source'] = 'Live SDK robot_type=aubo_i7H; actual DH used and FK verified'
        write_json(out/'config.json', c);c = config(out/'config.json')
        write_json(out/'planning_snapshot.json',{'controllers':controllers,'hands':hands,'hand_current_check':current_report,'read_only':True})
        prepared = prepare_camera(c, out)
        if prepared['blockers']:raise ValueError('Unresolved coordinate/mount inputs: '+str(prepared['blockers']))
        with np.load(out/'human_camera_prepared.npz', allow_pickle=False) as z:arrays = {k:z[k].copy() for k in z.files}
        sources = prepared['sources']+[provenance(p) for p in sorted(args.trajectory.glob('*.npy'))]
        sources += [provenance(p) for p in (Path(__file__), HERE/'camera_trajectory_input.py', HERE/'execute_checked_replay.py', HERE/'aubo_continuous_ik.py', HERE/'diagnose_ik.py', HERE/'solve_controller_ik.py', HERE/'replay_control.py', HERE/'collision_scene.py', HERE/'device_backends.py', HERE/'runtime_plan.py', HERE/'dh_kinematics.py', HERE/'read_hands.py', HERE/'replay_pipeline.py')]
        sources += [provenance(p) for p in sorted((HERE/'vendor/replay_ik/src/eef_replay/ik').glob('*.py'))]
        for side in SIDES:
            urdf = Path(c['sides'][side]['urdf'])
            sources += [provenance(urdf), provenance(urdf.with_suffix('.srdf'))]
        sources += [provenance(p) for p in sorted((HERE/'vendor/pyaubo_sdk_0_26').rglob('*.so'))]
        # Mesh files affect collision results and belong to the execution fingerprint.
        for root in (Path(c['sides']['left']['urdf']).parents[1]/'meshes/aubo_i7H', Path(c['source_episode_root'])/'assets/wuji'):
            sources += [provenance(p) for p in sorted(root.rglob('*.stl'))]
        # Same nominal candidate preparation used by episode594, without changing
        # the source frame timestamps or using collision to choose IK branches.
        if args.first_frame_only:
            # The replay interpolator needs two knots. Both represent the exact
            # same home pose; all movement is the measured-start transition.
            arrays = {'time_s': arrays['time_s'][:2], **{
                key: np.repeat(arrays[key][:1], 2, axis=0)
                for side in SIDES for key in (side+'_aubo_base_T_flange',side+'_wuji_joint_positions_rad')}}
            sdk_arrays = dict(arrays)
            for side in SIDES:
                sdk_arrays[side+'_aubo_joint_positions_rad'] = [controllers[side]['joint_positions_rad']]
            summary['source_frame'] = 0
        else:
            candidate = out/'nominal_ik_candidate.npz'
            diagnostic = diagnose(c, out/'human_camera_prepared.npz',
                                  out/'human_camera_report.json', 1, candidate)
            write_json(out/'nominal_ik_diagnostic.json', diagnostic)
            sources.append(provenance(candidate))
            with np.load(candidate, allow_pickle=False) as z:
                sdk_arrays = {k:z[k].copy() for k in z.files}
        solutions = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            pending = {side:pool.submit(solve_side, c, side, sdk_arrays, controllers,
                                       hands, out) for side in SIDES}
            errors = []
            for side in SIDES:
                try:solutions[side] = pending[side].result()
                except (ValueError, RuntimeError) as error:
                    solutions[side] = [];errors.append(side+': '+str(error))
        summary['whole_trajectory_solutions'] = {s:len(solutions[s]) for s in SIDES}
        if errors:raise ValueError('SDK per-frame IK failed; '+'; '.join(errors))
        attempts = [];chosen = None
        for left in solutions['left']:
            for right in solutions['right']:
                try:trajectory, plan = runtime_pair(c, controllers, hands, left, right, settings, arrays)
                except ValueError as error:
                    attempts.append({'ready_for_execution':False,'error':str(error)})
                    continue
                attempts.append(plan)
                if plan['ready_for_execution']:chosen = trajectory, plan;break
            if chosen:break
        write_json(out/'pair_checks.json', attempts)
        if chosen is None:raise ValueError('All branch pairs blocked by full assembly/transition checks')
        trajectory, plan = chosen
        # Final independent controller FK verification of both complete selected branches.
        for side, solution in zip(SIDES, (left, right)):
            arm = Aubo(c['sides'][side]);times, qs, path = solution
            try:
                targets = cartesian_targets(arrays['time_s'], arrays[side+'_aubo_base_T_flange'], times)
                for q, target in zip(qs, targets):
                    actual = arm.fk(q)
                    if np.linalg.norm(actual[:3, 3]-target[:3, 3])>1e-5 or Rotation.from_matrix(actual[:3, :3].T@target[:3, :3]).magnitude()>1e-4:raise ValueError('Controller FK validation failed: '+side)
            finally:arm.close()
        dest = out/'runtime.npz';np.savez_compressed(dest, joint_positions_left_right_rad=trajectory, execution_enabled=False)
        plan.update(name=name, arms_only=args.arms_only, execution_enabled=False, configuration=provenance(c['_config_path']),
                    sources=sources, trajectory=provenance(dest), controllers=controllers, hand_start=hands, settings=settings)
        write_json(out/'runtime.json', plan);summary.update(ready_for_execution=True, runtime=provenance(out/'runtime.json'))
        print('PREFLIGHT PASS', out/'runtime.json', flush=True)
        if args.execute:
            require_plan(c, plan, trajectory, reader);summary['execution_attempted'] = True
            execute(c, settings, trajectory, plan, reader, arms_only=args.arms_only)
        else:print('Read-only preflight complete. No movement commands sent.', flush=True)
    except BaseException as e:
        summary['error'] = str(e);print('FAILED:' if summary['execution_attempted'] else 'BLOCKED:', str(e), flush=True)
        raise
    finally:
        write_json(out/'summary.json', summary)
        # Do not overwrite a stop-not-confirmed state produced by execution cleanup.
        current = read_json(HERE/'output/replay_session.json')
        if current.get('pid') == os.getpid() and current.get('status') == 'preflight':session('stopped', error=summary.get('error'))
        if reader is not None:reader.close()
        lock.close()

if __name__ == '__main__':
    try:main()
    except (ValueError, RuntimeError):raise SystemExit(2)
    except KeyboardInterrupt:raise SystemExit(130)
