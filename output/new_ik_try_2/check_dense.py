"""Additional offline continuous tracking with interpolated Cartesian targets."""
exec((__import__('pathlib').Path(__file__).parent/'validate_new_ik.py').read_text().split('for bound in (.12,.4):')[0])
times=np.unique(np.r_[A['time_s'],np.arange(0,float(A['time_s'][-1]),.01)])
run={'sampling':'Original source timestamps plus 0.01s Cartesian position/SLERP samples','geometric_step_bound_rad':.12,'dynamic_validation_performed':False,'sides':{}}
for side in SIDES:
 model=models[side];solver=ContinuousIK(model,'flange',range(6));original=A[side+'_aubo_base_T_flange'];rot=Slerp(A['time_s'],Rotation.from_matrix(original[:,:3,:3]))(times).as_matrix();targets=np.tile(np.eye(4),(len(times),1,1));targets[:,:3,:3]=rot
 for j in range(3):targets[:,j,3]=np.interp(times,A['time_s'],original[:,j,3])
 seed=(A[side+'_aubo_joint_positions_rad'][0]+np.pi)%(2*np.pi)-np.pi;qs=[]
 try:
  for i,target in enumerate(targets):
   seed,res=solver.solve_near(target,seed,max_step=.12);qs.append(seed.copy())
   if i%500==0:print(side,'dense',i,'/',len(times),flush=True)
  run['sides'][side]={'passed':True,'frames':len(qs)};np.savez_compressed(OUT/f'dense_{side}_offline.npz',time_s=times,joint_positions_rad=qs,execution_enabled=False)
 except Exception as e:run['sides'][side]={'passed':False,'failed_dense_frame':len(qs),'source_time_s':float(times[len(qs)]),'error':str(e)}
 print(side,run['sides'][side],flush=True)
write_json(OUT/'dense_summary.json',run)
