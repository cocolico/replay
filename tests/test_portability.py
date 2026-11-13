"""Portable resource and target-configuration tests, without device access."""
import copy,json,sys,shutil,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from replay_pipeline import HERE,config,read_json,write_json,calibrated_chain
from configure_target import configure

class PortabilityTests(unittest.TestCase):
    def test_all_active_config_paths_stay_in_package(self):
        for p in (HERE/'config').glob('*.json'):
            if 'source_episode_root' not in read_json(p):continue
            c=config(p)
            paths=[c[k] for k in ('source_episode_root','source_franka_handeye','table_mapping','current_table_observation','camera_trajectory_root','camera_table_mapping','camera_source_observation') if c.get(k)]
            paths += [d[k] for d in c['sides'].values() for k in ('handeye','urdf')]
            for value in paths:
                path=Path(value);self.assertTrue(path.exists(),str(path));self.assertTrue(path.resolve().is_relative_to(HERE.resolve()),str(path))

    def test_target_configuration_accepts_new_camera_id_and_localizes_files(self):
        old=config(HERE/'config/human_replay.json')
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'code';root.mkdir()
            shutil.copytree(HERE/'config',root/'config')
            shutil.copytree(HERE/'resources/calibration',root/'resources/calibration')
            inputs=Path(folder)/'inputs';inputs.mkdir()
            obs=read_json(old['current_table_observation']);obs['camera_id']='new-test-workcell-camera';write_json(inputs/'observation.json',obs)
            spec={'table_observation':'observation.json','sides':{}}
            for index,side in enumerate(('left','right')):
                shutil.copy2(old['sides'][side]['handeye'],inputs/(side+'.json'))
                spec['sides'][side]={'ip':f'192.0.2.{index+1}','robot_name':'rob1','hand_address':f'192.0.2.{index+10}:50001','hand_serial':side+'_TEST','handeye':side+'.json','mount_confirmed':True,'flange_T_wuji_base':old['sides'][side]['flange_T_wuji_base']}
            write_json(inputs/'spec.json',spec);result=configure(inputs/'spec.json',root)
            self.assertTrue(result['configured']);self.assertFalse(result['devices_contacted'])
            c=config(root/'config/human_replay.json');mapping=read_json(c['camera_table_mapping']);self.assertEqual(mapping['target_camera_id'],obs['camera_id'])
            calibrated_chain(c,c['camera_table_mapping'],c['camera_source_id'])
            for side in ('left','right'):
                self.assertTrue(Path(c['sides'][side]['handeye']).is_relative_to(root))
                self.assertEqual(read_json(root/'config/runtime.json')['hands'][side]['serial'],side+'_TEST')
            self.assertTrue((root/'history/reference_configs/human_replay.json').exists())

    def test_incomplete_target_spec_does_not_change_active_configuration(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'config').mkdir();p=root/'config/human_replay.json';p.write_text('original')
            spec=root/'spec.json';write_json(spec,{})
            with self.assertRaises(ValueError):configure(spec,root)
            self.assertEqual(p.read_text(),'original')

if __name__=='__main__':unittest.main()
