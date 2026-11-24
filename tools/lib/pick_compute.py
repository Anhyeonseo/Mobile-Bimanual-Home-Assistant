"""Persistent detection/grasp/planning runtime, exclusively in a worker process."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def make_runtime(options):
    return PickCompute(options)


class PickCompute:
    def __init__(self, options):
        self.options = options
        self.detector = self.grasp = None
        from tools.diagnostics.carry_moveit_session import PersistentSession
        self.planner = PersistentSession()

    def prepare(self):
        import cv2
        cv2.setNumThreads(1)
        from tools.lib.grounding_rgbd import GroundingRGBD
        from tools.lib.sam_rgbd import SAMMasks
        artifacts=Path(self.options.get('artifacts_dir') or ROOT/'artifacts').resolve()
        cache=artifacts/'models/huggingface'
        import torch
        # CPU inference and preprocessing leave headroom for real observations.
        # This does not extend sensor leases or certify a real-time deadline.
        torch.set_num_threads(min(torch.get_num_threads(), 2))
        self.detector = GroundingRGBD(device=self.options.get('device','auto'),use_sam=True,cache_dir=cache,local_files_only=True)
        self.detector.segmenter = SAMMasks(self.detector.device,cache_dir=cache,local_files_only=True)
        if self.options.get('candidate_source','so101') in ('graspnet','graspnet-first'):
            from home_perception.grasp import graspnet
            graspnet.ARTIFACTS=artifacts  # private compute process only
            self.grasp = graspnet.GraspNetRuntime(artifacts/'models/checkpoint-rs.tar',
                                                device=self.detector.device, audit=False)
        return dict(detector_ready=True,graspnet_ready=self.grasp is not None,models_loaded_once=True,
                    compute_scheduling=self.options.get('compute_scheduling'),torch_threads=torch.get_num_threads())

    def warm_planner(self, profile, joints, output):
        from home_robot_tasks import s2_geometry as geo
        from home_robot_tasks.stationary_pick_sim import StationaryArm
        from home_robot_tasks.so101_jaw import load_stock_jaw
        from home_robot_tasks.carry_model import parameters
        from moveit_msgs.msg import RobotState
        config = ROOT/'config/home_assistant'
        geometry = geo.load_geometry(config/'official_geometry.json')
        ranges = json.loads((config/'arm_ranges.development.json').read_text())
        jaw = load_stock_jaw(config/'official_sources/so101_new_calib.urdf',ROOT/'ros2_ws/src/so101_description/meshes')
        arms = {s:StationaryArm(profile,geometry,ranges,joints['s2_lift_joint'],jaw,s) for s in ('left','right')}
        params = parameters(ROOT,profile,geometry,arms,Path(self.options['mobile_mesh_dir']))
        state = RobotState();state.joint_state.name=list(joints);state.joint_state.position=list(joints.values())
        with self.planner(params,state,Path(output)):
            pass
        return dict(moveit_starts=self.planner.starts)

    def save(self, folder, images, evidence):
        from home_robot_tasks.pick_capture import save_capture
        return save_capture(Path(folder),images,evidence)

    def locate(self, snapshot, prompt):
        _, _, folder = self.detector.analyze_snapshot(Path(snapshot), prompt)
        return str(folder/'detections.json')

    def detect(self, snapshot, prompt):
        path = self.locate(snapshot, prompt)
        report = json.loads(Path(path).read_text())
        if report['status'] != 'OK':
            raise ValueError('target detection failed: '+report['status'])
        if not any(d['label'] == prompt for d in report['detections']):
            raise ValueError('target detection failed: requested description did not match')
        return path

    def camera_candidates(self, snapshot, detections, prompt, camera_mode, output, location=None):
        from tools.diagnostics.replay_s3_grasp import replay
        from tools.diagnostics.plan_capture_carry import serial
        result = replay(Path(snapshot), Path(detections), prompt, side=location, camera_mode=camera_mode,
                        candidate_source=self.options.get('candidate_source','so101'),
                        grasp_inference=self.grasp,
                        grasp_checkpoint=self.grasp.checkpoint if self.grasp else None, grasp_audit=False)
        matches = [r for r in result.get('observations', []) if r['label'] == prompt]
        index = result.get('group', {}).get('observation_index')
        if type(index) is int and 0 <= index < len(result.get('observations', [])):
            matches = [result['observations'][index]]
        surface_error = None
        surface_height = None
        if result['status'] == 'CAMERA_CANDIDATES_ONLY' and len(matches) == 1:
            from home_robot_tasks.camera_evidence import load_capture_profile
            from home_robot_tasks.pick_calibration import check_top_surface
            from home_robot_tasks.s2_geometry import optical_to_base
            profile, _ = load_capture_profile(ROOT, camera_mode)
            xyz = matches[0].get('center_xyz_m') or matches[0].get('surface_xyz_m')
            if xyz is None:
                surface_error = '대상 물체에 대응하는 3차원 위치를 확인하지 못했습니다.'
            else:
                try:
                    z = check_top_surface(result['support_plane'], optical_to_base(profile['base_from_optical'], xyz), profile)
                    surface_height = dict(base_z_m=z, ground_z_m=profile['ground_z_m'],
                        height_above_model_floor_m=z-profile['ground_z_m'],
                        floor_reference='calibrated_model_not_fresh_floor_observation',
                        calibration_id=profile['calibration_id'])
                except ValueError as exc:
                    surface_error = str(exc)
        result.update(surface_height=surface_height, surface_error=surface_error)
        Path(output).write_text(json.dumps(result, default=serial, indent=2, allow_nan=False))
        return dict(status=result['status'], reason=result.get('reason'), surface_error=surface_error,
                    surface_height=surface_height,
                    count=len(result.get('group', {}).get('refined_grasps', [])),
                    depth_fraction=matches[0].get('valid_depth_fraction') if len(matches)==1 else None)

    def candidates(self, snapshot, detections, prompt, output, camera_mode=None, location=None):
        from tools.diagnostics.replay_s3_grasp import replay
        from tools.diagnostics.plan_capture_carry import serial
        snapshot = Path(snapshot)
        report = json.loads((snapshot/'robot_capture.json').read_text())
        result = replay(snapshot,Path(detections),prompt,side=location,robot_report=report,prepare_carry=True,
                        candidate_source=self.options.get('candidate_source','so101'),
                        camera_mode=camera_mode,grasp_inference=self.grasp,
                        grasp_checkpoint=self.grasp.checkpoint if self.grasp else None, grasp_audit=False)
        Path(output).write_text(json.dumps(result,default=serial,indent=2,allow_nan=False)+'\n')
        if result['status'] != 'CAPTURE_CANDIDATES_READY':
            if result['status'] == 'AMBIGUOUS_TARGET':
                raise ValueError('capture/candidate rejected: AMBIGUOUS_TARGET · 대상을 하나로 구분하지 못했습니다. '
                                 '촬영 화면 기준으로 왼쪽 또는 오른쪽을 지정해 주세요. 예: 오른쪽 지우개를 집어줘')
            raise ValueError('capture/candidate rejected: '+result['status']+' '+str(result.get('reason','')))
        return output

    def lift_preflight(self, prepared, output, board):
        from home_robot_tasks.pick_lift_transition import preflight
        return preflight(Path(prepared),Path(output),self.options,self.planner,board)

    def bind_arrival(self, prepared, snapshot, detections, arrival, proof, output):
        from home_robot_tasks.pick_lift_transition import bind_arrival
        return bind_arrival(Path(prepared),Path(snapshot),Path(detections),Path(arrival),proof,Path(output))

    def bind_resume(self, prepared, arrival, output, scene_unchanged=False):
        from home_robot_tasks.pick_lift_transition import bind_resume
        return bind_resume(Path(prepared),Path(arrival),Path(output),scene_unchanged=scene_unchanged)

    def plan(self, prepared, detections, snapshot, output):
        from tools.diagnostics.plan_capture_carry import plan
        from tools.lib.pick_plan_validation import validate_pick_plan
        from tools.diagnostics.bundle_carry_execution import bundle
        output = Path(output)
        from tools.lib.pick_planning import station_plan_options, save_plan_options
        args = station_plan_options(prepared, output/'planning', self.options['mobile_mesh_dir'],
                                    maximum_grasps=self.options.get('maximum_grasps',8))
        save_plan_options(args, output/'planning-options.json')
        try:
            report = plan(args,session_factory=self.planner)
        except ValueError as exc:
            raise ValueError('no executable route: '+str(exc)) from exc
        if report['status'] != 'OFFLINE_ROUTE_FOUND':
            from tools.lib.pick_planning import planning_failure_message
            raise ValueError(planning_failure_message(report))
        report_path=args.output/'report.json'
        proof=output/'proof'
        try:
            validate_pick_plan(report_path,prepared,args.mobile_mesh_dir,proof,session_factory=self.planner)
        except ValueError as exc:
            raise ValueError('no executable route: '+str(exc)) from exc
        selected=json.loads(Path(prepared).read_text())['review']['observation_index']
        mask=Path(detections).parent/f'mask_{selected:03}.png'
        bundle(report_path,Path(prepared),proof,Path(snapshot),mask,output/'bundle')
        return dict(bundle=str(output/'bundle'),report=str(report_path),moveit_starts=self.planner.starts,
                    contact_height_review=report.get('contact_height_review'))

    def load_checked(self, folder, *, binding):
        from home_robot_tasks.carry_bundle import load_bundle, validate_request_binding
        checked=load_bundle(Path(folder),for_execution=True)
        validate_request_binding(checked,**binding)
        return checked

    def planning_failure(self, report):
        """Decode potentially large evidence off the serial-owner thread."""
        from home_robot_tasks.pick_recovery import planning_recovery
        data=json.loads(Path(report).read_text())
        return dict(status=data.get('status'), recovery=planning_recovery(data))

    def verification(self, snapshot, bundle, reference_tcp, current_tcp, contact, empty, prompt):
        import numpy as np
        from PIL import Image
        from home_perception.geometry.plane import backproject
        from home_robot_tasks.carry_bundle import load_bundle
        from home_robot_tasks.pick_rgbd_verification import compare_clouds
        from tools.diagnostics.replay_s3_grasp import verified_mask
        from home_robot_tasks.snapshot_io import hashes
        snapshot=Path(snapshot)
        _, _, detected_folder=self.detector.analyze_snapshot(snapshot,prompt)
        detection_path=detected_folder/'detections.json'
        detection=json.loads(detection_path.read_text())
        if detection['source_sha256']!=hashes(snapshot):
            raise ValueError('verification image changed during inference')
        program,model,_,(_,reference_depth),mask,_,meta=load_bundle(Path(bundle),for_execution=True)
        targets=[d for d in detection['detections'] if d['label']==prompt and d['score']>=.5]
        result=dict(state='UNKNOWN',reason='TARGET_MISSING_OR_AMBIGUOUS')
        if len(targets)==1:
            target=targets[0]
            spec=target.get('segmentation',{})
            # Use the same checked mask representation as the grasp pipeline.
            mask_file=target.get('mask_file') or spec.get('mask_file')
            mask_hash=target.get('mask_sha256') or spec.get('sha256')
            if mask_file and mask_hash and spec.get('predicted_iou',0)>=.5:
                current_meta=json.loads((snapshot/'metadata.json').read_text())
                current_depth=np.asarray(Image.open(snapshot/'depth_aligned.png'),dtype=float)*current_meta['depth_scale_m']
                observed_mask=verified_mask(detection_path.parent/mask_file,mask_hash,current_depth.shape)
                def cloud(depth,intr,selected):
                    optical=backproject(depth,intr,selected & (depth>0))
                    return (np.asarray(model['profile']['base_from_optical'])@np.c_[optical,np.ones(len(optical))].T).T[:,:3]
                result=compare_clouds(cloud(reference_depth,meta['aligned_depth_intrinsics'],mask),
                    cloud(current_depth,current_meta['aligned_depth_intrinsics'],observed_mask),
                    reference_tcp,current_tcp,contact=contact,empty=empty)
        return dict(result,detections=str(detection_path))

    def close(self):
        self.planner.close()
