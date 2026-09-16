"""ROS assembly over explicitly prepared clients. No discovery, ARM or homing.

Use a single-threaded ROS executor: state callbacks, runtime and shared serial
clients have one owner. This remains the simulation/commissioning boundary;
RobotApplication still rejects physical-mode execution.
"""
from dataclasses import fields
from .capture_port import InferencePort
from .device_ports import ResidentArmPort, LiftPort
from .fetch import InvalidTask
from .manipulation_port import MoveItPlanPort
from .nav2_port import Nav2Port
from .payload_monitor import GripMeasurement, PayloadObservation
from .robot_state import RobotObservation, EvidencePublisher
from .ros_rgbd_source import RosRGBDSource
from .runtime_factory import build_runtime


class RosEvidenceSource:
    def __init__(self,node,state,payload,clock,profile_sha256,*,namespace='robot_evidence',world_frame='map'):
        if not isinstance(profile_sha256, str) or len(profile_sha256) != 64 or any(c not in '0123456789abcdef' for c in profile_sha256):
            raise InvalidTask('SHA-256 sensor profile identity required')
        if payload.camera_frames is None:raise InvalidTask('explicit qualified payload camera frames required')
        self.world_frame = world_frame
        from so101_interfaces.msg import RobotStateEvidence,GripperObservation,PayloadVisualObservation
        self.node,self.state,self.payload,self.clock,self.profile=node,state,payload,clock,profile_sha256
        self.error=None
        self.subscriptions=[node.create_subscription(kind,namespace+'/'+topic,callback,10) for kind,topic,callback in (
            (RobotStateEvidence,'state',self.robot),(GripperObservation,'gripper',self.grip),
            (PayloadVisualObservation,'payload',self.vision))]

    def stamp(self,msg):
        if msg.profile_sha256!=self.profile:raise InvalidTask('sensor profile mismatch')
        ros_age=self.node.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        if not 0<=ros_age<=.5:raise InvalidTask('stale or future sensor message')
        return self.clock()-ros_age

    def robot(self,msg):
        try:
            if msg.header.frame_id != self.world_frame: raise InvalidTask('robot world frame mismatch')
            values={f.name:getattr(msg,f.name) for f in fields(RobotObservation) if f.name!='observed_s'}
            for k in ('xy_yaw','base_velocity','joints_rad'):values[k]=tuple(float(v) for v in values[k])
            self.state.observe(RobotObservation(observed_s=self.stamp(msg),**values))
        except Exception as error:self.error=str(error);self.state.fault=self.error

    def grip(self,msg):
        try:self.payload.observe_grip(GripMeasurement(msg.sequence,self.stamp(msg),msg.gap_mm,msg.effort_raw,msg.healthy),self.clock())
        except Exception as error:self.error=str(error);self.state.fault=self.error

    def vision(self,msg):
        try:self.payload.observe_vision(PayloadObservation(msg.sequence,self.stamp(msg),msg.object_id,msg.relation,
            msg.destination or None,msg.object_at_destination,msg.header.frame_id),self.clock())
        except Exception as error:self.error=str(error);self.state.fault=self.error


def build_ros_runtime(*,node,tf_buffer,world,navigation_map,catalog_document,state,payload,camera,detect,
                      moveit_bridge,arm_adapter,arm_route_builder,lift_client,stop_client,evidence_client,
                      clock,scene_revision,surface_observer,rgbd_topics,depth_scale_m,
                      navigation_action='/offline_nav2/navigate_to_pose',planning_service='/offline_moveit/plan_kinematic_path',
                      manipulation_factory=None, manipulation_bindings=None, surface_sampler=None,
                      navigation_port=None, navigation_maintenance=(), navigation_observations=()):
    if manipulation_bindings is not None:
        if manipulation_factory is not None:raise InvalidTask('select one manipulation composition')
        manipulation_factory=manipulation_bindings.factory
    source=RosRGBDSource(node,tf_buffer,camera.policy,state.motion_at,clock,depth_scale_m=depth_scale_m,**rgbd_topics)
    detector=InferencePort(detect)
    planner=MoveItPlanPort(node,moveit_bridge.request,moveit_bridge.map,service=planning_service)
    holder={}
    def arm(stop):
        port=ResidentArmPort(arm_adapter,arm_route_builder,stop,clock);holder['arm']=port;return port
    try:
        assembly=build_runtime(world=world,navigation_map=navigation_map,catalog_document=catalog_document,state=state,payload=payload,
            camera=camera,source=source,detector=detector,navigation=navigation_port if navigation_port is not None else Nav2Port(node,navigation_action),arm=arm,
            lift=lambda stop:LiftPort(lift_client,clock,stop),planner=planner,surface_observer=surface_observer,
            stop_client=stop_client,clock=clock,scene_revision=scene_revision,
            maintenance=(lambda:holder['arm'].maintain(),)+tuple(navigation_maintenance),
            observations=(EvidencePublisher(state,payload,evidence_client,clock),)+tuple(navigation_observations),
            manipulation_factory=manipulation_factory,surface_sampler=surface_sampler)
    except Exception:
        detector.close();raise
    assembly.ros_source=source;assembly.inference=detector
    return assembly
