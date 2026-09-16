"""Explicit Cartesian MoveIt request and checked 12-axis resident trajectory map."""
import math
from .fetch import InvalidTask
from .navigation import number


class MoveItBridge:
    def __init__(self, joint_names, minimum_rad, maximum_rad, lift_height_m, *, maximum_speed_rad_s=.5):
        if len(joint_names)!=12 or len(set(joint_names))!=12 or len(minimum_rad)!=12 or len(maximum_rad)!=12:
            raise InvalidTask('twelve named joint limits required')
        self.names,self.low,self.high = tuple(joint_names),tuple(minimum_rad),tuple(maximum_rad)
        for a,b in zip(self.low,self.high):
            if number(a,'minimum')>=number(b,'maximum'):raise InvalidTask('invalid joint range')
        self.lift=lift_height_m;self.speed=number(maximum_speed_rad_s,'planning speed')
        if not 0<self.speed<=1:raise InvalidTask('bounded planning speed required')

    def request(self, parameters):
        from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        task,context=parameters['task'],parameters['context']
        arm=task['arm']
        if arm not in ('left','right') or task['target']['frame_id']!=context.frame_id:
            raise InvalidTask('planning arm/frame mismatch')
        q=task['quaternion_xyzw'];xyz=task['position_m']
        if len(q)!=4 or len(xyz)!=3 or abs(sum(number(v,'rotation')**2 for v in q)-1)>1e-6:
            raise InvalidTask('explicit finite Cartesian pose required')
        request=MotionPlanRequest();request.group_name=arm+'_arm';request.planner_id='RRTConnect'
        request.allowed_planning_time=1.;request.num_planning_attempts=1
        request.max_velocity_scaling_factor=.2;request.max_acceleration_scaling_factor=.2
        request.start_state.joint_state.name=list(self.names)+['lift_joint','left_wheel_joint','back_wheel_joint','right_wheel_joint']
        request.start_state.joint_state.position=list(context.anchor_rad)+[number(self.lift(),'lift height'),0.,0.,0.]
        # Supply measured joints while inheriting the scene's attached objects.
        # A full replacement with an empty attachment list would erase the load
        # from collision checks on the next retreat/placement plan.
        request.start_state.is_diff=True
        request.workspace_parameters.header.frame_id=context.frame_id
        request.workspace_parameters.min_corner.x=-2.;request.workspace_parameters.min_corner.y=-2.;request.workspace_parameters.min_corner.z=-1.
        request.workspace_parameters.max_corner.x=2.;request.workspace_parameters.max_corner.y=2.;request.workspace_parameters.max_corner.z=2.
        constraints=Constraints()
        p=PositionConstraint();p.header.frame_id=context.frame_id;p.link_name=arm+'_gripper_frame_link';p.weight=1.
        sphere=SolidPrimitive();sphere.type=SolidPrimitive.SPHERE;sphere.dimensions=[.005]
        pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=(number(v,'target') for v in xyz)
        p.constraint_region.primitives=[sphere];p.constraint_region.primitive_poses=[pose]
        o=OrientationConstraint();o.header.frame_id=context.frame_id;o.link_name=p.link_name;o.weight=1.
        o.orientation.x,o.orientation.y,o.orientation.z,o.orientation.w=map(float,q)
        o.absolute_x_axis_tolerance=o.absolute_y_axis_tolerance=o.absolute_z_axis_tolerance=.1
        constraints.position_constraints=[p];constraints.orientation_constraints=[o]
        request.goal_constraints=[constraints]
        return request

    def map(self, trajectory, context):
        from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
        jt=trajectory.joint_trajectory;names=tuple(jt.joint_names)
        if not names or len(names)!=len(set(names)) or not set(names)<=set(self.names):
            raise InvalidTask('unknown/duplicate trajectory joints')
        arms={n.split('_')[0] for n in names}
        if len(arms)!=1 or any(n.endswith('gripper_joint') for n in names):
            raise InvalidTask('stationary single-arm planning expected')
        arm=next(iter(arms))
        expected={n for n in self.names if n.startswith(arm+'_') and not n.endswith('gripper_joint')}
        if set(names)!=expected or not 2<=len(jt.points)<=20000:
            raise InvalidTask('complete finite arm plan required')
        mapped=[];last_t=-1.;last_q=None
        for p in jt.points:
            t=p.time_from_start.sec+p.time_from_start.nanosec/1e9
            if not math.isfinite(t) or t<0 or t<=last_t or len(p.positions)!=len(names):
                raise InvalidTask('invalid trajectory time/vector')
            positions=list(context.anchor_rad)
            for n,v in zip(names,p.positions):positions[self.names.index(n)]=number(v,'planned joint')
            if any(not a<=v<=b for a,v,b in zip(self.low,positions,self.high)):raise InvalidTask('planned joint limit')
            if last_q is None:
                if any(abs(a-b)>.02 for a,b in zip(positions,context.anchor_rad)) or t>.001:
                    raise InvalidTask('plan does not begin at measured anchor')
            elif any(abs(a-b)/(t-last_t)>self.speed+1e-6 for a,b in zip(positions,last_q)):
                raise InvalidTask('planned speed exceeds commissioned bound')
            # Preserve the opposite arm/gripper exactly; grid resampling below.
            mapped.append((t,tuple(positions)));last_t=t;last_q=positions
        result=[TimedJointPoint(50,tuple(context.anchor_rad))]
        segment=1
        for ms in range(50,math.ceil(mapped[-1][0]*1000/50)*50+1,50):
            t=min(ms/1000,mapped[-1][0])
            while segment<len(mapped)-1 and mapped[segment][0]<t:segment+=1
            a,qa=mapped[segment-1];b,qb=mapped[segment];alpha=(t-a)/(b-a)
            result.append(TimedJointPoint(ms+50,tuple(x+(y-x)*alpha for x,y in zip(qa,qb))))
            if len(result)>20000:raise InvalidTask('resampled trajectory too long')
        for a,b in zip(result,result[1:]):
            if any(abs(x-y)/((b.offset_ms-a.offset_ms)/1000)>self.speed+1e-6 for x,y in zip(a.positions_rad,b.positions_rad)):
                raise InvalidTask('resampled anchor transition exceeds speed bound')
        return tuple(result)
