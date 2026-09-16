"""Bounded object attach/detach transactions with Apply + Get readback.

The world pose on release must come from the caller's fresh observation/TF, not
the commanded hand pose. Only explicit gripper touch links are permitted. A
lost write acknowledgement is ambiguous and locks this port until recovery;
cancellation never rolls back a possibly held object's collision geometry.
"""
from copy import deepcopy
from .fetch import InvalidTask
from .navigation import identifier, number
from .skill_ports import PortResult


def validate_scene_change(p, now, *, gripper_links, world_frame, maximum_age_s):
    fields = {"operation", "object_id", "arm", "link_name", "frame_id", "position_m",
              "quaternion_xyzw", "size_m", "touch_links", "observed_s"}
    if not isinstance(p, dict) or set(p) != fields or p["operation"] not in {"attach", "detach"}:
        raise InvalidTask("complete object scene change required")
    identifier(p["object_id"], "scene object")
    arm = p["arm"]
    if arm not in gripper_links or p["link_name"] not in gripper_links[arm]:
        raise InvalidTask("unknown gripper attachment link")
    touches = p["touch_links"]
    if (not isinstance(touches, (list, tuple)) or not touches or len(set(touches)) != len(touches)
            or not set(touches) <= set(gripper_links[arm]) or p["link_name"] not in touches):
        raise InvalidTask("touch links must be confined to the selected gripper")
    expected_frame = p["link_name"] if p["operation"] == "attach" else world_frame
    if p["frame_id"] != expected_frame:
        raise InvalidTask("explicit link-relative attachment or world release pose required")
    for key, count in (("position_m", 3), ("quaternion_xyzw", 4), ("size_m", 3)):
        if not isinstance(p[key], (list, tuple)) or len(p[key]) != count:
            raise InvalidTask("invalid collision geometry")
        for value in p[key]:
            number(value, key)
    if any(not 0 < d <= 1 for d in p["size_m"]):
        raise InvalidTask("bounded box geometry required")
    if abs(sum(v*v for v in p["quaternion_xyzw"]) - 1) > 1e-6:
        raise InvalidTask("unit collision quaternion required")
    age = number(now, "scene clock") - number(p["observed_s"], "scene observation")
    if p["observed_s"] < 0 or not 0 <= age <= maximum_age_s:
        raise InvalidTask("stale scene pose")
    return deepcopy(p)


class SceneUpdatePort:
    """Client contract: ready(), apply(change)->future, read()->future, matches()."""
    def __init__(self, client, clock, *, gripper_links, world_frame="base_link", maximum_age_s=.5):
        self.client, self.clock = client, clock
        self.world = identifier(world_frame, "scene world frame")
        self.age = number(maximum_age_s, "scene age")
        if not 0 < self.age <= 2 or set(gripper_links) != {"left", "right"}:
            raise InvalidTask("explicit two-arm gripper links and pose age required")
        self.links = {}
        for arm, links in gripper_links.items():
            if not isinstance(links, (list, tuple)) or not 1 <= len(links) <= 8:
                raise InvalidTask("bounded gripper links required")
            self.links[arm] = tuple(identifier(v, "gripper link") for v in links)
        if set(self.links["left"]) & set(self.links["right"]):
            raise InvalidTask("gripper links must belong to separate arms")
        self.entries = {}
        self.owner = None
        self.fault = None
        self.revision = 0

    def ready(self):
        return self.owner is None and self.fault is None and self.client.ready()

    def start(self, goal_id, parameters):
        if not self.ready() or goal_id in self.entries or len(self.entries) >= 128:
            raise InvalidTask("scene owned, faulted or duplicate")
        p = validate_scene_change(parameters, self.clock(), gripper_links=self.links,
                                  world_frame=self.world, maximum_age_s=self.age)
        e = dict(change=p, phase="apply", future=None, state="RUNNING", reason="", cancel=False)
        self.entries[goal_id], self.owner = e, goal_id
        try:
            e["future"] = self.client.apply(p)
        except Exception as error:
            # Dispatch may have reached the service. No automatic retransmit.
            self.fault = e["reason"] = "scene_dispatch_unconfirmed:" + str(error)

    def cancel(self, goal_id):
        self.entries[goal_id]["cancel"] = True

    def poll(self, goal_id):
        e = self.entries[goal_id]
        if e["state"] != "RUNNING":
            return PortResult(e["state"], e["reason"])
        if self.fault or e["future"] is None or not e["future"].done():
            return PortResult("RUNNING", e["reason"])
        try:
            value = e["future"].result()
            if e["phase"] == "apply":
                if value is not True:
                    # Even a negative ACK does not prove the previous scene.
                    e["reason"] = "scene_apply_rejected"
                e["phase"] = "read"
                e["future"] = self.client.read()
                return PortResult("RUNNING", e["reason"])
            if self.client.matches(e["change"], value) is not True:
                self.fault = e["reason"] = "scene_readback_mismatch"
                e["state"] = "FAILED"
            elif e["reason"]:
                self.fault = e["reason"]
                e["state"] = "FAILED"
            else:
                self.revision += 1
                e["state"] = "CANCELLED" if e["cancel"] else "SUCCEEDED"
            self.owner = None
        except Exception as error:
            # A failed/locally cancelled future is not a server completion ACK.
            self.fault = e["reason"] = "scene_result_unconfirmed:" + str(error)
        return PortResult(e["state"], e["reason"])


class _MappedFuture:
    def __init__(self, future, transform):
        self.future, self.transform = future, transform

    def done(self):
        return self.future.done()

    def result(self):
        return self.transform(self.future.result())


class MoveItSceneClient:
    """Real ROS services; creates neither a MoveIt server nor a motor command."""
    def __init__(self, node, *, apply_service="/apply_planning_scene", get_service="/get_planning_scene"):
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        self.apply_type, self.get_type = ApplyPlanningScene, GetPlanningScene
        self.writer = node.create_client(ApplyPlanningScene, apply_service)
        self.reader = node.create_client(GetPlanningScene, get_service)

    def ready(self):
        return self.writer.service_is_ready() and self.reader.service_is_ready()

    @staticmethod
    def _box(p):
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        obj = CollisionObject()
        obj.id, obj.header.frame_id, obj.operation = p["object_id"], p["frame_id"], CollisionObject.ADD
        obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = map(float, p["position_m"])
        (obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z,
         obj.pose.orientation.w) = map(float, p["quaternion_xyzw"])
        primitive = SolidPrimitive(); primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(map(float, p["size_m"]))
        identity = Pose(); identity.orientation.w = 1.
        obj.primitives, obj.primitive_poses = [primitive], [identity]
        return obj

    def apply(self, p):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        request = self.apply_type.Request()
        request.scene.is_diff = request.scene.robot_state.is_diff = True
        box = self._box(p)
        attached = AttachedCollisionObject(); attached.link_name = p["link_name"]
        if p["operation"] == "attach":
            attached.object, attached.touch_links = box, list(p["touch_links"])
            # MoveIt attaches first and removes the matching world object itself.
            # An additional world REMOVE in the same diff then reports failure
            # because the object is already gone (verified on Jazzy 2.12.4).
            world = []
        else:
            attached.object.id = p["object_id"]; attached.object.operation = CollisionObject.REMOVE
            world = [box]
        request.scene.world.collision_objects = world
        request.scene.robot_state.attached_collision_objects = [attached]
        return _MappedFuture(self.writer.call_async(request), lambda r: r.success)

    def read(self):
        from moveit_msgs.msg import PlanningSceneComponents
        request = self.get_type.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        return _MappedFuture(self.reader.call_async(request), lambda r: r.scene)

    @staticmethod
    def matches(p, scene):
        world = [o for o in scene.world.collision_objects if o.id == p["object_id"]]
        attached = [a for a in scene.robot_state.attached_collision_objects if a.object.id == p["object_id"]]
        if p["operation"] == "attach":
            if world or len(attached) != 1 or attached[0].link_name != p["link_name"]:
                return False
            if set(attached[0].touch_links) != set(p["touch_links"]):
                return False
            obj = attached[0].object
        else:
            if attached or len(world) != 1:
                return False
            obj = world[0]
        if (obj.header.frame_id != p["frame_id"] or len(obj.primitives) != 1
                or len(obj.primitive_poses) != 1 or obj.primitives[0].type != 1
                or len(obj.primitives[0].dimensions) != 3
                or obj.meshes or obj.planes or obj.subframe_names):
            return False
        # Compare the composed object/shape pose, since MoveIt may normalize
        # where it stores the primitive transform on readback.
        def xyz(pose): return [pose.position.x, pose.position.y, pose.position.z]
        def quat(pose): return [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        def multiply(a, b):
            av, bv = np.asarray(a[:3]), np.asarray(b[:3])
            return np.append(a[3]*bv + b[3]*av + np.cross(av, bv), a[3]*b[3] - np.dot(av, bv))
        import numpy as np
        outer, inner = np.asarray(quat(obj.pose)), np.asarray(quat(obj.primitive_poses[0]))
        if any(not np.isfinite(q).all() or abs(np.dot(q, q)-1) > 1e-6 for q in (outer, inner)):
            return False
        conjugate = np.append(-outer[:3], outer[3])
        position = multiply(multiply(outer, [*xyz(obj.primitive_poses[0]), 0]), conjugate)[:3] + xyz(obj.pose)
        rotation = multiply(outer, inner)
        expected = np.asarray(p["quaternion_xyzw"])
        return bool(np.allclose(obj.primitives[0].dimensions, p["size_m"], rtol=0, atol=1e-6)
                    and np.allclose(position, p["position_m"], rtol=0, atol=1e-6)
                    and min(np.linalg.norm(rotation-expected), np.linalg.norm(rotation+expected)) <= 1e-6)
