"""Resolve a catalog viewpoint into checked arm/lift/Nav2 phases."""

from .workflow_ports import Phase
from .navigation import NavigateRequest
from .fetch import InvalidTask


class ViewMotionBuilder:
    def __init__(
        self, navigation_map, current_xy, arm, lift, navigation, *, transport_height_um
    ):
        if type(transport_height_um) is not int:
            raise InvalidTask("explicit transport height required")
        self.map, self.xy, self.arm, self.lift, self.navigation = (
            navigation_map,
            current_xy,
            arm,
            lift,
            navigation,
        )
        self.transport_height = transport_height_um

    def __call__(self, view):
        if (
            set(view) != {"navigate_request", "height_um", "transport_arm_route"}
            or type(view["height_um"]) is not int
        ):
            raise InvalidTask("complete registered viewpoint required")
        plan = self.map.plan(
            NavigateRequest.from_dict(view["navigate_request"]), self.xy()
        )
        stationary = ("base_stopped", "lift_stopped")
        arms = ("arms_safe_for_lift",)
        travel = (
            "localized",
            "arm_in_transport_pose",
            "lift_in_transport_position",
            "lift_stopped",
        )
        return (
            Phase(
                "view_arm_transport",
                self.arm,
                view["transport_arm_route"],
                stationary,
                stationary,
                ("arm_in_transport_pose",),
                20,
            ),
            Phase(
                "view_lift_transport",
                self.lift,
                {"height_um": self.transport_height},
                (*stationary, *arms),
                ("base_stopped", *arms),
                ("lift_stopped", "lift_in_transport_position"),
                20,
            ),
            Phase(
                "view_navigate",
                self.navigation,
                plan,
                ("base_stopped", *travel),
                travel,
                ("localized_at_view", "base_stopped"),
                60,
            ),
            Phase(
                "view_lift_observe",
                self.lift,
                {"height_um": view["height_um"]},
                (*stationary, *arms),
                ("base_stopped", *arms),
                ("lift_stopped", "view_reached"),
                20,
            ),
        )
