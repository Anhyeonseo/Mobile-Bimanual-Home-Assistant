"""L2 declared-device contract; absent devices never acquire synthetic feedback."""
from dataclasses import dataclass

from ..fetch import InvalidTask
from ..navigation import identifier
from .contracts import finite


@dataclass(frozen=True)
class Device:
    name: str
    bus: str
    motor_ids: tuple
    joint_names: tuple = ()

    def __post_init__(self):
        identifier(self.name, 'device'); identifier(self.bus, 'bus')
        if not self.motor_ids or any(type(v) is not int or not 1 <= v <= 253 for v in self.motor_ids):
            raise InvalidTask('valid motor IDs required')
        if len(set(self.motor_ids)) != len(self.motor_ids):
            raise InvalidTask('duplicate motor ID')
        if self.joint_names and len(self.joint_names) != len(self.motor_ids):
            raise InvalidTask('joint/motor count mismatch')
        for name in self.joint_names: identifier(name, 'joint name')


class DeviceRegistry:
    def __init__(self, devices):
        if not isinstance(devices, (list, tuple)) or not devices or any(not isinstance(d, Device) for d in devices):
            raise InvalidTask('explicit device list required')
        self.devices = {d.name: d for d in devices}
        addresses = [(d.bus, mid) for d in devices for mid in d.motor_ids]
        names = [j for d in devices for j in d.joint_names]
        if len(self.devices) != len(devices) or len(addresses) != len(set(addresses)) or len(names) != len(set(names)):
            raise InvalidTask('duplicate device, bus address or joint name')

    def evaluate(self, observations, *, now_s, maximum_age_s):
        now_s = finite(now_s, 'now', 0)
        maximum_age_s = finite(maximum_age_s, 'feedback maximum age', .001, 10)
        if not isinstance(observations, dict) or set(observations) - set(self.devices):
            raise InvalidTask('unknown device feedback')
        result = {}
        for name, device in self.devices.items():
            value = observations.get(name)
            if value is None:
                result[name] = {'available': False, 'reason': 'declared_device_missing'}
                continue
            if not isinstance(value, dict) or set(value) != {'observed_s', 'motor_ids', 'healthy', 'positions_rad'}:
                raise InvalidTask('invalid device feedback')
            age = now_s-finite(value['observed_s'], 'feedback time', 0)
            if tuple(value['motor_ids']) != device.motor_ids or type(value['healthy']) is not bool:
                raise InvalidTask('device identity mismatch')
            positions = value['positions_rad']
            if not isinstance(positions, dict) or set(positions) != set(device.joint_names):
                raise InvalidTask('installed joint feedback required')
            for v in positions.values(): finite(v, 'joint feedback')
            good = 0 <= age <= maximum_age_s and value['healthy']
            result[name] = {'available': good, 'reason': '' if good else 'stale_or_faulted',
                            'positions_rad': dict(positions) if good else None}
        return {'ready': all(v['available'] for v in result.values()), 'devices': result,
                'installed_devices': sorted(self.devices)}
