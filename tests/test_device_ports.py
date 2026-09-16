"""Actual task device ports over injected clients; no serial device is opened."""
from types import SimpleNamespace
import pytest
from home_robot_tasks.device_ports import LiftPort, ResidentArmPort
from home_robot_tasks.execution import Feedback


class Stop:
    def __init__(self):
        self.calls, self.confirmed, self.fail_once = 0, False, True

    def request(self, *args):
        self.calls += 1
        if self.fail_once:
            self.fail_once = False
            raise RuntimeError('stop ACK lost')

    def poll(self, g, t):
        return Feedback(g, t, 'STOPPED' if self.confirmed else 'RUNNING', {})


def make(kind):
    stop = Stop()
    if kind == 'lift':
        status = SimpleNamespace(session=4, flags=3, state=3, target_um=100,
                                 holding=lambda: False)
        client = SimpleNamespace(mobile=SimpleNamespace(boot_id=1), status=lambda: status,
                                 move=lambda *a: None, keepalive=lambda: None)
        port = LiftPort(client, lambda: 1., stop)
        port.start('run/prepare/1', {'height_um': 100})
    else:
        from so101_arm_bridge.bimanual_stream_adapter import TimedJointPoint
        client = SimpleNamespace(state=SimpleNamespace(value='ready'),
            feedback_snapshot=lambda: None, start=lambda *a, **k: None,
            poll=lambda *a: None, heartbeat_required=True, keepalive=lambda: None)
        port = ResidentArmPort(client, lambda *a: (TimedJointPoint(0., (0.,)*12),
            TimedJointPoint(1., (0.,)*12)), stop, lambda: 1.)
        port.start('run/prepare/1', {})
        client.state.value = 'running'
    return port, client, stop


@pytest.mark.parametrize('kind', ['lift', 'arm'])
def test_lost_stop_ack_retried_and_owner_kept_until_confirmed(kind):
    port, client, stop = make(kind)
    with pytest.raises(RuntimeError): port.cancel('run/prepare/1')
    assert port.owner == 'run/prepare/1'
    assert port.poll('run/prepare/1').state == 'RUNNING' and stop.calls == 2
    stop.confirmed = True
    assert port.poll('run/prepare/1').state == 'CANCELLED'
    assert port.owner is None


def test_lift_requires_homing_and_does_not_accept_wrong_target_hold():
    port, client, stop = make('lift')
    client.status().holding = lambda: True
    client.status().target_um = 101
    assert port.poll('run/prepare/1').state == 'RUNNING'
    client.status().target_um = 100
    assert port.poll('run/prepare/1').state == 'SUCCEEDED'
    client.status().flags = 0
    with pytest.raises(ValueError): port.start('run/lift2', {'height_um': 100})
    assert port.owner is None
