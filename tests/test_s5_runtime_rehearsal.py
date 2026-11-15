import pytest

from tools.lib.integration_rehearsal import rehearse, SCENARIOS


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_text_confirmation_and_full_composed_runtime(scenario):
    report = rehearse(scenario)
    assert report['passed'], report['result']
    assert report['hardware_commands']==0 and not report['physical_task_completed']
    if scenario=='success':
        run = report['execution']
        assert run['completed_steps']==15 and len(run['manipulation_stages'])==11
        assert [x['operation'] for x in run['placement_history']]==['pick','place']
        assert report['object']['support_id']=='bedroom_drop_zone'
        assert report['object']['state']=='placed'
    elif scenario in {'load_loss','sensor_loss','reboot'}:
        assert report['execution']['control_owned']
        assert not any(x['phase'].startswith('place_') for x in report['execution']['manipulation_stages'])
    elif scenario=='pick_failure':
        assert not any(x['phase']=='pick_attach' for x in report['execution']['manipulation_stages'])
    elif scenario=='place_failure':
        assert report['object']['state']=='carried'
        assert not any(x['phase']=='place_detach' for x in report['execution']['manipulation_stages'])


def test_unknown_scenario_cannot_run():
    with pytest.raises(ValueError, match='unknown offline'):
        rehearse('execute-on-robot')
