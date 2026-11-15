import json
import os
import subprocess
import sys
import time

import pytest

from tools.lib.stage_test_suite import junit_counts, language_acceptance, run_suite, ros_environment, run_logged


def test_failed_or_skipped_suite_is_not_counted_as_complete(tmp_path):
    path=tmp_path/'suite.xml'
    path.write_text('<testsuites><testsuite tests="3" failures="1" errors="0" skipped="1"/></testsuites>')
    assert junit_counts(path)==dict(tests=3,failures=1,errors=0,skipped=1)


@pytest.mark.parametrize('unsafe,scope_wrong,wrong,issues,accepted', [
    ([],0,[],[],True), (['bad'],0,[],[],False), ([],1,[],[],False),
    ([],0,['destination'],[],False), ([],0,['destination'],['clarify'],True),
])
def test_language_gate_preserves_scope_and_clear_request_accuracy(tmp_path,unsafe,scope_wrong,wrong,issues,accepted):
    path=tmp_path/'report.json'
    path.write_text(json.dumps(dict(complete=True,summary=dict(scope_cases=1,scope_wrong=scope_wrong,
        unsupported_marked_executable=unsafe),rows=[dict(intent=dict(issues=issues),wrong=wrong)])))
    assert language_acceptance(path)[0] is accepted


def test_incomplete_language_report_does_not_pass(tmp_path):
    path=tmp_path/'report.json';path.write_text('{"complete":false}')
    with pytest.raises(ValueError,match='incomplete'):
        language_acceptance(path)


def test_suite_will_not_replace_previous_records_or_accept_remote_language(tmp_path):
    with pytest.raises(FileExistsError):
        run_suite(tmp_path)
    with pytest.raises(ValueError):
        run_suite(tmp_path/'new',language_url='http://192.168.35.2:18767')
    assert not (tmp_path/'new').exists()


def test_ros_tests_do_not_inherit_robot_discovery_configuration():
    original=dict(ROS_DOMAIN_ID='0',ROS_DISCOVERY_SERVER='robot.example:11811',
                  ROS_STATIC_PEERS='192.168.35.2',CYCLONEDDS_URI='remote-profile.xml')
    env=ros_environment(original)
    assert env['ROS_DOMAIN_ID']!='0' and env['ROS_LOCALHOST_ONLY']=='1'
    assert not {'ROS_DISCOVERY_SERVER','ROS_STATIC_PEERS','CYCLONEDDS_URI'} & env.keys()
    assert original['ROS_DOMAIN_ID']=='0'


def test_logged_child_preserves_failure_exit_code(tmp_path):
    with (tmp_path/'log').open('w') as stream:
        assert run_logged([sys.executable,'-c','print("failed"); raise SystemExit(7)'],
                          dict(os.environ),stream,2)==7
    assert (tmp_path/'log').read_text().strip()=='failed'


def test_timeout_cleans_up_test_descendants(tmp_path):
    marker=tmp_path/'leaked'
    child=f'from pathlib import Path; import time; time.sleep(.6); Path({str(marker)!r}).write_text("bad")'
    parent=f'import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",{child!r}]); time.sleep(10)'
    with (tmp_path/'log').open('w') as stream:
        with pytest.raises(subprocess.TimeoutExpired):
            run_logged([sys.executable,'-c',parent],dict(os.environ),stream,.15)
    time.sleep(.7)
    assert not marker.exists()


@pytest.mark.parametrize('code,skipped,passed',[(0,0,True),(1,0,False),(0,1,False)])
def test_report_and_pending_cases_are_written_without_promoting_hardware(tmp_path,monkeypatch,code,skipped,passed):
    from pathlib import Path
    from tools.lib import stage_test_suite as suite, integration_rehearsal
    monkeypatch.setattr(suite,'GROUPS',{'s5_application':['fixture']})
    monkeypatch.setattr(suite,'controller_sources',lambda:{'example':'unchanged'})
    monkeypatch.setattr(suite,'inspect_assets',lambda stage:dict(passed=True))
    monkeypatch.setattr(integration_rehearsal,'SCENARIOS',())
    def run(arguments,env,stream,timeout):
        xml=Path(arguments[arguments.index('--junitxml')+1])
        xml.write_text(f'<testsuites><testsuite tests="2" skipped="{skipped}"/></testsuites>')
        return code
    monkeypatch.setattr(suite,'run_logged',run)
    output=tmp_path/'run'
    report=suite.run_suite(output,stage='s5')
    assert report['complete'] and report['passed'] is passed
    assert report['physical_test_ready'] is False
    assert report['language_model']=='NOT_RUN'
    cases=json.loads((output/'physical-cases.NOT_RUN.json').read_text())
    assert len(cases['cases'])==42 and all(c['status']=='NOT_RUN' for c in cases['cases'])
    assert (output/'summary.md').is_file()
    assert json.loads((output/'report.json').read_text())['passed'] is passed
