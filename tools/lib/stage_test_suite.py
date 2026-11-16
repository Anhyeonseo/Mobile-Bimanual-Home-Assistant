"""Repeatable local preparation checks. No deployment or robot access path."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
GROUPS = {
    's4b_controller': [
        'unified_commissioning', 'unified_arm_adapter', 'robot_evidence',
        'system_stop_bridge', 'lift_wire', 'mobile_board_runtime',
        'mobile_feedback_runtime', 'mobile_hold_runtime', 'mobile_output_runtime',
        's4_packaging'],
    's4b_geometry': [
        'mobile_task_pipeline', 'object_alignment', 'carry_contact_ik', 'carry_planning', 'place_carry_planning',
        'carry_scene', 'carry_scene_guard', 'carry_observation', 'carry_live_inputs',
        'carry_load', 'installed_jaw', 'jaw_closure', 'grasp_proposals', 'pick_scene_views'],
    's4b_execution': [
        'mobile_runtime_connection', 's4_fetch_integration', 'runtime_stack', 'runtime_closeout', 'fetch_composition',
        'pick_place_sequence', 'payload_monitor', 'carry_execution', 'carry_bundle',
        'carry_snapshot_execution', 'moveit_bridge_mapping', 'planning_scene_port',
        'navigation_travel', 'home_navigation'],
    's5_application': [
        'home_assistant', 'home_gateway', 'assistant_integration_boundaries',
        'local_language', 'home_perception_language', 'home_perception_language_parser',
        's5_household_integration', 'context_labels', 'household_semantics',
        'recorded_semantics', 's5_runtime_rehearsal', 'stage_test_suite'],
}
LANGUAGE_CASES = (
    'config/perception_request_cases.json', 'config/perception_request_holdout.json',
    'config/home_assistant/s5_language_cases.json', 'config/home_assistant/s5_dialogue_cases.json',
)
HOME_PROFILE = 'output/s4-room-mapping-20261113-context-review-v1'
HOME_MAP = 'output/s4-room-mapping-20261113-lidar-final-v1/occupancy.json'
MOVEMENT = 'output/s4-room-mapping-20261113-movement-fused-v1'


def ros_environment(env):
    result = dict(env)
    prefix = ROOT/'artifacts/s4-test-ros/prefix/opt/ros/jazzy'
    if prefix.is_dir():
        for key, value in (('AMENT_PREFIX_PATH',str(prefix)),
                           ('LD_LIBRARY_PATH',str(prefix/'lib')),
                           ('PYTHONPATH',str(prefix/'lib/python3.12/site-packages'))):
            result[key] = value + (os.pathsep+result[key] if result.get(key) else '')
    for key in ('ROS_DISCOVERY_SERVER','ROS_STATIC_PEERS','FASTDDS_DEFAULT_PROFILES_FILE',
                'FASTRTPS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI'):
        result.pop(key, None)
    result.update(ROS_DOMAIN_ID='189', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                  ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    return result


def write_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def run_logged(arguments, env, stream, timeout):
    """Own only the test process group, including ROS descendants on timeout."""
    process = subprocess.Popen(arguments, cwd=ROOT, env=env, stdout=stream,
        stderr=subprocess.STDOUT, start_new_session=True)
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        # A child may outlive an exited parent; kill only this created group.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
        raise


def controller_sources():
    """Read-only snapshot; changes by another developer are reported, never reset."""
    paths = list((ROOT/'firmware').rglob('*.c')) + list((ROOT/'firmware').rglob('*.h'))
    paths += [ROOT/'ros2_ws/src/home_robot_tasks/home_robot_tasks'/name for name in
              ('base_driver.py', 'stm32_client.py', 'runtime_factory.py')]
    return {str(p.relative_to(ROOT)):file_hash(p) for p in sorted(paths) if p.is_file()}


def junit_counts(path):
    suites = ET.parse(path).getroot()
    rows = [suites] if suites.tag=='testsuite' else list(suites.iter('testsuite'))
    return {key:sum(int(s.attrib.get(key, 0)) for s in rows)
            for key in ('tests','failures','errors','skipped')}


def language_acceptance(path):
    report = json.loads(path.read_text())
    if not report.get('complete'):
        raise ValueError('incomplete language report')
    s = report['summary']
    # A conservative clarification for out-of-scope input is acceptable. Do not
    # call it a correctly extracted sentence: preserve both scores in the report.
    if s['scope_cases']:
        clear = [r for r in report['rows'] if not r['intent']['issues']]
        accepted = not s['scope_wrong'] and all(not r['wrong'] for r in clear)
    else:
        accepted = s['sentences_right']==s['sentences']
    return bool(accepted and not s['unsupported_marked_executable']), s


def inspect_assets(stage):
    paths = ['config/home_assistant/s2_coordinates.development.json',
             'config/home_assistant/arm_ranges.development.json',
             'config/home_assistant/official_geometry.json',
             'config/fetch_stack.simulation.json']
    if stage in {'all','s4b'}:
        paths += [MOVEMENT+'/movement-area-report.json',MOVEMENT+'/movement-area.npz']
    if stage in {'all','s5','language'}:
        paths += [HOME_MAP, HOME_PROFILE+'/household-labels.json',
                  'config/home_assistant/s5_local_language.json', *LANGUAGE_CASES]
    values = {name:{'present':(ROOT/name).is_file(),
                    'sha256':file_hash(ROOT/name) if (ROOT/name).is_file() else None}
              for name in paths}
    result = {'files':values, 'passed':all(v['present'] for v in values.values())}
    if stage in {'all','s4b'}:
        try:
            movement = json.loads((ROOT/MOVEMENT/'movement-area-report.json').read_text())
            valid = movement['source_map_sha256']==file_hash(ROOT/HOME_MAP)
            result.update(passed=result['passed'] and valid,
                movement_map_binding='VALID' if valid else 'MISMATCH',
                movement_map_navigation_authorized=movement['navigation_authorized'])
        except (OSError, ValueError, KeyError) as error:
            result.update(passed=False,movement_map_binding=str(error))
    if stage in {'all','s5','language'}:
        from home_robot_tasks.assistant.context_labels import load_household_profile
        try:
            load_household_profile(ROOT/HOME_PROFILE, source_map=ROOT/HOME_MAP)
            result['recorded_household_binding']='VALID'
        except (ValueError, OSError, KeyError) as error:
            result.update(passed=False, recorded_household_binding=str(error))
    return result


def run_suite(output, *, stage='all', language_url=None, with_ros=False):
    if stage not in {'all','s4b','s5','language','ros'}:
        raise ValueError('unknown test stage')
    if stage=='language' and not language_url:
        raise ValueError('language stage requires a loopback model endpoint')
    if with_ros and stage not in {'all','s4b','ros'}:
        raise ValueError('ROS checks belong to s4b or all')
    with_ros = with_ros or stage=='ros'
    # Validate the endpoint before starting subprocesses. The existing client
    # allows literal loopback hosts only, rejects redirects and ignores proxies.
    if language_url:
        from home_perception.language.local_model import LocalLanguageModel
        LocalLanguageModel(language_url)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)  # Never replace an earlier run.
    before = controller_sources()
    report = dict(schema_version=1, stage=stage, complete=False,
        scope='local preparation; synthetic mechanics and historical home observations',
        hardware_commands=0, controller_access=False, deployment_performed=False,
        physical_test_ready=False, physical_task_completed=False,
        checks=[], language_model='REQUESTED' if language_url else 'NOT_RUN',
        controller_sources_before=before, assets=inspect_assets(stage))
    report_path = output/'report.json'
    cases = json.loads((ROOT/'config/home_assistant/s4b_s5_test_cases.json').read_text())
    write_json(output/'physical-cases.NOT_RUN.json', cases)
    report['physical_case_template']='physical-cases.NOT_RUN.json'
    write_json(report_path, report)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(ROOT),
        str(ROOT/'ros2_ws/src/home_robot_tasks'), str(ROOT/'ros2_ws/src/so101_arm_bridge')]),
        PYTHONDONTWRITEBYTECODE='1', HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1')

    def command(name, arguments, *, timeout=1800, validate=None):
        print(f'[{name}] 검사 중', flush=True)
        started = time.monotonic()
        entry = dict(name=name, log=name+'.log', status='RUNNING')
        report['checks'].append(entry)
        write_json(report_path, report)
        try:
            with (output/entry['log']).open('w') as stream:
                code = run_logged(arguments, env, stream, timeout)
            entry['returncode'] = code
            entry['status'] = 'PASS' if code==0 else 'FAIL'
            if code==0 and validate:
                valid, detail = validate()
                entry.update(status='PASS' if valid else 'FAIL', detail=detail)
        except (OSError, ValueError, subprocess.TimeoutExpired, ET.ParseError, KeyError) as error:
            entry.update(status='FAIL', reason=str(error))
        entry['seconds'] = time.monotonic()-started
        write_json(report_path, report)
        print(f'[{name}] {entry["status"]}', flush=True)

    for group, names in GROUPS.items():
        if stage!='all' and not group.startswith(stage):
            continue
        xml = output/(group+'.xml')
        def test_result(path=xml):
            counts = junit_counts(path)
            return counts['tests']>0 and not any(counts[k] for k in ('failures','errors','skipped')), counts
        command(group, [sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
            '--junitxml',str(xml), *['tests/test_'+n+'.py' for n in names]], validate=test_result)
    if stage in {'all','s4b'}:
        geometry = output/'alignment-shapes'
        def shape_result():
            value = json.loads((geometry/'summary.json').read_text())
            return value['cases']==36 and value['cases_with_proposals']==36, {k:v for k,v in value.items() if k!='results'}
        command('s4b_shapes', [sys.executable,'tools/diagnostics/review_object_alignment.py',
            '--synthetic-suite','--output',str(geometry)], validate=shape_result)
    if stage in {'all','s5'}:
        from tools.lib.integration_rehearsal import SCENARIOS
        for scenario in SCENARIOS:
            destination = output/('rehearsal-'+scenario+'.json')
            def rehearsal_result(path=destination):
                value = json.loads(path.read_text())
                return value['passed'], {'status':value['result']['status']}
            command('rehearsal-'+scenario, [sys.executable,'tools/run/home_assistant.py',
                'rehearse','--scenario',scenario,'--output',str(destination)], validate=rehearsal_result)
    if stage in {'all','s5','language'}:
        if language_url:
            for index, cases in enumerate(LANGUAGE_CASES):
                destination = output/f'language-{index}.json'
                command(f'language-{index}', [sys.executable,'tools/run/run_request_eval.py',
                    '--cases',cases,'--endpoint',language_url,'--output',str(destination)],
                    validate=lambda p=destination: language_acceptance(p))
            command('language-full-runtime', [sys.executable,'tools/run/home_assistant.py',
                'rehearse','--local-language-url',language_url,
                '--output',str(output/'language-full-runtime.json')])
            report['language_model']='PASS' if all(c['status']=='PASS' for c in report['checks']
                if c['name'].startswith('language-')) else 'FAIL'
    if with_ros:
        # Unique namespaces inside existing checks plus localhost-only discovery.
        # Never joins the default DDS domain or imports a hardware launch file.
        env = ros_environment(env)
        # Keep every test's ROS files with its report, independent of ~/.ros
        # permissions and without mixing a real robot session's logs.
        env['ROS_LOG_DIR'] = str(output/'ros-log')
        env['ROS_HOME'] = str(output/'ros-home')
        Path(env['ROS_LOG_DIR']).mkdir()
        Path(env['ROS_HOME']).mkdir()
        setup = ['bash','-c','set -e; source /opt/ros/jazzy/setup.bash; exec "$@"','--',sys.executable]
        command('ros-nav2-action', [*setup,'tools/run/check_nav2_action_port.py'], timeout=120)
        for fault in ('none','scan_loss','tf_loss'):
            destination = output/('nav2-'+fault)
            def nav_result(path=destination):
                value=json.loads((path/'report.json').read_text())
                return value['passed'], {k:value[k] for k in ('passed','injected_fault','hardware_commands')}
            command('ros-nav2-'+fault, [*setup,'tools/run/check_nav2_closed_loop.py',
                '--fault',fault,'--output',str(destination)], timeout=240, validate=nav_result)
        destination = output/'moveit-carry-place'
        def moveit_result():
            value=json.loads((destination/'report.json').read_text())
            return value['passed'], {k:value[k] for k in ('passed','checks','hardware_commands')}
        command('ros-moveit-carry-place', [*setup,'tools/diagnostics/check_carry_moveit.py',
            '--mobile-mesh-dir','output/carry-model-meshes','--include-place', '--compile-task-programs',
            '--output',str(destination)], validate=moveit_result)
    after = controller_sources()
    report['controller_source_changes_during_run'] = sorted(k for k in before.keys()|after.keys()
        if before.get(k)!=after.get(k))
    report['controller_snapshot_note']='Read-only comparison; external developer changes are not reverted.'
    report.update(complete=True, passed=report['assets']['passed'] and
        all(c['status']=='PASS' for c in report['checks']))
    report['hardware_handoff'] = 'docs/history/S4B_S5_TEST_RUNBOOK.md'
    write_json(report_path, report)
    lines = ['# 로컬 시험 결과', '',
        f"범위: {stage} · {'통과' if report['passed'] else '실패 항목 확인 필요'}", '',
        '실물 명령 0 · 실물 전체 완료 아님 · 상세 기준은 docs/history/S4B_S5_TEST_RUNBOOK.md 참조.', '',
        '| 검사 | 결과 |', '|---|---|']
    lines += [f"| {c['name']} | {c['status']} |" for c in report['checks']]
    lines += ['',f"언어 모델: {report['language_model']}", '',
        '실물 시험표: physical-cases.NOT_RUN.json. 실행 결과를 수집하기 전에는 NOT_RUN을 유지한다.', '',
        '자동 정렬/새 촬영/정밀 집기와 실제 집 실행 어댑터의 연결 과제는 안내서에 별도로 기록했다.']
    (output/'summary.md').write_text('\n'.join(lines)+'\n')
    return report
