from copy import deepcopy
import pytest

from home_robot_tasks.pick_table_clearance import require_table_clearance


def proof():
    stages = [dict(stage=name, minimum_by_finger={side: {'clearance_m': distance}
              for side in ('fixed','moving')}) for name, distance in
              [('open',.1),('pregrasp',.05),('approach',.0062),('close',.0062),('probe',.007),('lift',.012)]]
    return {'native_geometry':{'stock_finger_surfaces':{
        'approach_mode':'vertical','basis':'verified_official_STL_surface_vertices','stages':stages}}}


def test_inspection_checks_the_entire_executed_prefix_and_skips_descent():
    evidence=proof()
    require_table_clearance(evidence,pregrasp_check=True)
    for stage in (0,1):
        for side in ('fixed','moving'):
            bad=deepcopy(evidence)
            bad['native_geometry']['stock_finger_surfaces']['stages'][stage]['minimum_by_finger'][side]['clearance_m']=.029
            with pytest.raises(ValueError,match=side):
                require_table_clearance(bad,pregrasp_check=True)


@pytest.mark.parametrize('side', ['fixed','moving'])
def test_normal_pick_accepts_positive_gap_and_rejects_table_contact(side):
    evidence=proof()
    finger=evidence['native_geometry']['stock_finger_surfaces']['stages'][2]['minimum_by_finger'][side]
    finger['clearance_m']=.00074
    require_table_clearance(evidence,pregrasp_check=False)
    for contact in (0.0, -.001):
        finger['clearance_m']=contact
        with pytest.raises(ValueError,match=side):
            require_table_clearance(evidence,pregrasp_check=False)


def test_web_inspection_stops_before_publishing_second_capture(tmp_path):
    from types import SimpleNamespace
    from home_robot_tasks.pick_backend import PipelineBackend
    order = []
    folder = tmp_path/'pregrasp-check'/'rgbd-2'
    def execute(ctx, checked, *, pregrasp_check):
        assert pregrasp_check is True
        order.append('pregrasp')
        return folder, object(), {}
    def stop():
        order.append('stop')
        backend.hardware.stop_confirmed = True
    def compute(ctx, method, **kwargs):
        order.append(method)
        return str(folder/'planned-fingers.png') if method == 'pregrasp_overlay' else {}
    backend=PipelineBackend({'mode':'station'})
    backend.hardware=SimpleNamespace(profile={},session_id='active',execute=execute,abort=stop,stop_confirmed=False)
    backend._compute=compute
    ctx=SimpleNamespace(service=lambda:order.append('service'),check=lambda:None,artifact=lambda *args:None)
    result=backend.execute(ctx,dict(bundle='checked',prepared='prepared',snapshot='rgbd-1',pregrasp_check=True))
    assert result['status']=='PREGRASP_OBSERVED' and not result['physical_success']
    assert order==['service','load_checked','pregrasp','stop','save','pregrasp_overlay']
