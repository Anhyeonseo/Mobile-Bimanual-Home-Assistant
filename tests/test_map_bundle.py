from types import SimpleNamespace as N
import json
import pytest
from test_depth_archive import archive
from home_robot_tasks.fetch import InvalidTask
from home_robot_tasks.map_bundle import from_occupancy_grid, occupancy_values, nav2_image, export_scan, read_bundle
from home_robot_tasks.navigation_fusion import TravelEnvelope


def grid():
    return N(header=N(frame_id='map'), data=[0]*25,
        info=N(width=5,height=5,resolution=.1,
        origin=N(position=N(x=0.,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))))


def convert(msg):
    return from_occupancy_grid(msg,map_id='home',revision='slam-1',floor_id='floor',
        envelope=TravelEnvelope('fixture',.04,1.),clearance_m=0)


def test_grid_probabilities_and_image_y_axis_are_conservative():
    msg=grid();msg.data[:5]=[-1,0,20,65,100]
    doc=convert(msg)
    assert doc['rows'][0] == '?.?##'
    assert occupancy_values(doc)[:5] == [-1,0,-1,100,100]
    pixels=nav2_image(doc).split(b'\n',3)[3]
    assert list(pixels[-5:]) == [205,254,205,0,0]


@pytest.mark.parametrize('bad', ['rotation','probability','size','nan'])
def test_grid_rejects_geometry_it_cannot_represent(bad):
    msg=grid()
    if bad=='rotation':msg.info.origin.orientation.z=.1
    if bad=='probability':msg.data[0]=101
    if bad=='size':msg.info.width=257
    if bad=='nan':msg.info.origin.position.x=float('nan')
    with pytest.raises(InvalidTask):convert(msg)


def test_saved_map_bundle_roundtrip_and_tamper(tmp_path):
    service,_=archive(tmp_path/'world.db')
    try:
        service.scan.review('home-scan');service.scan.save('home-scan')
        dest=tmp_path/'bundle'
        manifest=export_scan(service.scan,'home-scan',dest)
        loaded,maps=read_bundle(dest)
        assert manifest == loaded and loaded['localization_required']
        assert maps['localization']['rows'][5][5]=='.'
        assert maps['navigation']['rows'][5][5]=='#'
        assert list((dest/'depth').glob('*.json'))
        with pytest.raises(InvalidTask):export_scan(service.scan,'home-scan',dest)
        (dest/'navigation.json').write_text('{}')
        with pytest.raises(InvalidTask,match='changed file'):read_bundle(dest)
    finally:service.store.close()


def test_bundle_cannot_reference_files_outside_it(tmp_path):
    service,_=archive(tmp_path/'world.db')
    try:
        service.scan.review('home-scan');service.scan.save('home-scan')
        dest=tmp_path/'bundle';export_scan(service.scan,'home-scan',dest)
        manifest=json.loads((dest/'manifest.json').read_text())
        manifest['files_sha256']['../world.db']='0'*64
        (dest/'manifest.json').write_text(json.dumps(manifest))
        with pytest.raises(InvalidTask):read_bundle(dest)
    finally:service.store.close()


def test_slam_origin_is_resampled_by_area_without_moving_obstacles():
    from home_robot_tasks.map_bundle import align_to_depth_lattice
    msg=grid();msg.info.origin.position.x=.025;msg.info.origin.position.y=.025
    msg.data[12]=100
    doc=convert(msg)
    aligned=align_to_depth_lattice(doc,resolution_m=.1,revision='aligned')
    assert aligned['origin']=={'x':0.,'y':0.}
    assert len(aligned['rows'])==6 and len(aligned['rows'][0])==6
    assert all(aligned['rows'][y][x]=='#' for y in (2,3) for x in (2,3))
    assert aligned['rows'][0]=='?'*6
    assert doc['origin']=={'x':.025,'y':.025}
