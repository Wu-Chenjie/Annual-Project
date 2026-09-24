import importlib.util
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
spec = importlib.util.spec_from_file_location('scene', PACKAGE/'scripts/scene.py')
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)
MAP = PACKAGE.parents[2]/'next_project/maps/sample_simple.json'


def test_geometry_matches_planning_map_and_fleet_interfaces(tmp_path):
    world_file, bridge_file = scene.generate(PACKAGE, MAP, tmp_path, [2,3,1.5])
    world = ET.parse(world_file).getroot().find('world')
    data = json.loads(MAP.read_text())
    for i, obstacle in enumerate(data['obstacles']):
        model = world.find(f"model[@name='obstacle_{i}']")
        geometry = model.find('link/collision/geometry')
        if obstacle['type']=='aabb':
            assert list(map(float, geometry.findtext('box/size').split())) == [b-a for a,b in zip(obstacle['min'],obstacle['max'])]
        else:
            assert float(geometry.findtext('cylinder/radius')) == obstacle['radius']
            assert float(geometry.findtext('cylinder/length')) == obstacle['z_range'][1]-obstacle['z_range'][0]
    bridges = yaml.safe_load(Path(bridge_file).read_text())
    assert len({b['ros_topic_name'] for b in bridges}) == 13
    assert sum(b['direction']=='ROS_TO_GZ' for b in bridges) == 3
    for i in range(3):
        drone=world.find(f"model[@name='drone_{i}']")
        assert float(drone.findtext('link/inertial/mass')) == 1.0
        assert drone.find('link/collision') is not None
        assert drone.find('plugin').attrib['name']=='annual::MotorSystem'
        assert drone.findtext("link/sensor[@name='contact']/topic")==f'/drone_{i}/contacts'


@pytest.mark.parametrize('obstacle', [
    {'type':'unsupported'},
    {'type':'aabb','min':[0,0,0],'max':[0,1,1]},
    {'type':'cylinder','center_xy':[1,1],'radius':-1,'z_range':[0,2]},
    {'type':'aabb','min':[0,0,0],'max':[float('nan'),1,1]},
])
def test_invalid_geometry_is_rejected(tmp_path,obstacle):
    source=tmp_path/'map.json'
    source.write_text(json.dumps({'bounds':[[0,0,0],[20,20,5]],'obstacles':[obstacle]}))
    with pytest.raises(ValueError): scene.generate(PACKAGE,source,tmp_path/'generated',[2,3,1.5])


def test_unordered_scientific_json_is_normalized_for_cpp_reader(tmp_path):
    source=tmp_path/'input.json'
    source.write_text('{"obstacles":[{"max":[1e1,2,3],"min":[1e-5,0,0],"type":"aabb"}],"bounds":[[0,0,0],[20,20,5]]}')
    scene.generate(PACKAGE,source,tmp_path/'generated',[2,3,1.5])
    text=(tmp_path/'generated/planner_map.json').read_text()
    result=json.loads(text)
    assert list(result['obstacles'][0]) == ['type','min','max']
    assert result['obstacles'][0]['min'][0] == 1e-5
    assert '0.00001' in text
