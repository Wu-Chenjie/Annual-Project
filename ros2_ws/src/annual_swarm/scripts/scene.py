"""Generate Gazebo geometry and bridges from the same JSON used by the ROS planning algorithms."""
import copy
from decimal import Decimal
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from exploration_palette import UAV_COLORS

OFFSETS = [(0.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 1.0, 0.0)]

def vector(values):
    values = list(values)
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Map contains non-finite coordinates')
    return ' '.join(str(float(v)) for v in values)

def coordinates(value, count):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f'Expected {count} coordinates')
    vector(value)
    return [float(v) for v in value]

def canonical_json(value):
    # The legacy C++ map reader expects ordered shape fields and decimal numbers.
    # Normalize once at launch so planning sees exactly the geometry Gazebo sees.
    if isinstance(value, dict):
        return '{' + ','.join(json.dumps(k)+':'+canonical_json(v) for k,v in value.items()) + '}'
    if isinstance(value, list): return '[' + ','.join(canonical_json(v) for v in value) + ']'
    if isinstance(value, (int,float)): return format(Decimal(str(value)), 'f')
    return json.dumps(value)

def generate(share, map_file, output, start, dynamic=False, starts=None, lidar=False):
    share, output = Path(share), Path(output)
    data = json.loads(Path(map_file).read_text())
    if len(start) != 3: raise ValueError('start requires three coordinates')
    vector(start)
    bounds = data['bounds']
    if len(bounds) != 2 or any(len(v) != 3 for v in bounds):
        raise ValueError('bounds must contain two 3D corners')
    bounds = [coordinates(v,3) for v in bounds]
    canonical = {'bounds':bounds, 'obstacles':[]}
    if any(bounds[1][i] <= bounds[0][i] for i in range(3)):
        raise ValueError('bounds must be ordered')
    tree = ET.parse(share / 'worlds/indoor.sdf')
    world = tree.getroot().find('world')
    if lidar:
        plugin = ET.SubElement(world, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
        ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    for i, obs in enumerate(data['obstacles']):
        model = ET.SubElement(world, 'model', name=f'obstacle_{i}')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='link')
        geom = ET.Element('geometry')
        if obs['type'] == 'aabb':
            low, high = coordinates(obs['min'],3), coordinates(obs['max'],3)
            canonical['obstacles'].append({'type':'aabb','min':low,'max':high})
            size = [high[k] - low[k] for k in range(3)]
            if min(size) <= 0: raise ValueError('AABB dimensions must be positive')
            center = [(high[k]+low[k])/2 for k in range(3)]
            ET.SubElement(ET.SubElement(geom,'box'),'size').text = vector(size)
        elif obs['type'] == 'cylinder':
            z0,z1 = coordinates(obs['z_range'],2); radius = float(obs['radius'])
            xy = coordinates(obs['center_xy'],2)
            vector([radius])
            canonical['obstacles'].append({'type':'cylinder','center_xy':xy,'radius':radius,'z_range':[z0,z1]})
            if radius <= 0 or z1 <= z0: raise ValueError('Cylinder dimensions must be positive')
            center = [*xy, (z0+z1)/2]
            cylinder = ET.SubElement(geom, 'cylinder')
            ET.SubElement(cylinder, 'radius').text = vector([radius])
            ET.SubElement(cylinder, 'length').text = vector([z1-z0])
        else:
            raise ValueError(f"Unsupported Gazebo map obstacle: {obs['type']}")
        ET.SubElement(model,'pose').text = vector([*center,0,0,0])
        ET.SubElement(link,'collision',name='collision').append(copy.deepcopy(geom))
        visual = ET.SubElement(link,'visual',name='visual'); visual.append(geom)
        ET.SubElement(ET.SubElement(visual,'material'),'diffuse').text = '0.4 0.5 0.6 1'
    if dynamic:
        model=ET.SubElement(world,'model',name='dynamic_obstacle')
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text='-20 -20 1.5 0 0 0'
        link=ET.SubElement(model,'link',name='link')
        for tag in ('collision','visual'):
            element=ET.SubElement(link,tag,name=tag)
            cylinder=ET.SubElement(ET.SubElement(element,'geometry'),'cylinder')
            ET.SubElement(cylinder,'radius').text='0.55';ET.SubElement(cylinder,'length').text='3.0'
            if tag=='visual':ET.SubElement(ET.SubElement(element,'material'),'diffuse').text='0.9 0.15 0.1 1'
    bridges = [dict(ros_topic_name='/clock',gz_topic_name='/clock',ros_type_name='rosgraph_msgs/msg/Clock',gz_type_name='gz.msgs.Clock',direction='GZ_TO_ROS',qos_profile='CLOCK')]
    template = ET.parse(share/'models/quadrotor.sdf').getroot().find('model')
    spawns=starts if starts is not None else [[start[k]+offset[k] for k in range(3)] for offset in OFFSETS]
    for i,spawn in enumerate(spawns):
        coordinates(spawn,3)
        name = f'drone_{i}'
        model = copy.deepcopy(template); model.set('name',name)
        ET.SubElement(model,'pose').text = vector([spawn[0],spawn[1],0.10,0,0,0])
        if starts is not None:
            color = vector([*UAV_COLORS[i % len(UAV_COLORS)], 1.])
            for visual in model.findall('link/visual'):
                if visual.get('name') == 'body' or visual.get('name', '').startswith('rotor'):
                    visual.find('material/diffuse').text = color
        model.find("link/sensor[@name='contact']/topic").text = f'/{name}/contacts'
        model.find("link/sensor[@name='imu']/topic").text = f'/{name}/imu'
        if lidar:
            sensor = ET.SubElement(model.find('link'), 'sensor', name='exploration_lidar', type='gpu_lidar')
            ET.SubElement(sensor, 'pose').text = '0 0 0.4 0 0 0'
            ET.SubElement(sensor, 'topic').text = f'/{name}/lidar'
            ET.SubElement(sensor, 'update_rate').text = '5'
            ET.SubElement(sensor, 'always_on').text = 'true'
            scan = ET.SubElement(ET.SubElement(sensor, 'lidar'), 'scan')
            for kind, count in [('horizontal', 181), ('vertical', 31)]:
                axis = ET.SubElement(scan, kind)
                for tag, value in [('samples', count), ('resolution', 1), ('min_angle', -math.pi/3), ('max_angle', math.pi/3)]:
                    ET.SubElement(axis, tag).text = str(value)
            ranges = ET.SubElement(sensor.find('lidar'), 'range')
            for tag, value in [('min', .1), ('max', 4.5), ('resolution', .01)]:
                ET.SubElement(ranges, tag).text = str(value)
            noise = ET.SubElement(sensor.find('lidar'), 'noise')
            ET.SubElement(noise, 'type').text = 'gaussian'
            ET.SubElement(noise, 'mean').text = '0'; ET.SubElement(noise, 'stddev').text = '0.005'
            bridges.append(dict(ros_topic_name=f'/{name}/lidar/points', gz_topic_name=f'/{name}/lidar/points',
                ros_type_name='sensor_msgs/msg/PointCloud2', gz_type_name='gz.msgs.PointCloudPacked', direction='GZ_TO_ROS'))
        world.append(model)
        for topic, ros_type, gz_type, direction in [
            ('odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),
            ('imu','sensor_msgs/msg/Imu','gz.msgs.IMU','GZ_TO_ROS'),
            ('contacts','ros_gz_interfaces/msg/Contacts','gz.msgs.Contacts','GZ_TO_ROS'),
            ('command/motor_speed','actuator_msgs/msg/Actuators','gz.msgs.Actuators','ROS_TO_GZ')]:
            bridges.append(dict(ros_topic_name=f'/{name}/{topic}',gz_topic_name=f'/{name}/{topic}',ros_type_name=ros_type,gz_type_name=gz_type,direction=direction))
    output.mkdir(parents=True,exist_ok=True)
    world_file, bridge_file = output/'scene.sdf',output/'bridge.yaml'
    (output/'planner_map.json').write_text(canonical_json(canonical))
    tree.write(world_file,encoding='unicode',xml_declaration=True)
    bridge_file.write_text(yaml.safe_dump(bridges))
    return str(world_file), str(bridge_file)
