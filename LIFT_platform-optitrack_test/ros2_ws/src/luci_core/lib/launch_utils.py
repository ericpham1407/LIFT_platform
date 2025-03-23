''' launch utils '''
import os
import yaml
from launch_ros.actions import Node as LaunchNode
from ament_index_python.packages import get_package_share_directory

def read_yaml_file(filename):
    ''' reads a yaml file in a format suitable for passing to
    a ROS2 node in a launch file'''
    params = []
    with open(filename, 'r', encoding = 'utf-8') as yaml_file:
        yaml_data = yaml.load(yaml_file, Loader=yaml.FullLoader)
        # Convert to list of dicts
        for (key, val) in yaml_data.items():
            params.append({key : val})
    return params

def get_node(package, node_name, parameters, namespace):
    ''' handle node creation in launch scripts for multiple ROS2 versions'''
    # pylint: disable=missing-kwoa
    if os.path.exists('/opt/ros/humble'):
        node = LaunchNode(package = package,
                          executable = node_name,
                          parameters = parameters,
                          namespace = namespace)
    elif os.path.exists('/opt/ros/foxy'):
        node = LaunchNode(package = package,
                          executable = node_name,
                          parameters = parameters,
                          namespace = namespace)
    elif os.path.exists('/opt/ros/eloquent'):
        node = LaunchNode(package = package,
                          output = 'screen',
                          emulate_tty = True,
                          node_executable = node_name,
                          parameters = parameters,
                          node_namespace = namespace)
    else:
        node = None
        raise NotImplementedError('Unhandled ROS distro')
    return node

def get_config_directory(filename, package = 'luci_core'):
    ''' returns a filepath to a file from the config directory for this package '''
    return os.path.join(get_package_share_directory(package), 'config', filename)
