#!/usr/bin/env python3
''' test node '''
import sys
import traceback

from launch import LaunchDescription

from luci_core.launch_utils import get_node, get_config_directory

def generate_launch_description():
    ''' the launch description '''
    try:
        parameters = [get_config_directory('test_config.yaml', 'luci_core')]
        namespace  = ''

        gps_node = get_node('luci_core', 'test_node.py', parameters, namespace)

        return LaunchDescription([gps_node])

    except Exception:
        print('There was an error inside your launch code!!!! 🙄️')
        print('ROS 2 won\'t handle it nicely, so here is the real error:')
        print('')
        traceback.print_exc()

        sys.exit(0)
