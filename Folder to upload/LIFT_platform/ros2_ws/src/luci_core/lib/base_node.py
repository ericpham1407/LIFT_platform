'''
core functionality for ROS2 python use
not provided by rclpy itself
'''
import array
from dataclasses import dataclass
from typing import Any, Dict, get_type_hints, Union, Tuple, Any
import sys
from enum import Enum

import rclpy
from rclpy.node import Node

import numpy as np

from onr_luci.pytypes import PythonMsg

@dataclass
class NodeParamTemplate:
    '''
    Base class for node parameter templates
    '''

    def spew_yaml(self):
        '''
        take a python parameter template
        and print out a text version
        that can be saved as a yaml
        '''
        def append_param(yaml_str, key, val, indent_depth) -> str:
            for _ in range(indent_depth):
                yaml_str += '  '

            yaml_str += key
            yaml_str += ': '
            if isinstance(val, str):
                yaml_str += "'" + val + "'"
            elif isinstance(val, np.ndarray):
                yaml_str += str(val.tolist())
            elif isinstance(val, (bool, int, float, str)):
                yaml_str += str(val)
            elif isinstance(val, (list, tuple, array.array)):
                yaml_str += str(val)
            elif val is None:
                yaml_str += 'null'
            elif val is PythonMsg:
                pass
            else:
                yaml_str += '0'

            yaml_str += '\n'
            return yaml_str


        def unpack_pythonmsg(yaml_str, msg, prefix, depth = 2):
            yaml_str = append_param(yaml_str, prefix, PythonMsg, depth)
            for key in vars(msg):
                val = getattr(msg, key)
                if isinstance(val, PythonMsg):
                    yaml_str = unpack_pythonmsg(yaml_str, val, key, depth + 1)
                else:
                    yaml_str = append_param(yaml_str, key, val, depth + 1)


            return yaml_str

        yaml_str = 'FILL THIS IN\n'
        yaml_str += '  ros__parameters:\n'

        for key in vars(self):
            val = getattr(self, key)
            if isinstance(val, PythonMsg):
                yaml_str = unpack_pythonmsg(yaml_str, val, key)
            else:
                yaml_str = append_param(yaml_str, key, val, 2)

        return yaml_str


def _is_valid_parameter_type(value: Any) -> bool:
    '''
    Method for checking if a variable "value" is a valid default value for a ROS2 parameter
    should not be used dynamically for transferring data to/fram ROS2 messages

    adapted from https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/parameter.py
    method "from_parameter_value"

    Will discard numpy types
    current workaround is "is_valid_numpy_parameter_type"
    Has to be handled separately so that the default value can be converted to a list.
    '''
    if value is None:
        return True
    elif isinstance(value, (bool, int, float, str)):
        return True
    elif isinstance(value, (list, tuple, array.array)):
        if all(isinstance(v,bytes) for v in value):
            return True
        elif all(isinstance(v,bool) for v in value):
            return True
        elif all(isinstance(v,int) for v in value):
            return True
        elif all(isinstance(v,float) for v in value):
            return True
        elif all(isinstance(v,str) for v in value):
            return True
    return False

def _is_valid_numpy_parameter_type(value: np.ndarray) -> bool:
    '''
    check if a numpy array is a valid default value for a ROS2 parameter

    THIS DOES NOT keep track of such parameters to convert autoloaded
    configurations to numpy - all array-type parameters are loaded as lists
    currently code to do that does not exist, but may exist at a later date.
    '''
    if isinstance(value, np.ndarray):
        if np.issubdtype(value.dtype, np.number):
            if value.ndim == 1:
                return True
    return False

class BaseNode(Node):
    '''
    base class for ROS2 nodes
    provides added features to
    automate delcaring parameters
    automate loading parameters
    package python messages in ROS2 messages
    unpack ROS2 messages into python messages
    '''
    _declared_optional_parameters: Dict[str, Tuple[callable]]
    '''
    optional parameters that should be nonetype if not provided
    they must still be declared with a type
    format is 
    (parameter label: type_converter)
    if the default ROS2 value is loaded is it replaced with None
    '''

    def __init__(self, label: str):
        self._declared_optional_parameters = {}
        super().__init__(label)

    def _close(self):
        ''' close the node '''
        sys.exit()

    def info(self, msg):
        ''' shorthand for ros2 info '''
        self.get_logger().info(str(msg))

    def get_ros_time(self):
        ''' helper function to get the current ROS clock'''
        time = self.get_clock().now()
        return time.nanoseconds/1000000000

    def autodeclare_parameters(self, template: PythonMsg, namespace: str, verbose = False):
        '''
        autodeclares parameters for the class
        template should be a class with fields corresponding to the parameters needed,
        e.g. template.dt, template.n_laps
        adds nested parameters for any instance of PythonMsg found in template
        '''

        def to_param(label: str, val: Any, type_hint: type):
            if val is None:
                # there should still be a type hint of typing.Union or typing.Optional
                assert hasattr(type_hint, '__origin__')
                assert type_hint.__origin__ is Union
                self._declared_optional_parameters[namespace + '.' + label] = type_hint.__args__[0]
                return (label, type_hint.__args__[0]())
            elif isinstance(val, Enum):
                return (label, val.value)
            elif _is_valid_parameter_type(val):
                return (label, val)
            elif _is_valid_numpy_parameter_type(val):
                return (label, val.tolist())
            return None

        def declare_pythonmsg(msg: PythonMsg, prefix):
            msg_parameters = []
            type_hints = get_type_hints(msg)
            for key in vars(msg):
                val = getattr(msg, key)
                label = prefix + '.' + key
                if isinstance(val, PythonMsg):
                    nested_parameters = declare_pythonmsg(val, label)
                    for param in nested_parameters:
                        msg_parameters.append(param)
                else:
                    param = to_param(label, val, type_hints[key])
                    if param is not None:
                        msg_parameters.append(param)
                    else:
                        self.get_logger().warn(f'Unable to Declare Parameter: {label}, {val} is an unsupported type')

            return msg_parameters

        parameters = []
        for key in vars(template):
            if verbose:
                self.get_logger().info(f'Checking parameter: {key}')
            val = getattr(template, key)
            if isinstance(val, PythonMsg):
                msg_parameters = declare_pythonmsg(val, key)
                for param in msg_parameters:
                    parameters.append(param)
                    if verbose:
                        self.get_logger().info(f'Declared parameter: {param[0]} with value {param[1]}')
            else:
                param = to_param(key, val, type(val))
                if param is not None:
                    parameters.append(param)
                    if verbose:
                        self.get_logger().info(f'Declared parameter: {key} with value {val}')
                else:
                    self.get_logger().warn(f'Unable to Declare Parameter: {key}, {val} is an unsupported type')


        self.declare_parameters(
            namespace=namespace,
            parameters=parameters
        )

    def autoload_parameters(self, template: PythonMsg, namespace: str, suppress_warnings = False, verbose = False):
        '''
        after parameters have been declared,
        this attempts to load them using the same template and namespace

        rather than modifying the template, loaded fields are added to self
        '''

        def load_parameter(target_msg: PythonMsg, key: str, param_key: str):
            try:
                loaded_val = self.get_parameter(param_key).value
            except rclpy.exceptions.ParameterNotDeclaredException:
                if not suppress_warnings:
                    self.get_logger().warn(f'Undeclared Node Param: {namespace + param_key}')
                loaded_val = None
            except rclpy.exceptions.ParameterUninitializedException:
                if param_key not in self._declared_optional_parameters:
                    self.get_logger().warn(f'Uninitialized Node Param:: {namespace + param_key}')
                loaded_val = None

            if loaded_val is None:
                setattr(target_msg, key, None)
                return

            target_type = type(getattr(target_msg, key))
            if target_type is type(None):
                assert param_key in self._declared_optional_parameters
                type_converter = self._declared_optional_parameters[param_key]
                if type_converter() == loaded_val:
                    set_val = None
                else:
                    set_val = type_converter(loaded_val)
            elif isinstance(target_type, np.ndarray):
                set_val = np.array(loaded_val)
            else:
                set_val = target_type(loaded_val)
            setattr(target_msg, key, set_val)
            if verbose:
                self.get_logger().info(f'Loaded parameter {param_key} with value {set_val}')

        def load_pythonmsg(namespace: str, target_python_msg: PythonMsg):
            if not namespace == '' and not namespace[-1] == '.':
                namespace += '.'

            for key in vars(target_python_msg):
                target_data = getattr(target_python_msg, key)
                if isinstance(target_data, PythonMsg):
                    load_pythonmsg(namespace + key, target_data)
                else:
                    load_parameter(target_python_msg, key, namespace + key)

        for key in vars(template):
            if isinstance(getattr(template, key), PythonMsg):
                msg = getattr(template, key)
                load_pythonmsg(namespace + '.' + key, msg)
            else:
                param = '.'.join((namespace,key))
                load_parameter(template, key, param)
            setattr(self, key, getattr(template, key))

    def populate_msg(self, msg, data: PythonMsg, suppress_warnings = False):
        '''
        Takes a python message 'data' and tries to load all of its keys data into
        the ROS2 message 'msg'

        Prints a warning if there is no destination for the key or in the event of type mismatch.
        '''
        for key in vars(data):
            if hasattr(msg, key):
                if isinstance(getattr(data, key), PythonMsg):
                    BaseNode.populate_msg(self, getattr(msg, key),
                                      getattr(data, key))
                    continue
                try:
                    new_data = getattr(data, key)
                    if new_data is None:
                        continue

                    if isinstance(new_data, Enum):
                        new_data = new_data.value

                    target_type = type(getattr(msg, key))
                    if isinstance(new_data, target_type):
                        setattr(msg, key, new_data)
                    else:
                        setattr(msg, key, target_type(new_data))

                except AssertionError:
                    err = f'Type error for key {key}, cannot write type' + \
                          f' {type(getattr(data, key))} to'      + \
                          f' {type(getattr(msg, key))}'
                    if self:
                        self.get_logger().warn(err)
                    else:
                        print(err)
            elif not suppress_warnings:
                err = f'No destination for key {key} in msg {type(msg)}'
                if self:
                    self.get_logger().warn(err)
                else:
                    print(err)

        return msg

    def unpack_msg(self, msg, data: PythonMsg, suppress_warnings = False):
        '''
        Takes a ROS2 message 'msg' and tries to load all of its data into
        the python message 'data'

        Prints a warning if there is no destination for the key
        '''
        target_types = get_type_hints(data)
        for key in msg.get_fields_and_field_types().keys():
            if key == 'header':
                continue
            if key not in data.__dict__:
                if not suppress_warnings:
                    err = f'No destination for key {key} in data {type(data)}'
                    if self:
                        self.get_logger().warn(err)
                    else:
                        print(err)
            elif isinstance(target_types[key], PythonMsg):
                BaseNode.unpack_msg(self, getattr(msg, key), getattr(data, key))
                continue
            else:
                try:
                    setattr(data, key, target_types[key](getattr(msg, key)))
                except TypeError as e:
                    if target_types[key].__origin__ is Union:
                        try:
                            setattr(data, key, target_types[key].__args__[0](getattr(msg, key)))
                        except TypeError as f:
                            raise f from e
                    else:
                        raise e
