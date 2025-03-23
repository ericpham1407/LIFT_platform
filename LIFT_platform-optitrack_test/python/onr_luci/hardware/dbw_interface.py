'''
'''

import time
from typing import List, Callable, Optional
from dataclasses import dataclass, field
import warnings
import math
from enum import Enum
from xbox360controller import Xbox360Controller

import numpy as np

from onr_luci.hardware.motor_interface import MotorControlMode, MotorReadout, MotorCommand, MotorInterface, MotorLocation
from onr_luci.pytypes import PythonMsg

# TODO: maybe make a vehicle configuration file to select from mulitple tire options in the future or eventually this is determined by the controller
WHEEL_BASE_W = 0.304 # wheel base width in meters (center of tire to center of tire)
TIRE_RADIUS = 0.125/2 # tire radius in meters 

TORQUE_LIMIT = 5  # 1
ACCEL_LIMIT = 10

class VehicleControlMode(Enum):
    POSITION = 0
    VELOCITY = 1
    ROTATE = 2             # rotate in place
    STOPPED = 3

def _vehicle_mode_postion_handler(velocity: float, omega: float, warning_handler):
    warning_handler('Position vehicle control mode not implemented')
    # TODO
    cmds = []
    return cmds

def _vehicle_mode_velocity_handler(velocity: float, omega: float, warning_handler):
    motor_mode = MotorControlMode.VELOCITY

    # assume this is both linear and angular veocity of the vehicle
    # NOTE - velocity for the motor commands is in Hz / revolutions per second
    # linear velocity component
    vel_cmd_lin = velocity / (2*TIRE_RADIUS*math.pi)
    
    # angular velocity component
    vel_cmd_omega = omega * (WHEEL_BASE_W/2) / (2*TIRE_RADIUS*math.pi)

    # velocities of the motors
    vel_right = vel_cmd_lin + vel_cmd_omega
    vel_left = vel_cmd_lin - vel_cmd_omega

    # commands
    cmd_fl = MotorCommand(
            motor = MotorLocation.FL,
            mode = motor_mode,
            velocity = vel_left,
            torque = TORQUE_LIMIT,
            accel_limit = ACCEL_LIMIT
        )
    cmd_rl = MotorCommand(
        motor = MotorLocation.RL,
        mode = motor_mode,
        velocity = vel_left,
        torque = TORQUE_LIMIT,
        accel_limit = ACCEL_LIMIT
    )
    cmd_fr = MotorCommand(
        motor = MotorLocation.FR,
        mode = motor_mode,
        velocity = vel_right,
        torque = TORQUE_LIMIT,
        accel_limit = ACCEL_LIMIT
    )
    cmd_rr = MotorCommand(
        motor = MotorLocation.RR,
        mode = motor_mode,
        velocity = vel_right,
        torque = TORQUE_LIMIT,
        accel_limit = ACCEL_LIMIT
    )

    cmds = [cmd_fl, cmd_rl, cmd_fr, cmd_rr]
    return cmds

def _vehicle_mode_rotate_handler(velocity: float, omega: float, warning_handler):
    motor_mode = MotorControlMode.VELOCITY
    if velocity != 0:
        warning_handler('Vehicle control mode cannot be pure rotation with nonzero vehicle linear velocity!')
    else:
        # angular velocity component to motor velocity commands
        vel_cmd_omega = omega * (WHEEL_BASE_W/2) / (2*TIRE_RADIUS*math.pi)

        # velocities of motors
        vel_right = vel_cmd_omega
        vel_left = - vel_cmd_omega

        # commands
        cmd_fl = MotorCommand(
            motor = MotorLocation.FL,
            mode = motor_mode,
            velocity = vel_left,
            torque = TORQUE_LIMIT,
            accel_limit = ACCEL_LIMIT
        )
        cmd_rl = MotorCommand(
            motor = MotorLocation.RL,
            mode = motor_mode,
            velocity = vel_left,
            torque = TORQUE_LIMIT,
            accel_limit = ACCEL_LIMIT
        )
        cmd_fr = MotorCommand(
            motor = MotorLocation.FR,
            mode = motor_mode,
            velocity = vel_right,
            torque = TORQUE_LIMIT,
            accel_limit = ACCEL_LIMIT
        )
        cmd_rr = MotorCommand(
            motor = MotorLocation.RR,
            mode = motor_mode,
            velocity = vel_right,
            torque = TORQUE_LIMIT,
            accel_limit = ACCEL_LIMIT
        )

        cmds = [cmd_fl, cmd_rl, cmd_fr, cmd_rr]
        return cmds

def _vehicle_mode_stop_handler(velocity: float, omega: float, warning_handler):
    motor_mode = MotorControlMode.IDLE
    warning_handler('Stopped vehicle control mode not implemented')
    # TODO
    cmds = []
    return cmds

VEHICLE_CONTROL_MODE_HANDLER = {
    VehicleControlMode.POSITION: _vehicle_mode_postion_handler,
    VehicleControlMode.VELOCITY: _vehicle_mode_velocity_handler,
    VehicleControlMode.ROTATE: _vehicle_mode_rotate_handler,
    VehicleControlMode.STOPPED:_vehicle_mode_stop_handler,
}

@dataclass
class VehicleActuation(PythonMsg):
    mode: VehicleControlMode = field(default = VehicleControlMode.VELOCITY)
    velocity: float = field(default = 0.)   # linear velocity of the vehicle [m/s]
    omega: float = field(default = 0.)      # angular velocity of the vehicle around the center of the vehicle, positive should be CCW [rad/s]
    # kappa: Optional[float] = field(default = 0.)    # curvature of path
    warning_handler: Callable[[str], None] = field(default = warnings.warn)

    # def _kappa_setter(self):

    # @property # (looks like a attribute but can't set)
    # def L(self) -> float:
    #     ''' wheelbase '''
    #     return self.lr + self.lf
    
    # @L.setter
    # def _L_setter(self, val):
    #     self.lr = self.lf - val

    def to_cmds(self) -> List[MotorCommand]:
        # match to map from VehicleControlMode to MotorControlMode
        cmds = VEHICLE_CONTROL_MODE_HANDLER.get(self.mode)(self.velocity, self.omega, self.warning_handler)       
        return cmds
    
class TelemetryFailures(Enum):
    '''
    enum class for types of fails we check for by checking telemetry from all motors
    '''
    SAFE = 0
    VOLTAGE = 1
    TEMPERATURE = 2

@dataclass
class MotorStatus(PythonMsg):
    motor: MotorLocation = field(default = None)
    readout: MotorReadout = field(default = None)
    status: TelemetryFailures = field(default = None)

@dataclass
class VehicleState(PythonMsg):
    u_dbw: VehicleActuation = field(default = None)
    fl: MotorStatus = field(default = None)
    rl: MotorStatus = field(default = None)
    fr: MotorStatus = field(default = None)
    rr: MotorStatus = field(default = None)

    def __post_init__(self):
        if self.u_dbw is None:
            self.u_dbw = VehicleActuation()
        if self.fl is None:
            # self.fl = MotorReadout(motor=MotorLocation.FL)
            self.fl = MotorStatus(motor=MotorLocation.FL, readout=MotorReadout(motor=MotorLocation.FL), status=TelemetryFailures.SAFE)
        if self.fr is None:
            self.fr = MotorStatus(motor=MotorLocation.FR, readout=MotorReadout(motor=MotorLocation.FR), status=TelemetryFailures.SAFE)
        if self.rl is None:
            self.rl = MotorStatus(motor=MotorLocation.RL, readout=MotorReadout(motor=MotorLocation.RL), status=TelemetryFailures.SAFE)
        if self.rr is None:
            self.rr = MotorStatus(motor=MotorLocation.RR, readout=MotorReadout(motor=MotorLocation.RR), status=TelemetryFailures.SAFE)

class MotorAdaptiveController():
    # NOTE - in the future when this is more of a controller potentially have inherit from AbstractController class
    _warning_handler: Callable[[str], None] = warnings.warn

    # TODO - potentially set up a config (not sure if it should be the controller config though) with params for the telemetry failure thresholds

    def __init__(self, iface: MotorInterface, state: VehicleState):
        self.iface = iface

    def _check_telemetry(self, motor_readout: MotorReadout):
        '''
        checks the motor readout for safe operating conditions (temperature, voltage, etc.)
        '''
        failure = False
        fail_type = TelemetryFailures.SAFE

        # voltage failure
        if motor_readout.voltage <= 3.7*6:
            failure = True
            fail_type = TelemetryFailures.VOLTAGE

        # temperature failure
        if motor_readout.temperature >= 50: # in C
            failure = True
            fail_type = TelemetryFailures.TEMPERATURE

        # add other failure modes if desired 

        return failure, fail_type

    # def set_velocity_target(self, lin_vel: float, ang_vel: float):
    #     '''
    # TODO write docstring
    #     sets the platform to velocity mode with given targets
    #     '''
    #     #TODO - or have all inputs in step function and modify by reference and stack in list outside

    def step(self, state: VehicleState):
        # NOTE - in the future might want to include an additional optimal input that parameterizes / characterizes (ex: env_state in barc used for tracking or collision)
       
        print("[MotorAdaptiveController] step() was called!")  # Debug print

        try:
            # write commands
            cmds = state.u_dbw.to_cmds()
            print(f"[MotorAdaptiveController] Sending commands: {cmds}")  # Debug print

            self.iface.write_commands(cmds)
            print("[MotorAdaptiveController] Sent commands to MotorInterface!")

        except Exception as e:
            print(f"[MotorAdaptiveController] ERROR: Exception occurred - {e}")
        time.sleep(0.5)

        # get readouts and assign to vehicle state
        readouts = self.iface.read()
        for readout in readouts[-4:]:
            failure, fail_type = self._check_telemetry(readout)
            # TODO - confirm/check that indexing the last 4 does indeed get one readout for each motor (no double reads and no case of where we don't receive a reading from a motor for any reason)
            # TODO - write the following in a better way
            if readout.motor == MotorLocation.FL:
                state.fl.readout = readout
                state.fl.status = fail_type
            elif readout.motor == MotorLocation.RL:
                state.rl.readout = readout
                state.rl.status = fail_type
            elif readout.motor == MotorLocation.FR:
                state.fr.readout = readout
                state.fr.status = fail_type
            elif readout.motor == MotorLocation.RR:
                state.rr.readout = readout
                state.rr.status = fail_type

            # print(readout.motor, 'vel:', readout.velocity)
            # print(readout.motor, 'voltage:', readout.voltage)

            if failure:
                self._warning_handler('Failed telemetry checks')
                print('Failure type:', fail_type)
                print('Failed motor location:', readout.motor)

        # eventually get new state / state update
        # for now just doing constant state (vehicle action / inputs always the same)
        # self.state = ....
        
if __name__ == '__main__':
    test_state = VehicleState()

    # FOR testing with joystick
    joystick = Xbox360Controller(0, axis_threshold=0.0)
    velocity_max = 0.4
    omega_max = 0.8

    # section for testing on mac (without jotstick)
    '''
    test_type = 'stopped'
    match test_type:
        case 'straight':
            test_state.u_dbw.mode = VehicleControlMode.VELOCITY
            test_state.u_dbw.velocity = 0.3
            test_state.u_dbw.omega = 0.
        case 'rotate':
            test_state.u_dbw.mode = VehicleControlMode.ROTATE
            test_state.u_dbw.velocity = 0.
            test_state.u_dbw.omega = 0.8
        case 'turning':
            test_state.u_dbw.mode = VehicleControlMode.VELOCITY
            test_state.u_dbw.velocity = 0.2
            test_state.u_dbw.omega = 0.4
        case 'position':
            test_state.u_dbw.mode = VehicleControlMode.POSITION
            test_state.u_dbw.velocity = 0.2
            test_state.u_dbw.omega = 0.4
        case 'stopped':
            test_state.u_dbw.mode = VehicleControlMode.STOPPED
'''
    iface = MotorInterface()
    controller = MotorAdaptiveController(iface, test_state)
    controller.iface.reset_motors()

    try:
        while True:
            # for testing with joystick
            velocity_input, omega_input = (-joystick.axis_l.y, -joystick.axis_r.x)

            velocity_input = np.clip(velocity_input, -velocity_max, velocity_max)
            omega_input = np.clip(omega_input, -omega_max, omega_max)

            controller.state.u_dbw.velocity = velocity_input
            controller.state.u_dbw.omega = omega_input
            controller.state.u_dbw.mode = VehicleControlMode.VELOCITY

            controller.step(test_state)
    
    except KeyboardInterrupt:
        controller.iface.reset_motors()
        time.sleep(1)
        controller.iface.shutdown()
