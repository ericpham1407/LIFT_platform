'''
interface to four motor setup on the car
'''

from enum import Enum
from typing import Optional, Union, List, Generator, Callable
from dataclasses import dataclass, field
import asyncio
from multiprocessing import Process, Queue, Value
from multiprocessing.sharedctypes import Synchronized
import time
import math
import warnings
import pdb

import moteus
import moteus.moteus        #(TF) - this seems to help pylance but doesn't impact runtime
import moteus.multiplex     #(TF) - this seems to help pylance but doesn't impact runtime

from onr_luci.pytypes import PythonMsg

MIN_WATCHDOG_TIMEOUT = 10.
MOTEUS_QUERY_RESOLUTION = moteus.moteus.QueryResolution()
MOTEUS_QUERY_RESOLUTION.q_current = moteus.multiplex.F32
MOTEUS_QUERY_RESOLUTION.d_current = moteus.multiplex.F32
MOTEUS_QUERY_RESOLUTION.power = moteus.multiplex.F32

class MotorControlMode(Enum):
    ''' a mode for controlling a motor '''
    IDLE = -1
    ''' stops the motor, also clears faults '''
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2
    RESET = 3
    ''' stops the motor and zeros the encoder, also clears faults '''


class MotorLocation(Enum):
    '''access to motor locations by location instead of CAN bus ID '''
    FR = 3 # need positive speed
    FL = 1 # need negative speed
    RR = 4 # need positive speed
    RL = 2 # need negative speed

@dataclass
class MotorCommand(PythonMsg):
    '''
    motor command structure

    motor is chosen by corner or directly by CAN bus address (int)
    control mode alters which fields are target:
    IDLE: no target, motor idle
    POSITION: motor holds current or input position
        velocity and torque fields become limits
    VELOCITY: motor maintains target velocity (pos or neg)
        torque parameter becomes a limit, position is discarded
    TORQUE: motor maintains target torque
        velocity parameter becomes a limit, position is discarded
    '''
    motor: Union[MotorLocation, int] = field(default = 1)
    mode: MotorControlMode = field(default = MotorControlMode.VELOCITY)
    position: Optional[float] = field(default = None)
    velocity: float = field(default = 0.)
    torque: float = field(default = 0.)
    accel_limit: float = field(default = None)


def _idle_cmd_parser(cmd: MotorCommand, controller: moteus.Controller, timeout: float):
    # pylint: disable=unused-argument
    # disabled since this command has no timeout but function args must match other parsers
    assert cmd.mode == MotorControlMode.IDLE
    return [controller.make_stop(query = True)]

def _pos_cmd_parser(cmd: MotorCommand, controller: moteus.Controller, timeout: float):
    assert cmd.mode == MotorControlMode.POSITION
    timeout = min(timeout, MIN_WATCHDOG_TIMEOUT)
    return [controller.make_position(
        position = cmd.position,
        velocity_limit = cmd.velocity,
        maximum_torque = cmd.torque,
        accel_limit = cmd.accel_limit,
        watchdog_timeout = timeout,
        query = True,
    )]

def _vel_cmd_parser(cmd: MotorCommand, controller: moteus.Controller, timeout: float):
    assert cmd.mode == MotorControlMode.VELOCITY
    timeout = min(timeout, MIN_WATCHDOG_TIMEOUT)
    return [controller.make_position(
        position = math.nan,
        velocity = cmd.velocity,
        maximum_torque = cmd.torque,
        accel_limit = cmd.accel_limit,
        watchdog_timeout = timeout,
        query = True,
    )]

def _torque_cmd_parser(cmd: MotorCommand, controller: moteus.Controller, timeout: float):
    assert cmd.mode == MotorControlMode.TORQUE
    timeout = min(timeout, MIN_WATCHDOG_TIMEOUT)
    return [controller.make_position(
        position = math.nan,
        velocity_limit = cmd.velocity,
        feedforward_torque = cmd.torque,
        accel_limit = cmd.accel_limit,
        watchdog_timeout = timeout,
        query = True,
    )]

def _reset_cmd_parser(cmd: MotorCommand, controller: moteus.Controller, timeout: float):
    # pylint: disable=unused-argument
    # disabled since this command has no timeout but function args must match other parsers
    assert cmd.mode == MotorControlMode.RESET
    return [
        controller.make_stop(),
        controller.make_rezero(query = True),
    ]

MOTOR_CMD_PARSERS = {
    MotorControlMode.IDLE: _idle_cmd_parser,
    MotorControlMode.POSITION: _pos_cmd_parser,
    MotorControlMode.VELOCITY: _vel_cmd_parser,
    MotorControlMode.TORQUE: _torque_cmd_parser,
    MotorControlMode.RESET: _reset_cmd_parser,
}

class MotorOperationMode(Enum):
    '''
    modes of operation of the moteus controller
    source: https://github.com/mjbots/moteus/blob/main/docs/reference.md#0x000---mode
    '''
    STOPPED = 0
    FAULT = 1
    STARTUP_0 = 2
    STARTUP_1 = 3
    STARTUP_2 = 4
    PWM_CONTROL = 5
    VOLTAGE_CONTROL = 6
    VOLTAGE_FOC_CONTROL = 7
    VOLTAGE_DQ_CONTROL = 8
    CURRENT_CONTROL = 9
    POSITION_CONTROL = 10
    TIMEOUT = 11
    ZERO_VEL_CONTROL = 12
    STAY_WITHIN_CONTROL = 13
    INDUCTANCE_MEASUREMENT = 14
    BRAKING = 15


class MotorFaultStatus(Enum):
    '''
    fault codes from moteus controller
    soure: https://github.com/mjbots/moteus/blob/main/docs/reference.md#0x00f---fault-code
    '''
    HEALTHY = 0
    DMA_STREAM_TRANSFER_ERROR = 1
    DMA_STREAM_FIFO_ERROR = 2
    UART_OVERRUN_ERROR = 3
    UART_FRAMING_ERROR = 4
    UART_NOISE_ERROR = 5
    UART_BUFFER_OVERRUN_ERROR = 6
    UART_PARITY_OVERRUN_ERROR = 7
    CALIBRATION_FAULT = 32
    MOTOR_DRIVER_FAULT = 33
    OVER_VOLTAGE = 34
    ENCODER_FAULT = 35
    MOTOR_NOT_CONFIGURED = 36
    PWM_CYCLE_OVERRUN = 37
    OVER_TEMPERATURE = 38
    START_OUTSIDE_LIMIT = 39
    UNDER_VOLTAGE = 40
    CONFIG_CHANGED = 41
    THETA_INVALID = 42
    POSITION_INVALID = 43
    DRIVER_ENABLE_FAULT = 44
    STOP_POSITION_DEPRECATED = 45
    TIMING_VIOLATION = 46

@dataclass
class MotorReadout(PythonMsg):
    '''
    readout of motor controller information
    '''
    motor: Union[MotorLocation, int] = field(default = 1)
    mode: MotorOperationMode = field(default = None)
    fault: MotorFaultStatus = field(default = None)
    position: float = field(default = None)
    velocity: float = field(default = None)
    torque: float = field(default = None)
    q_current: float = field(default = None)
    d_current: float = field(default = None)
    voltage: float = field(default = None)
    temperature: float = field(default = None)
    power: float = field(default = None)

@dataclass
class MotorInterfaceConfig(PythonMsg):
    '''
    configuation of the motor interface
    a watchdog timeout must be set, and may not be greater than MIN_WATCHDOG_TIMEOUT seconds.
    CAN id's for the motor corners are used to identify them
    if no CAN id's are provided it is assumed fl,rl, fr,rr = 1,2,3,4
    '''
    command_timeout: float = field(default = 1)
    can_bus_timeout: float = field(default = 0.1)
    blacklist_absent_addresses: bool = field(default = True)
    ''' whenever a can bus address fails to respond, silently ignore future commands to it '''
    warning_handler: Callable[[str], None] = field(default = warnings.warn)
    id_fr: int = field(default = MotorLocation.FR.value)
    id_fl: int = field(default = MotorLocation.FL.value)
    id_rr: int = field(default = MotorLocation.RR.value)
    id_rl: int = field(default = MotorLocation.RL.value)


class MotorInterface():
    '''
    interface to the moteus motor controllers
    exposes an API to parse MotorCommand structures and control the motors
    also provides MotorReadout structures in return

    it is recommended to call MotorInterface.reset_motors after initialization
    to clear any faults and rezero the motor encoders, as this means the motors
    will be armed and ready for commands.
    '''

    _motor_proc: Process = None

    def __init__(self, config: MotorInterfaceConfig = None):
        if config is None:
            config = MotorInterfaceConfig()
        self._config = config

        self._setup_child_proc()

    def _setup_child_proc(self):
        '''
        sets up child subprocess which controls motor asynchronously
        and parses control commands
        '''
        self._motor_cmd_queue = Queue()
        self._motor_telem_queue = Queue()
        self._can_connected = Value('d', 0)
        self._should_shutdown = Value('d', 0)

        self._motor_proc = Process(
            target = _MotorInterface,
            args = (self._config,
                    self._motor_cmd_queue,
                    self._motor_telem_queue,
                    self._can_connected,
                    self._should_shutdown)
        )
        self._motor_proc.start()

    def write_command(self, cmd: MotorCommand):
        ''' write a motor command, which will be applied asap '''
        if not self._alive():
            self._config.warning_handler('Motor Commands Written After Shutdown')
            return

        #self._motor_cmd_queue.put(cmd)
        for cmd in cmds:
            print(f"[MotorInterface] Writing Command: {cmd}")  # Debug print
            self._motor_cmd_queue.put(cmd)

    def write_command_to_all(self, cmd: MotorCommand):
        '''
        write a command to all motors
        '''
        commands: List[MotorCommand] = []
        for motor in MotorLocation:
            commands.append(cmd.copy())
            commands[-1].motor = motor
        self.write_commands(commands)

    def write_commands(self, cmds: List[MotorCommand]):
        ''' write several motor commands '''
        if not self._alive():
            self._config.warning_handler('Motor Commands Written After Shutdown')
            return

        for cmd in cmds:
            print(f"[MotorInterface] Writing Command: {cmd}")  # Debug 
            self._motor_cmd_queue.put(cmd)

    def read(self) -> List[MotorReadout]:
        ''' read any returned motor statuses '''
        if not self._alive():
            self._config.warning_handler('Attempt to Read from Closed Motor Interface')
            return []

        return list(self._read())

    def _read(self) -> Generator[MotorReadout, None, None]:
        while not self._motor_telem_queue.empty():
            yield self._motor_telem_queue.get()

    def stop_motors(self):
        '''
        helper function to stop all motors
        also clears any faults, meaing the motors are "armed" if any motion commands arrive
        '''
        commands = []
        for motor in MotorLocation:
            commands.append(
                MotorCommand(motor = motor, mode = MotorControlMode.IDLE)
            )
        self.write_commands(commands)

    def reset_motors(self):
        '''
        helper function to stop and rezero all motors
        also clears any faults, meaing the motors are "armed" if any motion commands arrive
        '''
        commands = []
        for motor in MotorLocation:
            commands.append(
                MotorCommand(motor = motor, mode = MotorControlMode.RESET)
            )
        self.write_commands(commands)

    def can_bus_connected(self) -> bool:
        '''
        check if the can bus is connected
        this is not the same as checking if motors are connected.
        '''
        if not self._alive():
            return False
        return self._can_connected.value != 0

    def _alive(self):
        if self._motor_proc is None or not self._motor_proc.is_alive():
            return False
        return True

    def shutdown(self):
        '''
        close the interface, this is irreversible
        '''
        if not self._alive():
            self._config.warning_handler('Motor Interface Shutdown Called Unnecessarily')
        # stop the subprocess
        self._should_shutdown.value = 1
        self._motor_proc.join()
        self._motor_proc = None

        # empty and close queues
        while not self._motor_cmd_queue.empty():
            self._motor_cmd_queue.get()
        while not self._motor_telem_queue.empty():
            self._motor_telem_queue.get()

        self._motor_cmd_queue.close()
        self._motor_telem_queue.close()


class _MotorInterface():
    '''
    intermediary, non-user-facing interface to the motors
    '''
    transport: Optional[moteus.Fdcanusb] = None
    controller_fr: Optional[moteus.moteus.Controller] = None
    controller_fl: Optional[moteus.moteus.Controller] = None
    controller_rr: Optional[moteus.moteus.Controller] = None
    controller_rl: Optional[moteus.moteus.Controller] = None
    blocked_addresses: List[int]

    def __init__(
            self,
            config: MotorInterfaceConfig,
            motor_cmd_queue: Queue,
            motor_telem_queue: Queue,
            can_connected: Synchronized,
            should_shutdown: Synchronized
            ):

        self.config = config
        self.motor_cmd_queue = motor_cmd_queue
        self.motor_telem_queue = motor_telem_queue
        self.can_connected = can_connected
        self.should_shutdown = should_shutdown
        self.blocked_addresses = []

        async_loop = asyncio.new_event_loop()
        async_loop.run_until_complete(self._loop())

    async def _loop(self):
        while not self.should_shutdown.value:
            if not self.connected():
                await self.connect()
            if not self.connected():
                # wait before trying again
                time.sleep(0.1)
            else:
                # CAN bus is connected
                self.can_connected.value = 1
                while not self.motor_cmd_queue.empty():
                    await self.parse_command(self.motor_cmd_queue.get())

        self.shutdown()

    async def parse_command(self, cmd: MotorCommand):
        '''
        convert a motor command into an appropriate moteus command and apply it
        parse any returned information and send it out via telemetry queue
        '''
        
        print(f"[_MotorInterface] Processing command: {cmd}")  # Debug print

        if self.config.blacklist_absent_addresses:
            if cmd.motor in self.blocked_addresses:
                print(f"[_MotorInterface] WARNING: {cmd.motor} is blacklisted!")
                return

        if isinstance(cmd.motor, MotorLocation):
            controller = self._get_controller_by_corner(cmd)
        else:
            controller = self._get_controller_by_id(cmd.motor)

        if controller is None:
            # user requested a motor by ID instead of corner and it doesn't exist
            self.config.warning_handler('Motor Command Written to Unavailable CAN ID')
            return

        msgs = MOTOR_CMD_PARSERS.get(cmd.mode)(cmd, controller, self.config.command_timeout)
        try:
            try:
                results = await asyncio.wait_for(
                    self.transport.cycle(msgs),
                    self.config.can_bus_timeout)

                for result in results:
                    self._parse_result(cmd, result)
            except asyncio.exceptions.TimeoutError:
                self.config.warning_handler(f"Motor {cmd.motor} failed to respond.")
                if self.config.blacklist_absent_addresses:
                    self.blocked_addresses.append(
                        cmd.motor
                    )

        except RuntimeError: #TODO - probably does not cover all possible errors
            self.transport = None
            self.can_connected.value = 0

    def _get_controller_by_corner(self, cmd: MotorCommand) -> moteus.moteus.Controller:
        if cmd.motor == MotorLocation.FR:
            controller = self.controller_fr
        elif cmd.motor == MotorLocation.FL:
            controller = self.controller_fl
        elif cmd.motor == MotorLocation.RR:
            controller = self.controller_rr
        elif cmd.motor == MotorLocation.RL:
            controller = self.controller_rl
        else:
            raise NotImplementedError('what have the developers done?')
        return controller

    def _get_controller_by_id(self, motor_id: int) -> Optional[moteus.moteus.Controller]:
        for controller in [self.controller_fr,
                           self.controller_fl,
                           self.controller_rr,
                           self.controller_rl
                           ]:
            if controller.id == motor_id:
                return controller
        return None

    def _parse_result(self, cmd: MotorCommand, result: moteus.moteus.Result):
        if not result:
            # empty result, ie no query so no return
            return

        status = MotorReadout(
            motor = cmd.motor,
            mode = MotorOperationMode(result.values[moteus.moteus.Register.MODE]),
            fault = MotorFaultStatus(result.values[moteus.moteus.Register.FAULT]),
            position = result.values[moteus.moteus.Register.POSITION],
            velocity = result.values[moteus.moteus.Register.VELOCITY],
            torque = result.values[moteus.moteus.Register.TORQUE],
            q_current = result.values[moteus.moteus.Register.Q_CURRENT],
            d_current = result.values[moteus.moteus.Register.D_CURRENT],
            voltage = result.values[moteus.moteus.Register.VOLTAGE],
            temperature = result.values[moteus.moteus.Register.TEMPERATURE],
            power = result.values[moteus.moteus.Register.POWER],
        )

        self.motor_telem_queue.put(status)

    def connected(self):
        ''' is CAN adapter connected? '''
        return self.transport is not None

    async def connect(self):
        ''' connect to CAN adapter '''
        try:
            self.transport = moteus.Fdcanusb()

            print("[_MotorInterface] CAN Bus Connected")  # Debug print

            self.controller_fr = moteus.moteus.Controller(
                id = self.config.id_fr,
                query_resolution = MOTEUS_QUERY_RESOLUTION,
                transport = self.transport,
            )
            self.controller_fl = moteus.moteus.Controller(
                id = self.config.id_fl,
                query_resolution = MOTEUS_QUERY_RESOLUTION,
                transport = self.transport,
            )
            self.controller_rr = moteus.moteus.Controller(
                id = self.config.id_rr,
                query_resolution = MOTEUS_QUERY_RESOLUTION,
                transport = self.transport,
            )
            self.controller_rl = moteus.moteus.Controller(
                id = self.config.id_rl,
                query_resolution = MOTEUS_QUERY_RESOLUTION,
                transport = self.transport,
            )

            controllers = [
                self.controller_fr,
                self.controller_fl,
                self.controller_rr,
                self.controller_rl,
            ]

            for controller in controllers:
                try:
                    stream = moteus.Stream(controller)
                    await stream.command(b'conf set servopos.position_min nan')
                    await stream.command(b'conf set servopos.position_max nan')
                    await stream.command(b'conf set servo.timeout_mode 0')
                    await controller.set_stop()
                    await controller.set_rezero()
                    if controller.id == self.config.id_fr or controller.id == self.config.id_rr:
                        #confirm right side forward
                        await stream.command(b'conf set motor_position.output.sign 1')
                    else:
                        # flip left side signs
                        await stream.command(b'conf set motor_position.output.sign -1')
                except asyncio.exceptions.TimeoutError:
                    pass

        except RuntimeError:
            self.transport = None
            self.can_connected.value = 0

    def shutdown(self):
        ''' shut down the CAN interface '''
        #NOTE - may want to stop the motors here to avoid timeout
        self.can_connected.value = 0


if __name__ == '__main__':
    iface = MotorInterface()
    iface.reset_motors()
    try:
        while True:
            iface.write_command_to_all(
                MotorCommand(
                    mode = MotorControlMode.VELOCITY,
                    velocity = 1,
                    torque = 1,
                    accel_limit = 10,
                )
            )
            time.sleep(0.5)

            readout = iface.read()
            for motor_readout in readout:
                print(motor_readout.motor, 'vel:', motor_readout.velocity)

    except KeyboardInterrupt:
        iface.reset_motors()
        time.sleep(1)
        iface.shutdown()

    # time.sleep(1)
    # #print(iface.can_bus_connected())
    # readout = iface.read()
    # iface.reset_motors()
    # pdb.set_trace()
    # time.sleep(1)
    # iface.shutdown()