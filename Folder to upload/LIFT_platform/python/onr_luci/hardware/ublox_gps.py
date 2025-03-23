'''
code to interface with ublox gps for luci rover
it is expected that the gps has been preconfigured with the following:
    output format: UBX only
    necessary outputs: UBX-NAV-PVT (others ignored)
'''

import time
import sys
from dataclasses import dataclass, field
from multiprocessing import Process, Queue, Value
from multiprocessing.sharedctypes import Synchronized
from typing import Optional, List, Generator
import platform
from enum import Enum

from serial.tools import list_ports
import serial
import pyubx2
from pyubx2 import UBXReader

from onr_luci.pytypes import PythonMsg

import pdb

UNABELED_USB_PORTS = platform.system() in ['Windows']

class GPSFixType(Enum):
    ''' mapping of GPS fix type '''
    NO_FIX = 0
    DEAD_RECKONING = 1
    FIX_2D = 2
    FIX_3D = 3
    FIX_2D_DEAD_RECKONING = 4
    TIME_ONLY = 5

VALID_FIXES = [2,3,4]

@dataclass
class GPSFix(PythonMsg):
    '''
    Data object for a single gps fix

    Based on the UBX-NAV-PVT message in the 
    uBlox ZED-F9P Interface Description
    (available from uBlox webpage)
    UBX-NAV-PVT message is described in section 3.15.13 of that document
    '''

    year: int = field(default = 0)
    ''' current year in UTC time system '''
    month: int = field(default = 0)
    ''' current month in UTC time system '''
    day: int = field(default = 0)
    ''' current day in UTC time system '''
    hour: int = field(default = 0)
    ''' current hour in UTC time system '''
    min: int = field(default = 0)
    ''' current minute in UTC time system '''
    second: int = field(default = 0)
    ''' current second in UTC time system '''
    nano: int = field(default = 0)
    ''' current nanoseconds in UTC time system '''

    t_acc: float = field(default = 0.)
    ''' time fix accuracy in seconds '''

    fix_type: GPSFixType = field(default = GPSFixType.NO_FIX)
    ''' gps fix type '''
    num_sv: int = field(default = 0)
    ''' number of satellites '''

    lon: float = field(default = 0.)
    ''' longitude in degrees, positive east '''
    lat: float = field(default = 0.)
    ''' latitude in degrees, positive north '''
    height: float = field(default = 0.)
    ''' wgs84 ellipsoid height in meters '''
    h_msl: float = field(default = 0.)
    ''' mean sea level height in meters '''
    h_acc: float = field(default = 0.)
    ''' horizontal accuracy (ie. lat/lon position) in meters '''
    v_acc: float = field(default = 0.)
    ''' vertical accuracy in meters '''
    vel_n: float = field(default = 0.)
    ''' north velocity, m/s '''
    vel_e: float = field(default = 0.)
    ''' east velocity, m/s '''
    vel_d: float = field(default = 0.)
    ''' down velocity, m/s '''
    g_speed: float = field(default = 0.)
    ''' ground speed m/s '''
    s_acc: float = field(default = 0.)
    ''' speed accuracy m/s '''
    head_mot: float = field(default = 0.)
    ''' heading of motion (degrees) '''
    head_veh: float = field(default = 0.)
    ''' not currently populated '''
    head_acc: float = field(default = 0.)
    ''' heading accuracy (degrees) '''

    def repeat_print(self):
        ''' repeatedly print in a clean manner'''
        self.print()
        for _ in vars(self):
            sys.stdout.write("\033[F") # Cursor up one line
            sys.stdout.write("\033[K") # clear line

    def from_pyubx(self, parsed_data):
        ''' unpack data from pyubx interface '''
        self.year = parsed_data.year
        self.month = parsed_data.month
        self.day = parsed_data.month
        self.hour = parsed_data.hour
        self.min = parsed_data.min
        self.second = parsed_data.second
        self.nano = parsed_data.nano

        self.t_acc = parsed_data.tAcc * 1e-9

        self.fix_type = GPSFixType(parsed_data.fixType)

        self.num_sv = parsed_data.numSV

        self.lon = parsed_data.lon
        self.lat = parsed_data.lat
        self.height = parsed_data.height * 1e-3
        self.h_msl = parsed_data.hMSL * 1e-3
        self.h_acc = parsed_data.hAcc * 1e-3
        self.v_acc = parsed_data.vAcc * 1e-3
        self.vel_n = parsed_data.velN * 1e-3
        self.vel_e = parsed_data.velE * 1e-3
        self.vel_d = parsed_data.velD * 1e-3
        self.g_speed = parsed_data.gSpeed * 1e-3
        self.s_acc = parsed_data.sAcc * 1e-3
        self.head_mot = parsed_data.headMot
        self.head_veh = parsed_data.headVeh
        self.head_acc = parsed_data.headAcc

        if pyubx2.__version__[0] == '0':
            self.lon *= 1e-7
            self.lat *= 1e-7
            self.head_mot *= 1e-5
            self.head_veh *= 1e-5
            self.head_acc *= 1e-5

@dataclass
class GPSConfig(PythonMsg):
    ''' configuration for a GPS connection '''
    verbose: bool = field(default = True)
    # seconds between attempts to find and connect device
    reconnect_interval: float = field(default = 1)
    baudrate: int = field(default = 921600)
    serial_timeout: float = field(default = 1)

    usb_identifier: str = field(default = 'u-blox GNSS receiver')
    gps_address: Optional[str] = field(default = None)


class _GPSProcess:
    '''
    class meant for use as a subprocess for a single gps device.
    status.value is a shared value indicating the mode:
        -1: shutdown
        0:  not connected
        1:  connected with GPS fix
        2:  connected without GPS fix

    write -1 to should_close to end the process.

    '''

    config: GPSConfig
    msg: GPSFix
    output_handler: callable

    last_connect_attempt: float = None
    last_time_connected: float = None

    conn: serial.Serial = None
    reader: UBXReader = None

    def __init__(self,
            config: GPSConfig,
            status: Synchronized,
            should_close: Synchronized,
            fix_output_queue: Queue,
            correction_queue: Queue,
            output_handler = print):

        self.config = config
        self.msg = GPSFix()
        self.output_handler = output_handler

        self.status = status
        self.should_close = should_close

        self.fix_output_queue  = fix_output_queue
        self.correction_queue  = correction_queue

        self.run()

    def run(self):
        ''' run the GPS process'''
        # for timeout to self-delete, ie. if port number has changed
        self.last_time_connected = time.time()
        while not self.should_close.value:
            if not self._connected():
                self._connect()
            if not self._connected():
                time.sleep(0.1)
            else:
                self.last_time_connected = time.time()
                self._read()
            self._read_corrections()
            self._update_status()
        self._close()

    def _connected(self):
        return self.conn is not None

    def _connect(self):
        if not self._connect_frequency_limit():
            self._open_serial_port()

    def _connect_frequency_limit(self):
        if not self.last_connect_attempt:
            self.last_connect_attempt = time.time()
            return False
        limit = time.time() - self.last_connect_attempt < 1
        if not limit:
            self.last_connect_attempt = time.time()

        return limit

    def _open_serial_port(self):
        if self.config.gps_address is None:
            ports = get_available_gps_ports()
            if ports:
                self.config.gps_address = ports[0]
            else:
                return
        try:
            self.conn = serial.Serial(self.config.gps_address,
                                      baudrate = self.config.baudrate,
                                      timeout = self.config.serial_timeout)
            self.reader = UBXReader(self.conn)
        except serial.SerialException as e:
            print(e)
            self.config.gps_address = None
            self._disconnect()
        self.msg = GPSFix()

    def _disconnect(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except serial.SerialException:
                pass
            self.conn = None

    def _read(self):
        try:
            _, msg = self.reader.read()
            # print(msg)
            if msg.identity == 'NAV-PVT':
                fix = GPSFix()
                fix.from_pyubx(msg)
                if fix.fix_type.value in VALID_FIXES: #NOTE - may change based on user preference
                    self.fix_output_queue.put(fix)
        except serial.SerialException:
            self._disconnect()
        except AttributeError as e:
            print(e)
            return

    def _read_corrections(self):
        # should always be called so input buffer does not clog
        while not self.correction_queue.empty():
            msg = self.correction_queue.get()
            if self._connected():
                try:
                    self.conn.write(msg)
                except serial.SerialException:
                    self._disconnect()

    def _update_status(self):
        if self.should_close.value:
            self.status.value = -1

        if self.status.value == -1:
            return
        if not self._connected():
            self.status.value = 0
        elif self.msg.fix_type in VALID_FIXES:
            self.status.value = 1
        else:
            self.status.value = 2

    def _close(self):
        self._disconnect()


class GPSClient:
    ''' higher level interface to GPS'''
    _output_handler: callable

    _config: GPSConfig
    _device: _GPSProcess

    def __init__(self,
            config: Optional[GPSConfig] = None,
            output_handler = print):
        self._output_handler = output_handler
        self._config = config

        if self._config is None:
            self._config = GPSConfig()

        self._gps_connected = Value('d', 0)
        self._should_shutdown = Value('d', 0)
        self._gps_fix_queue = Queue()
        self._gps_correction_queue = Queue()

        self._gps_proc = Process(
            target = _GPSProcess,
            args = (self._config,
                    self._gps_connected,
                    self._should_shutdown,
                    self._gps_fix_queue,
                    self._gps_correction_queue,
                    self._output_handler)
        )
        self._gps_proc.start()

    def read(self) -> List[GPSFix]:
        ''' read for gps fixes '''
        return list(self._read())

    def connected(self) -> bool:
        ''' check if the gps is connected '''
        return self._gps_connected.value > 0

    def _read(self) -> Generator[GPSFix, None, None]:
        while not self._gps_fix_queue.empty():
            yield self._gps_fix_queue.get()

    def write_corrections(self, corrections: bytes):
        ''' write corrections to GPS devices '''
        self._gps_correction_queue.put(corrections)

    def _alive(self):
        if self._gps_proc is None or not self._gps_proc.is_alive():
            return False
        return True

    def shutdown(self):
        '''
        close the interface, this is irreversible
        '''
        if not self._alive():
            self._output_handler('Motor Interface Shutdown Called Unnecessarily')
        # stop the subprocess
        self._should_shutdown.value = 1
        self._gps_proc.join()
        self._gps_proc = None

        # empty and close queues
        while not self._gps_correction_queue.empty():
            self._gps_correction_queue.get()
        while not self._gps_fix_queue.empty():
            self._gps_fix_queue.get()

        self._gps_correction_queue.close()
        self._gps_fix_queue.close()


def get_available_gps_ports(usb_identifier: Optional[str] = 'u-blox GNSS receiver') -> List[str]:
    ''' get serial ports matching the given identifier '''
    ports = list_ports.comports()
    print(list(ports))

    gps_ports = []
    for port in ports:
        if usb_identifier is None or UNABELED_USB_PORTS:
            gps_ports.append(port.device)
        elif port.product is not None and usb_identifier in port.product:
            gps_ports.append(port.device)
    return gps_ports

def test_gps():
    ''' test one GPS device '''
    gps = GPSClient()

    time.sleep(1)
    for msg in gps.read():
        msg.print()
    gps.shutdown()


if __name__ == '__main__':
    #test_gps(
    print(GPSFixType(1))
