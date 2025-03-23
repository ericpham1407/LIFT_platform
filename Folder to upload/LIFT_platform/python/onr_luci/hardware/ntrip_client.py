'''
NTRIP client for getting RTK corrections
'''
import socket
import base64
import time
from typing import Dict
from dataclasses import dataclass, field
from multiprocessing import Process, Queue, Value, Array
from multiprocessing.sharedctypes import Synchronized
from ctypes import c_char

import numpy as np

from onr_luci.hardware.ublox_gps import GPSFix
from onr_luci.pytypes import PythonMsg

DEFAULT_TIMEOUT = 5
STREAM_ATTEMPT_TIMEOUT = 1
MOUNTPOINT_SWAP_DELAY = 10

@dataclass
class NtripConfig(PythonMsg):
    ''' config for NTRIP connection '''
    host: str = field(default = 'rtgpsout.unavco.org')   #NTRIP hostname
    port: int = field(default = 2101)                    #NTRIP port
    mount: str = field(default = 'P181_RTCM3')           #NTRIP mountpoint
    user:  str = field(default = 'tfork')                #NTRIP username
    psk:  str = field(default = '9x0kGxCw')              #NTRIP password
    eol:   str = field(default = '\r\n'.encode('utf-8')) #NTRIP end of line

    timeout = 5
    sourcetable_timeout = 10

    load_sourcetable: bool = field(default = False)
    '''
    load a table of all mountpoints avaialble (all base stations)
    this will also enable the client to change mountpoint on the fly, e.g. if the
    initially-provided mountpoint is farther away than another base station

    to always use the same base station, set load_sourcetable to False
    '''

    max_mount_length: int = field(default = 20)
    '''
    does not impact NTRIP functionality.
    This is for sharing NTRIP mountpoint out of child processes and is necessary
    for allocating a shared memory array for this purpose.
    If too short, ntrip mountpoints are truncated for NtripClient.get_mountpoint,
    the full mointpoint label is still used for connection.
    '''


class _NtripConnection:
    '''
    base class for an NTRIP connection
    may be a sourcetable lookup or a stream request
    see server request functions
        get_source_request_str
        get_stream_request_str
    '''
    config: NtripConfig
    def __init__(self, config: NtripConfig, output_handler = print):
        self.output_handler = output_handler

        self.config = config
        self.connected = False
        self.socket = None

    def get_source_request_str(self) -> bytes:
        '''
        HTTP requiest for NTRIP sourcetable
        '''
        encoded_credentials = base64.b64encode(
            (self.config.user + ':' + self.config.psk).encode('ascii'))
        EOL = self.config.eol
        server_request =  'GET /HTTP/1.0'.encode('utf-8') + EOL + \
                          'User-Agent: NTRIP ABC/1.2.3'.encode('utf-8') + EOL + \
                          'Accept: */*'.encode('utf-8') + EOL + \
                          'Connection: close'.encode('utf-8') + EOL + \
                          'Authorization: Basic '.encode('utf-8') + encoded_credentials + EOL + EOL

        return server_request

    def get_stream_request_str(self) -> bytes:
        '''
        HTTP request for NTRIP correction stream
        '''
        encoded_credentials = base64.b64encode(
            (self.config.user + ':' + self.config.psk).encode('ascii'))
        EOL = self.config.eol
        server_request = (f'GET /{self.config.mount} HTTP/1.0').encode('utf-8') + EOL + \
                          'User-Agent: NTRIP ABC/1.2.3'.encode('utf-8') + EOL + \
                          'Authorization: Basic '.encode('utf-8') + encoded_credentials + EOL + EOL
        return server_request

    def connect(self, timeout = None):
        '''
        opens a socket for NTRIP
        '''
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if timeout:
                self.socket.settimeout(timeout)
            else:
                self.socket.settimeout(DEFAULT_TIMEOUT)
            self.socket.connect((self.config.host, self.config.port))
            self.output_handler('connected to NTRIP')
            self.connected = True
        except socket.error:
            self.output_handler('unable to establish NTRIP connection')
            self.connected = False

    def disconnect(self):
        '''
        deletes the internal socket object
        a new one can be created with connect()

        trying to read from a deleted socket usually raises OSError for bad file descriptor
        which is a good indication if poor use of this class.
        '''
        if self.connected:
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()
            self.connected = False


class _NtripMountpointReader(_NtripConnection):
    '''
    opens a connection to an ntrip server and reads the sourcetable
    then parses the sourcetable for viable mountpoints

    this closes the socket at the end,
    leaving self.mountpoints as a data structure containing the mountpoint information.
    '''
    sourcetable: bytes
    mountpoints: Dict[str, np.ndarray]

    def __init__(self, config: NtripConfig, output_handler = print):
        super().__init__(config, output_handler)
        self.mountpoints = None
        self._read()

    def _read(self):
        self.connect(timeout = self.config.sourcetable_timeout)
        self._read_sourcetable()
        self._parse_sourcetable()

    def _read_sourcetable(self):
        if not self.connected:
            self.mountpoints = None
            return

        self.socket.sendall(self.get_source_request_str())

        read_started = False
        read_ok = False
        t_start = time.time()

        sourcetable = ''

        while time.time() - t_start < self.config.sourcetable_timeout and not read_ok:
            sourcetable += self.socket.recv(1024).decode()

            if not read_started and '200 OK' in sourcetable:
                read_started = True
            if read_started and 'ENDSOURCETABLE' in sourcetable:
                read_ok = True

            time.sleep(0.01)

        self.disconnect()

        if read_ok:
            self.output_handler('Source Table Read OK')
            self.sourcetable = sourcetable
        elif read_started:
            self.output_handler('Source Table End Not Found')
            self.sourcetable = sourcetable
        else:
            self.output_handler('Source Table Not Found')
            self.sourcetable = None

    def _parse_sourcetable(self):
        if not hasattr(self,'sourcetable'):
            self.mountpoints = None
            return

        sources = 0
        mountpoints = []
        lats = []
        lons = []

        lines = self.sourcetable.split(self.config.eol.decode())
        for line in lines:
            if line[0:3] == 'STR': # only look for STR streams
                fields = line.split(';')
                mountpoint = fields[1]
                #data_format = fields[3]
                #nav_system = fields[6]
                lat = fields[9]
                lon = fields[10]
                req_nmea = fields[11] != '0'
                auth = fields[15] != 'B'
                fee = fields[16] != 'N'

                if fee or auth or req_nmea or 'RTCM3' not in mountpoint:
                    self.output_handler(f'unsupported mountpoint: {line}')
                else:
                    sources += 1
                    mountpoints.append(mountpoint)
                    lats.append(float(lat))
                    lons.append(float(lon))

        if sources:
            self.mountpoints = {'mountpoint':np.array(mountpoints),
                                'lat': np.array(lats),
                                'lon': np.array(lons)}

        self.output_handler(f'found {sources} sources')

    def get_mountpoint(self, lat: float, lon: float) -> str:
        ''' get a mountpoint for a given lat/lon location '''
        if self.mountpoints is None:
            return None

        err = (self.mountpoints['lat'] - lat)**2 + (self.mountpoints['lon'] - lon)**2
        idx = np.argmin(err)
        return self.mountpoints['mountpoint'][idx]


class _NtripStream(_NtripConnection):
    '''
    class for creating NTRIP streams

    change self.config for updating mountpoint or credentials
    call begin_stream() to start a stream
    reading from a dead stream will automatically attempt to open one
    '''

    def __init__(self, config:NtripConfig, output_handler = print):
        super().__init__(config, output_handler)
        self.stream_open = False
        self.last_connect_attempt = time.time() - STREAM_ATTEMPT_TIMEOUT

    def _connect_attempt_limit(self):
        if time.time() - self.last_connect_attempt < STREAM_ATTEMPT_TIMEOUT:
            return True
        else:
            self.last_connect_attempt = time.time()
            return False

    def connect(self, timeout = None):
        if self._connect_attempt_limit():
            return

        if self.connected:
            self.disconnect()

        if self.config.timeout is not None:
            timeout = self.config.timeout
        super().connect(timeout = timeout)

        if not self.connected:
            return

        try:
            self.socket.sendall(self.get_stream_request_str())

            t_start = time.time()
            while time.time() - t_start < self.config.timeout:
                msg = self.socket.recv(1024)
                if b'ICY 200' in msg:
                    self.connected = True
                    return
        except socket.error:
            self.output_handler('lost NTRIP connection')
            self.disconnect()
        self.disconnect()

    def read(self):
        ''' read the stream for new data '''
        if not self.connected:
            self.connect()
            if not self.connected:
                return

        try:
            msg = self.socket.recv(1024)
            return msg
        except socket.error:
            self.output_handler('lost NTRIP connection')
            self.disconnect()


class _NtripProcess:
    ''' process for running NTRIP asynchronously '''
    gps_queue: Queue
    correction_queue: Queue
    status: Synchronized
    close: Synchronized
    mountpoint: Synchronized

    dynamic_source: bool
    last_mountpoint_change: float

    gps_msg: GPSFix
    stream: _NtripStream
    reader: _NtripMountpointReader

    def __init__(self,
            config: NtripConfig,
            output_handler = print):
        self.output_handler = output_handler

        self.config = config
        self.status = Value('i',0)
        self.close  = Value('i',0)
        self.mountpoint  = Array(c_char, b' ' * self.config.max_mount_length)
        self.gps_queue = Queue()
        self.correction_queue = Queue()

        self.gps_msg = None
        self.stream = None
        self.last_mountpoint_change = time.time()

    def start(self):
        ''' start the process '''
        self.dynamic_source = self.config.load_sourcetable

        if self.dynamic_source:
            self.read_sourcetable()

        self._connect()
        self.run()

    def run(self):
        ''' run the process '''
        while self.status.value != -1:
            self._read_corrections()

            if self.dynamic_source:
                self._check_mountpoint()
            self._update_status()
            self._check_gps_queue()

    def read_sourcetable(self):
        ''' read sourcetable for dynamic mountpoint '''
        if not self.dynamic_source:
            return
        self.reader = _NtripMountpointReader(self.config, self.output_handler)

        if self.reader.mountpoints is None:
            self.close.value = -1
            self.status.value = -1
            self.output_handler('No sourcetable found, shutting down NTRIP client')

        self.output_handler(f'Defaulting to mountpoint {self.config.mount}')

    def _connect(self):
        self.stream = _NtripStream(self.config, self.output_handler)
        self.mountpoint.value = self.config.mount.encode('utf-8')[: self.config.max_mount_length]

    def _disconnect(self):
        if self.stream:
            self.stream.disconnect()

    def _read_corrections(self):
        msg = self.stream.read()
        if msg:
            self.correction_queue.put(msg)

    def _check_mountpoint(self):
        if self.dynamic_source and self.gps_msg:
            mount = self.reader.get_mountpoint(self.gps_msg.lat, self.gps_msg.lon)

            if mount != self.config.mount and self._mount_swap_limiter():
                self.config.mount = mount
                self._disconnect()

                self.output_handler(f'Swapped to mountpoint {self.config.mount}')

    def _mount_swap_limiter(self):
        if time.time() - self.last_mountpoint_change > MOUNTPOINT_SWAP_DELAY:
            self.last_mountpoint_change = time.time()
            return False
        return True

    def _update_status(self):
        if self.close.value == -1:
            self.status.value = -1
            return
        if self.stream.connected:
            self.status.value = 1
        else:
            self.status.value = 0

    def _check_gps_queue(self):
        while not self.gps_queue.empty():
            msg = self.gps_queue.get()
            if type(msg).__name__ == 'GPSFix':
                self.gps_msg = msg


class NtripClient:
    ''' overall client to run NTRIP '''

    process: _NtripProcess
    job: Process

    def __init__(self, config: NtripConfig, output_handler = print):
        self.config = config
        self.output_handler = output_handler
        self.process = _NtripProcess(self.config, output_handler)

    def start(self):
        ''' start and run the ntrip client '''
        job = Process(target = self.process.start)
        job.start()
        self.job = job

    def step(self, gps_msg: GPSFix = None):
        '''
        step the client, returning any corrections
        if provieded a gps message, it may alter the mount used
        to one that is closer to current location
        '''
        if gps_msg:
            self.process.gps_queue.put(gps_msg)

        data = []
        while not self.process.correction_queue.empty():
            data.append(self.process.correction_queue.get())
        return data

    def is_connected(self):
        ''' check if currently clonnected'''
        return self.process.status.value

    def shutdown(self):
        ''' close the ntrip client '''
        self.process.close.value = -1
        self.output_handler('closing ntrip')

    def get_mountpoint(self) -> str:
        ''' get a string ID of the current mountpoint in use '''
        return self.process.mountpoint.value.decode('utf-8')


def test_mountpoint_reader():
    ''' try reading a mountpoint table '''
    reader = _NtripMountpointReader(NtripConfig())
    print(reader.mountpoints)

def test_ntrip_stream():
    ''' try connecting and printing streamed corrections'''
    cli = NtripClient(NtripConfig())
    cli.start()
    time.sleep(2)
    for _ in range(50):
        corrections = cli.step()
        for correction in corrections:
            print(correction)

        time.sleep(.1)

    cli.shutdown()

if __name__ == '__main__':
    #test_mountpoint_reader()
    test_ntrip_stream()
