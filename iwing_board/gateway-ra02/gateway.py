import sys
import time
import ctypes
from datetime import datetime, timedelta
import json
import asyncio
import requests

import RPi.GPIO as GPIO
from SX127x.constants import *
from lora import LoRa
from board import BaseBoard

try:
    from config import *
except:
    API_USER = None
    pass

if len(sys.argv) != 2:
    print("Usage: {} <log>".format(sys.argv[0]))
    exit(1)

if sys.argv[1] == '-':
    LOG_FILE = None
else:
    LOG_FILE = open(sys.argv[1],'a')

PKT_TYPE_ADVERTISE = 0x01
PKT_TYPE_REPORT    = 0x02
PKT_TYPE_ACK       = 0x03
PKT_TYPE_BOOT      = 0x04
PKT_TYPE_STRING = 0x05

###########################################################
def log(line):
    print(datetime.now(),line)
    if LOG_FILE is not None:
        print(datetime.now(),line,file=LOG_FILE)
        LOG_FILE.flush()

###########################################################
class BaseStruct(ctypes.Structure):

    _pack_ = 1

    @classmethod
    def size(cls):
        return ctypes.sizeof(cls)

    def unpack(self, blob):
        if not isinstance(blob,bytes):
            raise Exception('Byte array expected for blob')
        if ctypes.sizeof(self) != len(blob):
            raise Exception('Size mismatched')
        ctypes.memmove(ctypes.addressof(self), blob, ctypes.sizeof(self))

###########################################################
class StructConfig(BaseStruct):
    _fields_ = [
        ('radio_device_address', ctypes.c_ubyte),
        ('radio_gateway_address', ctypes.c_ubyte),
        ('radio_freq', ctypes.c_float),
        ('radio_tx_power', ctypes.c_ubyte),
        ('collect_interval_day', ctypes.c_ushort),
        ('collect_interval_night', ctypes.c_ushort),
        ('day_start_hour', ctypes.c_ubyte),
        ('day_end_hour', ctypes.c_ubyte),
        ('time_zone', ctypes.c_byte),
        ('advertise_interval', ctypes.c_ushort),
        ('use_ack', ctypes.c_byte),
        ('ack_timeout', ctypes.c_ushort),
        ('long_range', ctypes.c_ubyte),
        ('tx_repeat', ctypes.c_ubyte),
        ('gps_max_wait_for_fix', ctypes.c_ushort),
        ('next_collect_no_fix', ctypes.c_ushort),
        ('total_slots', ctypes.c_ushort),
        ('slot_interval', ctypes.c_ushort),
        ('prog_file_name', ctypes.c_char*10),
    ]

###########################################################
class StructReport(BaseStruct):
    _fields_ = [
        ('year', ctypes.c_ubyte),
        ('month', ctypes.c_ubyte),
        ('day', ctypes.c_ubyte),
        ('hour', ctypes.c_ubyte),
        ('minute', ctypes.c_ubyte),
        ('second', ctypes.c_ubyte),
        ('vbat', ctypes.c_ushort),    # unit of mV
        ('latitude', ctypes.c_long),
        ('longitude', ctypes.c_long), # unit of 1/100000 degrees
        ('quality', ctypes.c_ubyte),
        ('satellites', ctypes.c_ubyte),
        ('temperature', ctypes.c_ushort),
        ('last_heard_from_gw', ctypes.c_ulong),
    ]

###########################################################
class StructPktBoot(BaseStruct):
    _fields_ = [
        ('type', ctypes.c_ubyte),
        ('firmware', ctypes.c_ushort),
        ('device_model', ctypes.c_ubyte),
        ('reset_flags', ctypes.c_ubyte),
        ('config', StructConfig),
    ]

###########################################################
class StructPktReport(BaseStruct):
    _fields_ = [
        ('type', ctypes.c_ubyte),
        ('seq', ctypes.c_ubyte),
        ('report', StructReport),
    ]

###########################################################
def get_token(user,passwd):
    requests.packages.urllib3.disable_warnings()

    headers = {
        "content-type": "application/json"
    }

    body_json = {
        "username": user,
        "password": passwd,
    }

    resp = requests.post(API_AUTH_TOKEN,json.dumps(body_json),headers=headers,verify=False)
    response_json = resp.json()
    receivedToken = response_json["token"]

    return receivedToken

###########################################################
def upload_location(token,data):
    headers = {
            "content-type": "application/json",
            "Authorization": "Bearer "+ token,
        }
    resp=requests.post(API_LOCATION,json.dumps(data),headers=headers,verify=False)
    return resp.status_code

#JSON Web Token generated from our web app
with open(API_TOKEN_FILE) as f:
    auth_token = f.read()
head = {'Authorization': 'Bearer ' + auth_token}

###########################################################
def upload(data):
    to_upload = {
        'nodeId':      data['device'],
        'gatewayId':   GATEWAY_ID,
        'created_at':  datetime.timestamp(data['recorded_time'])*1e3,
        'rx_at':       datetime.timestamp(data['received_time'])*1e3,
        'lat':         data['lat'],
        'lng':         data['lon'],
        'temperature': data['temperature'],
        'vBat':        data['vbat'],
        'quality':     data['quality'],
        'satellites':  data['satellites'],
        'rssi':        data['rssi'],
        'ttf':         data['last_heard_from_gw'],
    }
    response = requests.post(API_URL, json=to_upload, headers=head)
    print("Server response:", response.text)


###########################################################
class Board0(BaseBoard):
    # Note that the BCM numbering for the GPIOs is used.
    DIO0    = 24
    DIO1    = 6
    RST     = 17
    LED     = 4
    SPI_BUS = 0
    SPI_CS  = 8

#################################################
config = {
    'board' : Board0,
    'bw'    : BW.BW125,
    'freq'  : 434.0,
    'cr'    : CODING_RATE.CR4_8,
    'sf'    : 12,
}

board = config['board']
GPIO.setmode(GPIO.BCM)
GPIO.setup(board.SPI_CS, GPIO.OUT)
GPIO.output(board.SPI_CS, GPIO.HIGH)
board.setup()
board.reset()

###########################################################
class MyLora(LoRa):

    def __init__(self, board, verbose=False):
        super().__init__(board,verbose=verbose)
        self.board = board
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.rx_avail = False

    @property
    def name(self):
        return self.board.__name__

    def on_rx_done(self):
        self.rx_avail = True

    def on_pkt_boot(self, src, pkt, rssi):
        boot = StructPktBoot()
        boot.unpack(pkt)
        my_addr = boot.config.radio_device_address
        gw_addr = boot.config.radio_gateway_address
        log(f'Device booting reported from {src}, RSSI={rssi}, with parameters:')
        log(f' * Firmware version: {boot.firmware}')
        log(f' * Device model: 0x{boot.device_model:02x}')
        log(f' * Reset flags: 0x{boot.reset_flags:02x}')
        log(f' * radio_freq = {boot.config.radio_freq:.2f} MHz')
        log(f' * radio_tx_power = {boot.config.radio_tx_power} dBm')
        log(f' * radio_device_address = {my_addr} (0x{my_addr:02X})')
        log(f' * radio_gateway_address = {gw_addr} (0x{gw_addr:02X})')
        log(f' * collect_interval_day = {boot.config.collect_interval_day} sec')
        log(f' * collect_interval_night = {boot.config.collect_interval_night} sec')
        log(f' * day_start_hour = {boot.config.day_start_hour}')
        log(f' * day_end_hour = {boot.config.day_end_hour}')
        log(f' * time_zone = {boot.config.time_zone} hours')
        log(f' * use_ack = {boot.config.use_ack}')
        log(f' * ack_timeout = {boot.config.ack_timeout} sec')
        log(f' * long_range = {boot.config.long_range}')
        log(f' * tx_repeat = {boot.config.tx_repeat}')
        log(f' * gps_max_wait_for_fix = {boot.config.gps_max_wait_for_fix} sec')
        log(f' * next_collect_no_fix = {boot.config.next_collect_no_fix} sec')
        log(f' * total_slots = {boot.config.total_slots}')
        log(f' * slot_interval = {boot.config.slot_interval} sec')

    def on_pkt_report(self, src, pkt, rssi):
        report_pkt = StructPktReport()
        report_pkt.unpack(pkt)
        report = report_pkt.report
        log('LOG: {} 20{:02}-{:02}-{:02} {:02}:{:02}:{:02} {} {} {} {} {} {} {} {}'.format(
            src,
            report.year,
            report.month,
            report.day,
            report.hour,
            report.minute,
            report.second,
            report.latitude,
            report.longitude,
            report.vbat,
            report.quality,
            report.satellites,
            report.temperature,
            report.last_heard_from_gw,
            rssi,
        ))
        try:
            rxtime = datetime.now()
            logtime = datetime(2000+report.year,
                               report.month,
                               report.day,
                               report.hour,
                               report.minute,
                               report.second)
            logtime += timedelta(hours=7)
        except ValueError:
            return
        data = {
            "recorded_time"      : logtime,
            "received_time"      : rxtime.now(),
            "device"             : src,
            "lat"                : report.latitude/1e5,
            "lon"                : report.longitude/1e5,
            "vbat"               : report.vbat/1e3,
            "quality"            : report.quality,
            "satellites"         : report.satellites,
            "temperature"        : report.temperature,
            "last_heard_from_gw" : report.last_heard_from_gw/1000,
            "rssi"               : rssi,
        }
        try: # attempt to connect to CatTrack web service
            upload(data)
            log("Data uploaded to web service")
        except:
            log("Failed to upload data to web service")
            pass

    async def start(self):
        log(f"[{self.name}] START")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT) # Receiver mode

        while True:
            while not self.rx_avail:
                await asyncio.sleep(0.001)
            self.rx_avail = False
            self.board.led_on()
            rssi = self.get_pkt_rssi_value()
            payload = self.read_payload(nocheck=True) # mean "do not check CRC!"
            pkt_error = not self.rx_is_good()
            self.clear_irq_flags(RxDone=1, PayloadCrcError=1)
            if pkt_error:
                log(f'[{self.name}] CRC ERROR (no data written)')
                continue
            try:
                log(f'got data: {bytes(payload).decode("utf-8")}')
            except:
                log(f'error data {payload}')
            '''src = payload[1]
            dst = payload[0]
            pkt = bytes(bytearray(payload[4:]))
            self.board.led_off()
            if pkt[0] == PKT_TYPE_BOOT:
                self.on_pkt_boot(src, pkt, rssi)
            elif pkt[0] == PKT_TYPE_REPORT:
                self.on_pkt_report(src, pkt, rssi)
            elif pkt[0] == PKT_TYPE_ACK:
                log(f'ACK packet received from 0x{src:02X}')
            elif pkt[0] == PKT_TYPE_ADVERTISE:
                log(f'ADVERTISE packet received from 0x{src:02X}')
            elif pkt[0] == PKT_TYPE_STRING:
                msg = pkt[1:].decode('utf-8', errors='ignore')
                log(f'STRING packet received from 0x{src:02X}: {msg}')
            else:
                log(f'Unknown packet of size {len(pkt)} bytes received from 0x{src:02X}')
            '''
###########################################################

if __name__ == "__main__":
    lora = MyLora(board)
    lora.set_pa_config(pa_select=1, max_power=21, output_power=15)
    lora.set_bw(config['bw'])
    lora.set_freq(config['freq'])
    lora.set_coding_rate(config['cr'])
    lora.set_spreading_factor(config['sf'])
    lora.set_rx_crc(True)
    lora.set_low_data_rate_optim(True)
    assert(lora.get_agc_auto_on() == 1)

    try:
        loop = asyncio.get_event_loop()
        loop.create_task(lora.start())
        loop.run_forever()
    except KeyboardInterrupt:
        sys.stdout.flush()
        log("Keyboard interrupt; exit")
    finally:
        sys.stdout.flush()
        log("Exit")
        lora.set_mode(MODE.SLEEP)
        lora.board.teardown()