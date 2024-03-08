import serial
import time
import socket
import threading
import logging
import struct
from textwrap import wrap
from bitarray import bitarray
import paho.mqtt.client as mqtt
import json
try:
    import construct as cs
except ImportError:
    print("You are missing dependencies")
    print("To install use:")
    print("    python -m pip install 'construct'")


# Checksum checksum8_mod256
def calcCheckSum8Mod256(data: str) -> str:
    """
    :return:  The checksum computed with the method of 'sum and modulo 256'.
    """
    checksum = sum(data[:299]) % 256
    return checksum
# --------------------------------------------------------------------------- #
# Calculate the modbus CRC
# --------------------------------------------------------------------------- #
def calcCrc16(data, size):
    crcHi = 0XFF
    crcLo = 0xFF
    
    crcHiTable	= [	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
                    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
                    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
                    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
                    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
                    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
                    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
                    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
                    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
                    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
                    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
                    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
                    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40]

    crcLoTable = [  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
                    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
                    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
                    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
                    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
                    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
                    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
                    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
                    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
                    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
                    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
                    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
                    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
                    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
                    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
                    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
                    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
                    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
                    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
                    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
                    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
                    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
                    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
                    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
                    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
                    0x43, 0x83, 0x41, 0x81, 0x80, 0x40]

    index = 0
    while index < size:
        crc = crcHi ^ data[index]
        crcHi = crcLo ^ crcHiTable[crc]
        crcLo = crcLoTable[crc]
        index += 1

    metCRC16 = (crcHi * 0x0100) + crcLo
    return metCRC16

def onConnect():
    def onConnect(client, userdata, flags, rc):
        if rc == 0:
            logging.info("Connected to MQTT Broker!")
        else:
            logging.error("Failed to connect, return code %d\n", rc)
    # Set Connecting Client ID
    client = mqtt_client.Client(MQTT_CLIENT_ID)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.onConnect = onConnect
    client.connect(MQTT_BROKER, MQTT_PORT)
    return client

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

def onDisconnect(client, userdata, rc):
    logging.info("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logging.info("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            logging.info("Reconnected successfully!")
            return
        except Exception as err:
            logging.error("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)
    global FLAG_EXIT
    FLAG_EXIT = True

def onMessage(client, userdata, msg):
    logging.info(f'Received `{msg.payload.decode()}` from `{msg.topic}` topic')

def connectMqtt():
    client = mqtt_client(MQTT_CLIENT_ID)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = onConnect
    client.on_message = onMessage
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=120)
    client.on_disconnect = onDisconnect
    return client



# JK ESS PB BMS Record Type 1 - Config Details 0x1000
def getConfig (data):
    logging.debug(f"getConfig Function")
    config = {}
    # strip header
    dataOnly = data[6:]
    # Data Byte 0
    config['VolSmartSleep'] = int.from_bytes(dataOnly[0:4], byteorder='little')
    config['VolCellUV'] = int.from_bytes(dataOnly[4:8], byteorder='little')
    config['VolCellUVPR'] = int.from_bytes(dataOnly[8:12], byteorder='little')
    config['VolCellOV'] = int.from_bytes(dataOnly[12:16], byteorder='little')
    config['VolCellOVPR'] = int.from_bytes(dataOnly[16:20], byteorder='little')
    config['VolBalanTrig'] = int.from_bytes(dataOnly[20:24], byteorder='little')
    config['VolSOC100%'] = int.from_bytes(dataOnly[24:28], byteorder='little')
    config['VolSOC0%'] = int.from_bytes(dataOnly[28:32], byteorder='little')
    config['VolCellRCV'] = int.from_bytes(dataOnly[32:36], byteorder='little')
    config['VolCellRFV'] = int.from_bytes(dataOnly[36:40], byteorder='little')
    config['VolSysPwrOff'] = int.from_bytes(dataOnly[40:44], byteorder='little')
    config['CurBatCOC'] = int.from_bytes(dataOnly[44:48], byteorder='little')
    config['TIMBatCOCPDly'] = int.from_bytes(dataOnly[48:52], byteorder='little')
    config['TIMBatCOCPRDly'] = int.from_bytes(dataOnly[52:56], byteorder='little')
    config['CurBatDcOC'] = int.from_bytes(dataOnly[56:60], byteorder='little')
    config['TIMBatDcOCPDly'] = int.from_bytes(dataOnly[60:64], byteorder='little')
    config['TIMBatDcOCPRDly'] = int.from_bytes(dataOnly[64:68], byteorder='little')
    config['TIMBatSCPRDly'] = int.from_bytes(dataOnly[68:72], byteorder='little')
    config['CurBalanMax'] = int.from_bytes(dataOnly[72:76], byteorder='little')
    config['TMPBatCOT'] = int.from_bytes(dataOnly[76:80], byteorder='little', signed=True)
    config['TMPBatCOTPR'] = int.from_bytes(dataOnly[80:84], byteorder='little', signed=True)
    config['TMPBatDcOT'] = int.from_bytes(dataOnly[84:88], byteorder='little', signed=True)
    config['TMPBatDcOTPR'] = int.from_bytes(dataOnly[88:92], byteorder='little', signed=True)
    config['TMPBatCUT'] = int.from_bytes(dataOnly[92:96], byteorder='little', signed=True)
    config['TMPBatCUTPR'] = int.from_bytes(dataOnly[96:100], byteorder='little', signed=True)
    config['TMPMosOT'] = int.from_bytes(dataOnly[100:104], byteorder='little', signed=True)
    config['TMPMosOTPR'] = int.from_bytes(dataOnly[104:108], byteorder='little', signed=True)
    config['CellCount'] = int.from_bytes(dataOnly[108:112], byteorder='little')
    config['BatChargeEN'] = int.from_bytes(dataOnly[112:116], byteorder='little')
    config['BatDisChargeEN'] = int.from_bytes(dataOnly[116:120], byteorder='little')
    config['BalanEN'] = int.from_bytes(dataOnly[120:124], byteorder='little')
    config['CapBatCell'] = int.from_bytes(dataOnly[124:128], byteorder='little')
    config['SCPDelay'] = int.from_bytes(dataOnly[128:132], byteorder='little')
    config['VolStartBalan'] = int.from_bytes(dataOnly[132:136], byteorder='little')
    config['CellConWireRes0'] = int.from_bytes(dataOnly[136:140], byteorder='little')
    config['CellConWireRes1'] = int.from_bytes(dataOnly[140:144], byteorder='little')
    config['CellConWireRes2'] = int.from_bytes(dataOnly[144:148], byteorder='little')
    config['CellConWireRes3'] = int.from_bytes(dataOnly[148:152], byteorder='little')
    config['CellConWireRes4'] = int.from_bytes(dataOnly[152:156], byteorder='little')
    config['CellConWireRes5'] = int.from_bytes(dataOnly[156:160], byteorder='little')
    config['CellConWireRes6'] = int.from_bytes(dataOnly[160:164], byteorder='little')
    config['CellConWireRes7'] = int.from_bytes(dataOnly[164:168], byteorder='little')
    config['CellConWireRes8'] = int.from_bytes(dataOnly[168:172], byteorder='little')
    config['CellConWireRes9'] = int.from_bytes(dataOnly[172:176], byteorder='little')
    config['CellConWireRes10'] = int.from_bytes(dataOnly[176:180], byteorder='little')
    config['CellConWireRes11'] = int.from_bytes(dataOnly[180:184], byteorder='little')
    config['CellConWireRes12'] = int.from_bytes(dataOnly[184:188], byteorder='little')
    config['CellConWireRes13'] = int.from_bytes(dataOnly[188:192], byteorder='little')
    config['CellConWireRes14'] = int.from_bytes(dataOnly[192:196], byteorder='little')
    config['CellConWireRes15'] = int.from_bytes(dataOnly[196:200], byteorder='little')
    config['CellConWireRes16'] = int.from_bytes(dataOnly[200:204], byteorder='little')
    config['CellConWireRes17'] = int.from_bytes(dataOnly[204:208], byteorder='little')
    config['CellConWireRes18'] = int.from_bytes(dataOnly[208:212], byteorder='little')
    config['CellConWireRes19'] = int.from_bytes(dataOnly[212:216], byteorder='little')
    config['CellConWireRes20'] = int.from_bytes(dataOnly[216:220], byteorder='little')
    config['CellConWireRes21'] = int.from_bytes(dataOnly[220:224], byteorder='little')
    config['CellConWireRes22'] = int.from_bytes(dataOnly[224:228], byteorder='little')
    config['CellConWireRes23'] = int.from_bytes(dataOnly[228:232], byteorder='little')
    config['CellConWireRes24'] = int.from_bytes(dataOnly[232:236], byteorder='little')
    config['CellConWireRes25'] = int.from_bytes(dataOnly[236:240], byteorder='little')
    config['CellConWireRes26'] = int.from_bytes(dataOnly[240:244], byteorder='little')
    config['CellConWireRes27'] = int.from_bytes(dataOnly[244:248], byteorder='little')
    config['CellConWireRes28'] = int.from_bytes(dataOnly[248:252], byteorder='little')
    config['CellConWireRes29'] = int.from_bytes(dataOnly[252:256], byteorder='little')
    config['CellConWireRes30'] = int.from_bytes(dataOnly[256:260], byteorder='little')
    config['CellConWireRes31'] = int.from_bytes(dataOnly[260:264], byteorder='little')
    config['DevAddr'] = int.from_bytes(dataOnly[264:268], byteorder='little')
    config['TIMProdischarge'] = int.from_bytes(dataOnly[268:272], byteorder='little')
    CfgEnable = bitarray()
    CfgEnable.frombytes(dataOnly[276:278])
    config['HeatEN'] = CfgEnable[0]
    config['DisableTempSensor'] = CfgEnable[1]
    config['GPSHeartbeat'] = CfgEnable[2]
    config['PortSwitch'] = CfgEnable[3]
    config['LCDAlwaysOn'] = CfgEnable[4]
    config['SpecialCharger'] = CfgEnable[5]
    config['SmartSleep'] = CfgEnable[6]
    config['DisablePCLModule'] = CfgEnable[7]
    config['TimedStoredData'] = CfgEnable[8]
    config['ChargingFloatMode'] = CfgEnable[9]
    config['TIMSmartSleep'] = int.from_bytes(dataOnly[280:281], byteorder='little')
    config['FieldEnableControl0'] = int.from_bytes(dataOnly[281:282], byteorder='little')

    # Return Dict with all values
    return config
record_01 = cs.Struct(
    "VolSmartSleep" / cs.Int32ul,
    "VolCellUV" / cs.Int32ul,
    "VolCellUVPR" / cs.Int32ul,
    "VolCellOV" / cs.Int32ul,
    "VolCellOVPR" / cs.Int32ul,
    "VolBalanTrig" / cs.Int32ul,
    "VolSOC100%" / cs.Int32ul,
    "VolSOC0%" / cs.Int32ul,
    "VolCellRCV" / cs.Int32ul,
    "VolCellRFV" / cs.Int32ul,
    "VolSysPwrOff" / cs.Int32ul,
    "CurBatCOC" / cs.Int32ul,
    "TIMBatCOCPDly" / cs.Int32ul,
    "TIMBatCOCPRDly" / cs.Int32ul,
    "CurBatDcOC" / cs.Int32ul,
    "TIMBatDcOCPDly" / cs.Int32ul,
    "TIMBatDcOCPRDly" / cs.Int32ul,
    "TIMBatSCPRDly" / cs.Int32ul,
    "CurBalanMax" / cs.Int32ul,
    "TMPBatCOT" / cs.Int32sl,
    "TMPBatCOTPR" / cs.Int32sl,
    "TMPBatDcOT" / cs.Int32sl,
    "TMPBatDcOTPR" / cs.Int32sl,
    "TMPBatCUT" / cs.Int32sl,
    "TMPBatCUTPR" / cs.Int32sl,
    "TMPMosOT" / cs.Int32sl,
    "TMPMosOTPR" / cs.Int32sl,
    "CellCount" / cs.Int32ul,
    "BatChargeEN" / cs.Int32ul,
    "BatDisChargeEN" / cs.Int32ul,
    "BalanEN" / cs.Int32ul,
    "CapBatCell" / cs.Int32ul,
    "SCPDelay" / cs.Int32ul,
    "VolStartBalan" / cs.Int32ul,
    "0CellConWireRes0" / cs.Int32ul,
    "1CellConWireRes1" / cs.Int32ul,
    "2CellConWireRes2" / cs.Int32ul,
    "3CellConWireRes3" / cs.Int32ul,
    "4CellConWireRes4" / cs.Int32ul,
    "5CellConWireRes5" / cs.Int32ul,
    "6CellConWireRes6" / cs.Int32ul,
    "7CellConWireRes7" / cs.Int32ul,
    "8CellConWireRes8" / cs.Int32ul,
    "9CellConWireRes9" / cs.Int32ul,
    "10CellConWireRes10" / cs.Int32ul,
    "11CellConWireRes11" / cs.Int32ul,
    "12CellConWireRes12" / cs.Int32ul,
    "13CellConWireRes13" / cs.Int32ul,
    "14CellConWireRes14" / cs.Int32ul,
    "15CellConWireRes15" / cs.Int32ul,
    "16CellConWireRes16" / cs.Int32ul,
    "17CellConWireRes17" / cs.Int32ul,
    "18CellConWireRes18" / cs.Int32ul,
    "19CellConWireRes19" / cs.Int32ul,
    "20CellConWireRes20" / cs.Int32ul,
    "21CellConWireRes21" / cs.Int32ul,
    "22CellConWireRes22" / cs.Int32ul,
    "23CellConWireRes23" / cs.Int32ul,
    "24CellConWireRes24" / cs.Int32ul,
    "25CellConWireRes25" / cs.Int32ul,
    "26CellConWireRes26" / cs.Int32ul,
    "27CellConWireRes27" / cs.Int32ul,
    "28CellConWireRes28" / cs.Int32ul,
    "29CellConWireRes29" / cs.Int32ul,
    "30CellConWireRes30" / cs.Int32ul,
    "31CellConWireRes31" / cs.Int32ul,
    "DevAddr" / cs.Int32ul,
    "TIMProdischarge" / cs.Int32ul,
    "CfgEnable" / cs.BitStruct(
        HeatEN=cs.Enum(cs.Bit, DISABLED=0, Closed=1),
        DisableTempSensor=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        GPSHeartbeat=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        PortSwitch=cs.Enum(cs.Bit, CAN=0, RS485=1),
        LCDAlwaysOn=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        SpecialCharger=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        SmartSleep=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        DisablePCLModule=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        TimedStoredData=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        ChargingFloatMode=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        UnusedBit10=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        UnusedBit11=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        UnusedBit12=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        UnusedBit13=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        UnusedBit14=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
        UnusedBit15=cs.Enum(cs.Bit, DISABLED=0, ENABLED=1),
    ),
    "TIMSmartSleep" / cs.Int8ul,
    "FieldEnableControl0" / cs.Int8ul,
    #"rest" / cs.GreedyBytes,
)
# JK ESS PB BMS Record Type 2 - Status Details 0x1200
def getState (data):
    logging.debug(f"getState Function")
    state = {}
    # strip header
    dataOnly = data[6:]
    # Data Byte 0 with 
    #print(f"Normal    Len:{len(data)}: {data.hex()}")
    #print(f"Data Only Len:{len(data[6:])}: {data[6:].hex()}")
    state['CellVol0'] = int.from_bytes(dataOnly[0:2], byteorder='little')
    state['CellVol1'] = int.from_bytes(dataOnly[2:4], byteorder='little')
    state['CellVol2'] = int.from_bytes(dataOnly[4:6], byteorder='little')
    state['CellVol3'] = int.from_bytes(dataOnly[6:8], byteorder='little')
    state['CellVol4'] = int.from_bytes(dataOnly[8:10], byteorder='little')
    state['CellVol5'] = int.from_bytes(dataOnly[10:12], byteorder='little')
    state['CellVol6'] = int.from_bytes(dataOnly[12:14], byteorder='little')
    state['CellVol7'] = int.from_bytes(dataOnly[14:16], byteorder='little')
    state['CellVol8'] = int.from_bytes(dataOnly[16:18], byteorder='little')
    state['CellVol9'] = int.from_bytes(dataOnly[18:20], byteorder='little')
    state['CellVol10'] = int.from_bytes(dataOnly[20:22], byteorder='little')
    state['CellVol11'] = int.from_bytes(dataOnly[22:24], byteorder='little')
    state['CellVol12'] = int.from_bytes(dataOnly[24:26], byteorder='little')
    state['CellVol13'] = int.from_bytes(dataOnly[26:28], byteorder='little')
    state['CellVol14'] = int.from_bytes(dataOnly[28:30], byteorder='little')
    state['CellVol15'] = int.from_bytes(dataOnly[30:32], byteorder='little')
    # ignore Cell16-31
    # CellSta Cell Status #64 Bits 0-31 for 4 bytes 
    CellStaArray = bitarray()
    CellStaArray.frombytes(dataOnly[64:68])
    state['CellSta0'] = CellStaArray[0]
    state['CellSta1'] = CellStaArray[1]
    state['CellSta2'] = CellStaArray[2]
    state['CellSta3'] = CellStaArray[3]
    state['CellSta4'] = CellStaArray[4]
    state['CellSta5'] = CellStaArray[5]
    state['CellSta6'] = CellStaArray[6]
    state['CellSta7'] = CellStaArray[7]
    state['CellSta8'] = CellStaArray[8]
    state['CellSta9'] = CellStaArray[9]
    state['CellSta10'] = CellStaArray[10]
    state['CellSta11'] = CellStaArray[11]
    state['CellSta12'] = CellStaArray[12]
    state['CellSta13'] = CellStaArray[13]
    state['CellSta14'] = CellStaArray[14]
    state['CellSta15'] = CellStaArray[15]
    # Ignore 16-31 cells
    state['CellVolAve'] = int.from_bytes(dataOnly[68:70], byteorder='little')
    state['CellVdifMax'] = int.from_bytes(dataOnly[70:72], byteorder='little')
    state['MaxVolCellNbr'] = int.from_bytes(dataOnly[72:73], byteorder='little')
    state['MinVolCellNbr'] = int.from_bytes(dataOnly[73:74], byteorder='little')
    # CellWireRes Byte74 1-31
    state['CellWireRes0'] = int.from_bytes(dataOnly[74:76], byteorder='little')
    state['CellWireRes1'] = int.from_bytes(dataOnly[76:78], byteorder='little')
    state['CellWireRes2'] = int.from_bytes(dataOnly[78:80], byteorder='little')
    state['CellWireRes3'] = int.from_bytes(dataOnly[80:82], byteorder='little')
    state['CellWireRes4'] = int.from_bytes(dataOnly[82:84], byteorder='little')
    state['CellWireRes5'] = int.from_bytes(dataOnly[84:86], byteorder='little')
    state['CellWireRes6'] = int.from_bytes(dataOnly[86:88], byteorder='little')
    state['CellWireRes7'] = int.from_bytes(dataOnly[88:90], byteorder='little')
    state['CellWireRes8'] = int.from_bytes(dataOnly[90:92], byteorder='little')
    state['CellWireRes9'] = int.from_bytes(dataOnly[92:94], byteorder='little')
    state['CellWireRes10'] = int.from_bytes(dataOnly[94:96], byteorder='little')
    state['CellWireRes11'] = int.from_bytes(dataOnly[96:98], byteorder='little')
    state['CellWireRes12'] = int.from_bytes(dataOnly[98:100], byteorder='little')
    state['CellWireRes13'] = int.from_bytes(dataOnly[100:102], byteorder='little')
    state['CellWireRes14'] = int.from_bytes(dataOnly[102:104], byteorder='little')
    state['CellWireRes15'] = int.from_bytes(dataOnly[104:106], byteorder='little')
    # Skip cells 16-31
    state['TempMos'] = int.from_bytes(dataOnly[138:140], byteorder='little', signed=True)
    # CellWireResSta Cell Status #140 Bits 0-31 for 4 bytes 
    CellWireResSta = bitarray()
    CellWireResSta.frombytes(dataOnly[140:144])
    state['CellWireResSta0'] = CellWireResSta[0]
    state['CellWireResSta1'] = CellWireResSta[1]
    state['CellWireResSta2'] = CellWireResSta[2]
    state['CellWireResSta3'] = CellWireResSta[3]
    state['CellWireResSta4'] = CellWireResSta[4]
    state['CellWireResSta5'] = CellWireResSta[5]
    state['CellWireResSta6'] = CellWireResSta[6]
    state['CellWireResSta7'] = CellWireResSta[7]
    state['CellWireResSta8'] = CellWireResSta[8]
    state['CellWireResSta9'] = CellWireResSta[9]
    state['CellWireResSta10'] = CellWireResSta[10]
    state['CellWireResSta11'] = CellWireResSta[11]
    state['CellWireResSta12'] = CellWireResSta[12]
    state['CellWireResSta13'] = CellWireResSta[13]
    state['CellWireResSta14'] = CellWireResSta[14]
    state['CellWireResSta15'] = CellWireResSta[15]
    state['BatVol'] = int.from_bytes(dataOnly[144:148], byteorder='little')
    state['BatWatt'] = int.from_bytes(dataOnly[148:152], byteorder='little')
    state['BatCurrent'] = int.from_bytes(dataOnly[152:156], byteorder='little', signed=True)
    state['TempBat1'] = int.from_bytes(dataOnly[156:158], byteorder='little', signed=True)
    state['TempBat2'] = int.from_bytes(dataOnly[158:160], byteorder='little', signed=True)
    # Alarms Array Byte 160 = 4 bytes into bits
    BatAlarms = bitarray()
    BatAlarms.frombytes(dataOnly[160:164])
    state['AlarmWireRes'] = BatAlarms[0]
    state['AlarmMosOTP'] = BatAlarms[1]
    state['AlarmCellQuantity'] = BatAlarms[2]
    state['AlarmCurSensorErr'] = BatAlarms[3]
    state['AlarmCellOVP'] = BatAlarms[4]
    state['AlarmBatOVP'] = BatAlarms[5]
    state['AlarmChOCP'] = BatAlarms[6]
    state['AlarmChSCP'] = BatAlarms[7]
    state['AlarmChOTP'] = BatAlarms[8]
    state['AlarmChUTP'] = BatAlarms[9]
    state['AlarmCPUAuxCommuErr'] = BatAlarms[10]
    state['AlarmCellUVP'] = BatAlarms[11]
    state['AlarmBatUVP'] = BatAlarms[12]
    state['AlarmDchOCP'] = BatAlarms[13]
    state['AlarmDchSCP'] = BatAlarms[14]
    state['AlarmDchOTP'] = BatAlarms[15]
    state['AlarmChargeMOS'] = BatAlarms[16]
    state['AlarmDischargeMOS'] = BatAlarms[17]
    state['GPSDisconneted'] = BatAlarms[18]
    state['AlarmModifyPwd'] = BatAlarms[19]
    state['AlarmDischargeOnFailed'] = BatAlarms[20]
    state['AlarmBatteryOverTemp'] = BatAlarms[21]
    state['AlarmTempSenAnomaly'] = BatAlarms[22]
    state['AlarmPLCModuleAnomaly'] = BatAlarms[23]
    state['BalanCurrent'] = int.from_bytes(dataOnly[164:166], byteorder='little', signed=True)
    state['BalanSta'] = int.from_bytes(dataOnly[166:167], byteorder='little')
    state['SOCStateOfCharge'] = int.from_bytes(dataOnly[167:168], byteorder='little')
    state['SOCCapRemain'] = int.from_bytes(dataOnly[168:172], byteorder='little', signed=True)
    state['SOCFullChargeCap'] = int.from_bytes(dataOnly[172:176], byteorder='little', signed=False)
    state['SOCCycleCount'] = int.from_bytes(dataOnly[176:180], byteorder='little', signed=False)
    state['SOCCycleCap'] = int.from_bytes(dataOnly[180:184], byteorder='little', signed=False)
    state['SOCSOH'] = int.from_bytes(dataOnly[184:185], byteorder='little')
    state['Precharge'] = int.from_bytes(dataOnly[185:186], byteorder='little')
    state['UserAlarm'] = int.from_bytes(dataOnly[186:188], byteorder='little')
    state['RunTime'] = int.from_bytes(dataOnly[188:192], byteorder='little', signed=False)
    state['Charge'] = int.from_bytes(dataOnly[192:193], byteorder='little')
    state['DisCharge'] = int.from_bytes(dataOnly[193:194], byteorder='little')
    state['UserAlarm2'] = int.from_bytes(dataOnly[194:196], byteorder='little')
    state['TimeDcOCPR'] = int.from_bytes(dataOnly[196:198], byteorder='little')
    state['TimeDcSCPR'] = int.from_bytes(dataOnly[198:200], byteorder='little')
    state['TimeCOCPR'] = int.from_bytes(dataOnly[200:202], byteorder='little')
    state['TimeCSCPR'] = int.from_bytes(dataOnly[202:204], byteorder='little')
    state['TimeUVPR'] = int.from_bytes(dataOnly[204:206], byteorder='little')
    state['TimeOVPR'] = int.from_bytes(dataOnly[206:208], byteorder='little')
    # TempSensor Alarms Array Byte 208 = 1 bytes into bits
    TempSenAlarms = bitarray()
    TempSenAlarms.frombytes(dataOnly[208:209])
    state['AlarmMOSTempSenAbsent'] = TempSenAlarms[0]
    state['AlarmTempSen1Absent'] = TempSenAlarms[1]
    state['AlarmTempSen2Absent'] = TempSenAlarms[2]
    state['AlarmTempSen3Absent'] = TempSenAlarms[3]
    state['AlarmTempSen4Absent'] = TempSenAlarms[4]
    state['AlarmTempSen5Absent'] = TempSenAlarms[5]
    state['AlarmTempSen1Absent'] = TempSenAlarms[0]
    state['Heating'] = int.from_bytes(dataOnly[209:210], byteorder='little')
    # Byte 210 Reserved
    state['TimeEmergency'] = int.from_bytes(dataOnly[212:214], byteorder='little')
    state['BatDisCurCorrect'] = int.from_bytes(dataOnly[214:216], byteorder='little')
    state['VolChargCur'] = int.from_bytes(dataOnly[216:218], byteorder='little')
    state['VolDischargCur'] = int.from_bytes(dataOnly[218:220], byteorder='little')
    state['BatVolCorrect'] = struct.unpack('<f', dataOnly[220:224])[0]
    state['BatVol'] = int.from_bytes(dataOnly[228:230], byteorder='little')
    state['HeatCurrent'] = int.from_bytes(dataOnly[230:232], byteorder='little', signed=True)
    state['RVD'] = int.from_bytes(dataOnly[238:239], byteorder='little')
    state['ChargerPlugged'] = int.from_bytes(dataOnly[239:240], byteorder='little')
    state['SysRunTicks'] = int.from_bytes(dataOnly[240:244], byteorder='little')
    # Byte 244 - 248  Reserved
    state['TempBat3'] = int.from_bytes(dataOnly[248:250], byteorder='little', signed=True)
    state['TempBat4'] = int.from_bytes(dataOnly[250:252], byteorder='little', signed=True)
    state['TempBat5'] = int.from_bytes(dataOnly[252:254], byteorder='little', signed=True)
    state['RTCTicks'] = int.from_bytes(dataOnly[256:260], byteorder='little')
    state['TimeEnterSleep'] = int.from_bytes(dataOnly[264:268], byteorder='little')
    state['PCLModuleSta'] = int.from_bytes(dataOnly[268:269], byteorder='little')
    state['RVD'] = int.from_bytes(dataOnly[269:270], byteorder='little')
    # Return Dict with all values
    return state

record_02 = cs.Struct(
    "cell_voltage_array" / cs.Array(32, cs.Int16ul),
    "cell_presence" / cs.BitStruct("cells" / cs.Array(32, cs.Enum(cs.Bit, not_present=0, present=1))),
    "CellVolAve" / cs.Int16ul,
    "CellVdifMax" / cs.Int16ul,
    "MaxVolCellNbr" / cs.Byte,
    "MinVolCellNbr" / cs.Byte,
    "cell_resistance_array" / cs.Array(32, cs.Int16ul),
    "TempMos" / cs.Int16ul,
    "CellWireResStatus" / cs.BitStruct("cells:" / cs.Array(32, cs.Enum(cs.Bit, OK=0, ALARM=1))),
    "BatVoltage" / cs.Int32ul,
    "BatWatt" / cs.Int32ul,
    "BatCurrent" / cs.Int32sl,
    "TempBat1" / cs.Int16ul,
    "TempBat2" / cs.Int16ul,
    "Alarms" / cs.BitStruct(
        AlarmWireRes=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmMosOTP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmCellQuantity=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmCurSensorErr=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmCellOVP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmBatOVP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmChOCP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmChSCP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmChOTP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmChUTP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmCPUAuxCommuErr=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmCellUVP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmBatUVP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmDchOCP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmDchSCP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmDchOTP=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmChargeMOS=cs.Enum(cs.Bit, OK=0, ALARM=1),
        AlarmDischargeMOS=cs.Enum(cs.Bit, OK=0, ALARM=1),
        GPSDisconneted=cs.Enum(cs.Bit, OK=0, ALARM=1),
        ModifyPWDinTime=cs.Enum(cs.Bit, OK=0, ALARM=1),
        DischargeOnFailed=cs.Enum(cs.Bit, OK=0, ALARM=1),
        BatteryOverTemp=cs.Enum(cs.Bit, OK=0, ALARM=1),
        TemperatureSensorAnomaly=cs.Enum(cs.Bit, OK=0, ALARM=1),
        PLCModuleAnomaly=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit24=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit25=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit26=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit27=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit28=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit29=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit30=cs.Enum(cs.Bit, OK=0, ALARM=1),
        UnusedBit31=cs.Enum(cs.Bit, OK=0, ALARM=1)
    ),
    "BalanCurrent" / cs.Int16sl,
    "BalanStatus" /  cs.Enum(cs.Int8ul, off=0, charge=1, discharge=2),
    "SOCStateOfCharge" / cs.Int8ul,
    "SOCCapRemain" / cs.Int32ul,
    "SOCFullChargeCap" / cs.Int32ul,
    "SOCCycleCount" / cs.Int32ul,
    "SOCCycleCap" / cs.Int32ul,
    "SOCSOH" / cs.Int8ul,
    "Precharge" /  cs.Enum(cs.Int8ul, closed=0, open=1),
    "UserAlarm" / cs.Int16sl,
    "Runtime" / cs.Int32ul,
    "Charge" /  cs.Enum(cs.Int8ul, closed=0, open=1),
    "Discharge" /  cs.Enum(cs.Int8ul, closed=0, open=1),
    "UserAlarm2" / cs.Int16sl,
    "TimeDcOCPR" / cs.Int16ul,
    "TimeDcSCPR" / cs.Int16ul,
    "TimeCOCPR" / cs.Int16ul,
    "TimeCSCPR" / cs.Int16ul,
    "TimeUVPR" / cs.Int16ul,
    "TimeOVPR" / cs.Int16ul,
    "SensorAlarms" / cs.BitStruct(
        MOSTempSensorAbsent=cs.Enum(cs.Bit, OK=1, ALARM=0),
        BATTempSensor1Absent=cs.Enum(cs.Bit, OK=1, ALARM=0),
        BATTempSensor2Absent=cs.Enum(cs.Bit, OK=1, ALARM=0),
        BATTempSensor3Absent=cs.Enum(cs.Bit, OK=1, ALARM=0),
        BATTempSensor4Absent=cs.Enum(cs.Bit, OK=1, ALARM=0),
        BATTempSensor5Absent=cs.Enum(cs.Bit, OK=1, ALARM=0),
        Unusedbit6=cs.Enum(cs.Bit, OK=1, ALARM=0),
        Unusedbit7=cs.Enum(cs.Bit, OK=1, ALARM=0),
    ),
    "Heating" /  cs.Enum(cs.Int8ul, closed=0, open=1),
    "Reseved210" / cs.Bytes(16),
    "TimeEmergency" / cs.Int16ul,
    "BatDisCurCorrect" / cs.Int16ul,
    "VolChargCur" / cs.Int16ul,
    "VolDischargCur" / cs.Int16ul,
    "BatDisCurCorrect" / cs.Float32b,
    "BatVol" / cs.Int16ul,
    "HeatCurrent" / cs.Int16ul,
    "RVD" / cs.Int8ul,
    "ChargerPlugged" /  cs.Enum(cs.Int8ul, disconnected=0, connected=1),
    "SysRunTicks" / cs.Int32ul,
    "TempBat3" / cs.Int16ul,
    "TempBat4" / cs.Int16ul,
    "TempBat5" / cs.Int16ul,
    "RTCTicks" / cs.Int32ul,
    "TimeEnterSleep" / cs.Int32ul,
    "PCLModuleSta" /  cs.Enum(cs.Int8ul, closed=0, open=1),
     # "rest" / cs.GreedyBytes,
)


# JK ESS PB BMS Record Type 3 - Device Details 0x1400
record_03 = cs.Struct(
    "ManufacturerDeviceID" / cs.PaddedString(16, "utf8"),
    "HardwareVersion" / cs.PaddedString(8, "utf8"),
    "SoftwareVersion" / cs.PaddedString(8, "utf8"),
    "ODDRunTime" / cs.Int32ul,
    "PWROnTimes" / cs.Int32ul,
     "rest" / cs.GreedyBytes,
)
jkess_definition = cs.Struct(
   "header" / cs.Bytes(4),
   "record_type" / cs.Byte,
   "record_source" / cs.Enum(cs.Byte, Master=0, Slave=5),
   "data" / cs.Switch(cs.this.record_type, {1: record_01, 2: record_02, 3: record_03}),
)

def read_serial(port, baudrate, buffer_size):
    ser = serial.Serial(port, baudrate)
    allData = b''
    frag = False
    fragCount = 0
    id = ''
    currentData = {}
    logging.info(f"Start Reading from the serial port {port} with baudrate {baudrate}")
    try:
        while True:
            # Read a chunk of data from the serial port
            data = ser.read_all()
            # Check if there is data to read from the serial
            if len(data) > 1:
                # Identify Modbus FC16 (0x10) Write Multiple registers Request
                # Request size: UnitIdentifier (1) + FunctionCode (1) + WriteAddress (2) + WriteQuantity (2) + WriteByteCount (1) + WriteData (n) + CRC (2)
                if (len(data) in [11]) and data.hex()[2:4] in ['10']:
                    id = int(data.hex()[:2],16)
                    fc = int(data.hex()[2:4],16)
                    writeAddr = data.hex()[4:8]
                    writeQty = data.hex()[8:12]
                    writeByteQount = data.hex()[12:14]
                    writeDataIdx = (14+(int(writeByteQount,16)*2))
                    writeData = data.hex()[14:writeDataIdx]
                    crc16 = int(data.hex()[writeDataIdx:(writeDataIdx+4)],16)
                    resultCrc16 = calcCrc16(data, 7+int(writeByteQount,16))
                    crcOK = crc16 == resultCrc16
                    logging.debug(f"Write Multiple registers Request ID: {id} FC{fc} writeAddr:{writeAddr} writeQty:{writeQty} writeByteQount:{writeByteQount} writeData:{writeData} crcOK:{crcOK} {len(data)}bytes: {data.hex()}")
                    # Identify Write Register Config data Request 
                    if crcOK and writeAddr == '161e':
                        logging.debug(f"Master 0 Request Config Data Slave ID: {id} Addr:{writeAddr} {len(data)}bytes: {data.hex()}")
                    # Identify Write Register Status data Request 
                    if crcOK and writeAddr == '1620':
                        logging.debug(f"Master 0 Request Status Data Slave ID: {id} Addr:{writeAddr} {len(data)}bytes: {data.hex()}")
                # Identify Modbus FC16 (0x10) Write Multiple registers Response
                # Responce size: UnitIdentifier (1) + FunctionCode (1) + WriteAddress (2) + WriteQuantity (2) + CRC (2)
                elif (len(data) in [8]) and data.hex()[2:4] in ['10']:
                    id = int(data.hex()[:2],16)
                    fc = int(data.hex()[2:4],16)
                    writeAddr = data.hex()[4:8]
                    writeQty = data.hex()[8:12]
                    crc16 = int(data.hex()[12:16],16)
                    resultCrc16 = calcCrc16(data, 6)
                    crcOK = crc16 == resultCrc16
                    logging.debug(f"Write Multiple registers Response ID: {id} FC{fc} writeAddr:{writeAddr} writeQty:{writeQty} crcOK:{crcOK} {len(data)}bytes: {data.hex()}")
                    # Identify Write Register Config data Response 
                    if crcOK and writeAddr == '161e':
                        logging.debug(f"Slave ID: {id} Response -> Master 0 Config Data Addr:{writeAddr} {len(data)}bytes: {data.hex()}")
                    # Identify Write Register Status data Response 
                    if crcOK and writeAddr == '1620':
                        logging.debug(f"Slave ID: {id} Response -> Master 0 Status Data Addr:{writeAddr} {len(data)}bytes: {data.hex()}")

                # Identify JK BMS Custom Data Packet
                ###############################################
                elif data.hex().startswith('55aaeb90'):
                    # Extract the fragment count from Byte 4 
                    fragCount = data[3] >>4
                    logging.debug(f"DEBUG Fragments: {data[3] >>4}")
                    frag = True
                    allData += data
                    logging.debug(f"Debug Frag {fragCount}: {data.hex()}")
                    data = b''
                    fragCount = fragCount - 1
                # If fragment assemble was started and still being assembled    
                elif (frag and fragCount > 0):
                    if data:
                        allData += data
                        logging.debug(f"Fragment {fragCount} {len(data)}bytes: {data.hex()}")
                        data = b''
                        fragCount = fragCount - 1
                # If last fragment detected and final assembly need to be done
                elif (frag and fragCount == 0):
                    allData += data
                    logging.debug(f"Fragment {fragCount} {len(data)}bytes: {data.hex()}")
                    data = allData
                    logging.debug(f"Rx Assembled - {len(allData)}: {allData.hex()}")
                    allData = b''
                    fragCount = 0
                    frag = False
                    # Identify Data packets 300bytes and bigger with valid checksum
                    if len(data) >= 300 and data[299] == calcCheckSum8Mod256(data):
                        # Parse JK ESS BMS Record data
                        # Extract JK recordType
                        recordType = int(data[4:5].hex())

                        # Extract Modbus recordSource
                        if int(data[5:6].hex()) == 0:
                            recordSource = 'Master'
                            # When master set ID to 0 as there is no modbus request for this data
                            id = 0
                        else:
                            recordSource = 'Slave'
                        logging.info(f"DEBUG Length  {len(data)} Type: {recordType} device id: {id}")

                        # Detect new Packs
                        if id not in currentData:
                            logging.info(f"New Pack ID:{id} discovered")
                            currentData[id] = {}
                            currentData[id]['config'] = {}
                            currentData[id]['state'] = {}
                            currentData[id]['info'] = {}
                        logging.debug(f"Raw Data ID: {id} Rx-{len(data)}bytes: {data.hex()}")

                        # Record Type 1 is used for Configuration data
                        if recordType == 1:
                            currentData[id]['config'] = getConfig(data)
                            logging.info(f"Config Data {recordSource}-> ID: {id}, Type: {recordType}, Device Addr: {currentData[id]['config'].get('DevAddr',None)} Cells: {currentData[id]['config'].get('CellCount',None)} Checksum: {hex(data[299])} CmpChecksum: {hex(calcCheckSum8Mod256(data))}")
                            logging.debug(f"Device: {id} State: {currentData[id]['config']}")
                        # Record Type 2 is used for State data
                        if recordType == 2:
                            currentData[id]['state'] = getState(data)
                            logging.info(f"State Data {recordSource}-> ID: {id}, Type: {recordType}, SoC: {currentData[id]['state'].get('SOCStateOfCharge',None)}% Checksum: {hex(data[299])} CmpChecksum: {hex(calcCheckSum8Mod256(data))}")
                            logging.debug(f"Device: {id} State: {currentData[id]['state']}")
                        # Record Type 3 is used for Device Info Data
                        if recordType == 3:
                            #logging.info(f"{result.record_source}-> ID: {id}, Type: {result.record_type}, Source: {result.record_source}, DeviceID: {result.data.ManufacturerDeviceID} HW_Ver: {result.data.HardwareVersion} SW_Ver: {result.data.SoftwareVersion}")
                            logging.debug(f"Raw Data Record Type 3: ID: {id} Rx-{len(data)}bytes: {data.hex()}")
                    else:
                        logging.critical(f"BAD Checksum Device ID: {id} Packet Lenght:{len(data)} Data: {data.hex()}")
                else:
                    logging.debug(f"DEBUG: UNKNOWN Packet {len(data)}bytes Data: {data.hex()}")
            ###############################################
            #print(f"{currentData}")
                

    except serial.SerialException as e:
        logging.error("Serial port error:", e)
    finally:
        ser.close()  # Close the serial port

# Application Start
MONITOR_HOST = socket.gethostname()
        
# MQTT Broker Configuration
MQTT_BROKER = "10.0.10.25"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "jkess-mqtt-1"
MQTT_USERNAME = "iot"
MQTT_PASSWORD = "!@QWASZX"
MQTT_TOPIC = "tele"
MQTT_TOPIC_DISCOVERY = "homeassistant/binary_sensor/bms/config"

# Example usage:
port = '/dev/ttyUSB0'  # Example serial port
baudrate = 115200  # Example baud rate
buffer_size = 64  # Example buffer size in bytes

# Start a new thread to continuously read from the serial port
serial_thread = threading.Thread(target=read_serial, args=(port, baudrate, buffer_size))
serial_thread.daemon = False  # Allow the program to exit even if the thread is still running
serial_thread.start()



if __name__ == "__main__":
    logging.basicConfig(format=f"%(asctime)-15s \033[36m%(levelname)-8s\033[0m: %(message)s", level=logging.INFO)
    logging.debug('This is a debug message')
    logging.info('This is an info message')
    logging.warning('This is a warning message')
    logging.error('This is an error message')
    logging.critical('This is a critical message')
    # Initialize MQTT client
    #mqtt_client = mqtt.Client()
    #client = connectMqtt()
    #client.loop_start()
    time.sleep(1)
# The main thread can continue executing other tasks or wait for user input
#input("Press Enter to exit...\n")