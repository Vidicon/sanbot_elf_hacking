import numpy as np
from time import sleep

import serial
import numpy as np
import threading

from Common.sara_common import bodypart_to_string
from Common.sara_common import SaraRobotPartNames
from Common.sara_common import SaraRobotCommands
from Common.sara_common import RobotArmPositions

class BridgeManager:
    def __init__(self, port1, port2, baudrate=115200):
        self.port1 = port1
        self.port2 = port2
        self.baudrate = baudrate
        print(f"BridgeManager initializing")

        # print(f"Port1: {self.port1}")
        # print(f"Port2: {self.port2}")   


    def set_receive_callback(self, callback):
        self.receive_callback = callback


    def connect(self):
        self.com_connection_head = COMConnection(self, self.port1, mainBoard=SaraRobotPartNames.HEAD, baudrate=self.baudrate)
        assert self.com_connection_head.connect() == True, "Not connected to HEAD"

        self.com_connection_body = COMConnection(self, self.port2, mainBoard=SaraRobotPartNames.BODY, baudrate=self.baudrate)
        assert self.com_connection_body.connect() == True, "Not connected to BODY"

        sleep(1)

        # Open all the ports before accepting any data
        self.com_connection_head.set_receive_callback(self.receive_callback)    
        self.com_connection_body.set_receive_callback(self.receive_callback)    

    def disconnect(self):
        print(f"Disconnecting")

        if self.com_connection_head:
            self.com_connection_head.disconnect()

        if self.com_connection_body:
            self.com_connection_body.disconnect()


    def generate_modbus_crc(self, data):
        # Convert the data array to uint8 if it's not already
        data = np.asarray(data, dtype=np.uint8)

        crc = 0xFFFF
        polynomial = 0xA001  # Modbus CRC-16 polynomial

        for byte in data:
            crc ^= int(byte)
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= polynomial
                else:
                    crc >>= 1

        # Swap bytes
        # crc = ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF)
        return crc


    def cmd_createCompassMoveCommand(self, cmd, angle, timeout):
        high = (int(angle) >> 8) & 0xFF
        low = int(angle) & 0xFF

        # Parameter #3 = timeout
        self.cmd_Generic(cmd, 3, np.array([high, low, timeout]))

        return

    def create_message(self, datalength=0):

        data = bytearray([0x00] * (5 + datalength))
        data[0] = 0x55

        return data

    def cmd_Generic(self, cmd: int, datalength, payload: np.array, bodypart=None):
        data = self.create_message(datalength)

        data[0] = 0x55
        data[1] = cmd & 0xFF
        data[2] = datalength & 0xFF

        for i in range(datalength):
            data[3 + i] = payload[i] & 0xFF

        crc = self.generate_modbus_crc(data[:-2])  # skip empty CRC bytes

        # print("> Modbus CRC: 0x", format(crc, '04X'))

        data[datalength + 3] = crc & 0xFF
        data[datalength + 4] = (crc >> 8) & 0xFF

        # determine which port to send the data to
        if bodypart is not None:
            if bodypart == SaraRobotPartNames.HEAD and self.com_connection_head:
                self.com_connection_head.send_data(data)
            elif bodypart == SaraRobotPartNames.BODY and self.com_connection_body:
                self.com_connection_body.send_data(data)
            else:
                print(f"Invalid bodypart specified: {bodypart}. Cannot send data.")
        
        elif cmd >= SaraRobotCommands.CMD_LA_COLOR and cmd <= SaraRobotCommands.CMD_LARA_COLOR:
            if self.com_connection_body:
                self.com_connection_body.send_data(data)
            else:
                print("No connection to BODY. Cannot send data.")

        elif cmd >= SaraRobotCommands.CMD_LA_MOVE and cmd <= SaraRobotCommands.CMD_BASE_BRAKE:
            if self.com_connection_body:
                self.com_connection_body.send_data(data)
            else:
                print("No connection to BODY. Cannot send data.")
        else:  
            print(f"No bodypart specified. Cannot send data with command {cmd}.")

        return

class COMConnection:
    def __init__(self, parent, port, mainBoard="", baudrate=115200):  
        self.running = False
        self.receive_callback = None
        self.mainBoard = mainBoard
        self.baudrate = baudrate
        self.port = port
        self.parent = parent
        self.timeout = 1
        self.serial_port = None
        self.receive_callback = None
        self.receive_thread = None
        self.receive_thread_stop = threading.Event()
    
    def set_receive_callback(self, callback):
        self.receive_callback = callback
    
    def start_receive_thread(self):
        self.receive_thread_stop.clear()
        self.receive_thread = threading.Thread(target=self.receive_serial_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def receive_serial_data(self):
        while not self.receive_thread_stop.is_set():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)
                if self.receive_callback:
                    self.receive_callback(data)        

    def connect(self):
        # print(f"Connecting to port {self.port} at baudrate {self.baudrate}")
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            print(f"Serial port {self.port} opened successfully.")
            self.start_receive_thread()
            return True
        except serial.SerialException as e:
            print(f"{e}")
            return False

    def disconnect(self):
        if self.serial_port:
            self.receive_thread_stop.set()
            self.receive_thread.join()
            self.serial_port.close()
            print(f"Serial port closed.")
        return True
    

    def send_data(self, data):
        if self.serial_port:
            try:
                data_bytes = bytes(data)
                self.serial_port.write(data_bytes)
            except serial.SerialException as e:
                assert False, f"Failed to send data: {e}"

        else:
            assert False, "Serial port is not open. Cannot send data."


# class TCPConnection:
#     def __init__(self, parent, host, port, mainBoard=""):
#         self.parent = parent
#         self.host = host
#         self.port = port
#         self.socket = None
#         self.running = False
#         self.receive_callback = None
#         self.mainBoard = mainBoard

#     def set_receive_callback(self, callback):
#         self.receive_callback = callback

#     def connect(self):
#         try:
#             self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             self.socket.connect((self.host, self.port))
#             self.running = True
#             print(f"Connected to {bodypart_to_string(self.mainBoard)} at {self.host}:{self.port}")
#             threading.Thread(target=self._receive_data, daemon=True).start()
#             return True
#         except Exception as e:
#             print(f"Failed to connect to {self.host}:{self.port} - {e}")
#             return False

#     def send_data(self, data):
#         if self.socket and self.running:
#             try:
#                 # print(f"> Sending data: {data.hex()}")
#                 self.socket.sendall(data)
#             except Exception as e:
#                 print(f"Failed to send data - {e}")
#         else:
#             print("Connection is not active. Cannot send data.")

#     def _receive_data(self):
#         try:
#             while self.running:
#                 data = self.socket.recv(2048)
#                 if data:
#                     pass 
#                 else:
#                     print("Connection closed by the server.")
#                     self.running = False

#                 if self.receive_callback:
#                     self.receive_callback(data)

#         except Exception as e:
#             print(f"Error receiving data - {e}")
#         finally:
#             self.disconnect()

#     def disconnect(self):
#         if self.socket:
#             self.running = False
#             self.socket.close()
#             self.socket = None
#             print(f"Disconnected from {self.host}:{self.port}")