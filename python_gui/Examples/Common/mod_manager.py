import serial
import numpy as np
import threading


class ModManager:
    def __init__(self, port1, port2, baudrate=57600, timeout=1):
        self.port1 = port1
        self.port2 = port2
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.receive_callback = None
        self.receive_thread = None
        self.receive_thread_stop = threading.Event()

    def open_port(self):
        try:
            self.serial_port = serial.Serial(port=self.port1, baudrate=self.baudrate, timeout=self.timeout)
            print(f"Serial port {self.port1} opened successfully.")
            self.start_receive_thread()
        except serial.SerialException as e:
            print(f"{e}")

            try:
                self.serial_port = serial.Serial(port=self.port2, baudrate=self.baudrate, timeout=self.timeout)
                print(f"Serial port {self.port2} opened successfully.")
                self.start_receive_thread()
            except serial.SerialException as e:
                print(f"{e}")

    def close_port(self):
        if self.serial_port:
            self.receive_thread_stop.set()
            self.receive_thread.join()
            self.serial_port.close()
            print(f"Serial port closed.")

    def send_data(self, data):
        if self.serial_port:
            try:
                data_bytes = bytes(data)
                self.serial_port.write(data_bytes)
                # print(f"Data sent: {data}")
            except serial.SerialException as e:
                assert False, f"Failed to send data: {e}"

        else:
            assert False, "Serial port is not open. Cannot send data."

    def set_receive_callback(self, callback):
        self.receive_callback = callback
        # self.stream_area = stream_area

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
                    # self.receive_callback(data, self.stream_area)
                    self.receive_callback(data)

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

    def print_array_as_hex(self, data):
        hex_values = " ".join([format(x, "02X") for x in data])
        print("> " + hex_values)

    def send_enable(self, ID, enable):
        datalength = 2
        data = self.create_message(datalength)

        data[0] = 0x55
        data[1] = 0x01
        data[2] = datalength
        data[3] = ID
        data[4] = enable

        crc = self.generate_modbus_crc(data[:-2])  # skip empty CRC bytes

        print("> Modbus CRC: 0x", format(crc, "04X"))

        data[datalength + 3] = crc & 0xFF
        data[datalength + 4] = (crc >> 8) & 0xFF

        self.print_array_as_hex(data)

        self.send_data(data)

        return

    def create_message(self, datalength=0):

        data = bytearray([0x00] * (5 + datalength))
        data[0] = 0x55

        return data

    def cmd_GetVersion(self):
        datalength = 0
        data = self.create_message(datalength)

        data[0] = 0x55
        data[1] = 0x40
        data[2] = datalength

        crc = self.generate_modbus_crc(data[:-2])  # skip empty CRC bytes

        data[datalength + 3] = crc & 0xFF
        data[datalength + 4] = (crc >> 8) & 0xFF

        self.print_array_as_hex(data)

        self.send_data(data)

        return

    def cmd_Generic(self, cmd: int, datalength, payload: np.array):
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

        # self.print_array_as_hex(data)
        self.send_data(data)

        return

    def cmd_createCompassMoveCommand(self, cmd, angle):
        high = (int(angle) >> 8) & 0xFF
        low = int(angle) & 0xFF

        self.cmd_Generic(cmd, 2, np.array([high, low]))

        return
