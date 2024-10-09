import serial
import numpy as np
import threading

class ModManager:
    def __init__(self, port, baudrate=57600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.receive_callback = None
        self.receive_thread = None
        self.receive_thread_stop = threading.Event()

    def open_port(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f"Serial port {self.port} opened successfully.")
            self.start_receive_thread()           
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.port}: {e}")


    def close_port(self):
        if self.serial_port:
            self.receive_thread_stop.set()
            self.receive_thread.join()
            self.serial_port.close()
            print(f"Serial port {self.port} closed.")

    def send_data(self, data):
        if self.serial_port:
            try:
                data_bytes = bytes(data)
                self.serial_port.write(data_bytes)
                # print(f"Data sent: {data}")
            except serial.SerialException as e:
                print(f"Failed to send data: {e}")
        else:
            print("Serial port is not open. Cannot send data.")

    def set_receive_callback(self, callback, stream_area):
        self.receive_callback = callback
        self.stream_area = stream_area
        
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
                    self.receive_callback(data, self.stream_area)

    def generate_modbus_crc(self, data):
        # Convert the data array to uint8 if it's not already
        data = np.asarray(data, dtype=np.uint8)
        
        crc = 0xFFFF
        polynomial = 0xA001  # Modbus CRC-16 polynomial
    
        for byte in data:
            crc ^= byte
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
        hex_values = ' '.join([format(x, '02X') for x in data])
        print("> " + hex_values)

    def send_enable(self, ID, enable):
        datalength = 2
        data = self.create_message(datalength)
        
        data[0] = 0x55
        data[1] = 0x01
        data[2] = datalength
        data[3] = ID
        data[4] = enable
        
        crc = self.generate_modbus_crc(data[:-2])   # skip empty CRC bytes
        
        print("> Modbus CRC: 0x", format(crc, '04X'))
        
        data[datalength + 3] = (crc & 0xff);
        data[datalength + 4] = ((crc >> 8)& 0xff);
        
        self.print_array_as_hex(data)
        
        self.send_data(data)
        
        return

    def create_message(self, datalength=0):
        
        data = bytearray([0x00] * (5+datalength))
        data[0] = 0x55
        
        return data
    
    # def send_speed(self, ID, speed):
    #     datalength = 3
    #     data = self.create_message(datalength)
        
    #     data[0] = 0x55
    #     data[1] = 0x03  
    #     data[2] = datalength
    #     data[3] = ID
    #     data[4] = (speed >> 8) & 0xff
    #     data[5] = (speed & 0xff)
        
    #     crc = self.generate_modbus_crc(data[:-2])   # skip empty CRC bytes
        
    #     print("Modbus CRC: 0x", format(crc, '04X'))
        
    #     data[datalength + 3] = (crc & 0xff);
    #     data[datalength + 4] = ((crc >> 8)& 0xff);
        
    #     self.print_array_as_hex(data)
        
    #     self.send_data(data)
        
    #     return


    # def restore_defaults(self, ID):
    #     datalength = 1
    #     data = self.create_message(datalength)
        
    #     data[0] = 0x55
    #     data[1] = 0x04  # command = 4  
    #     data[2] = datalength
    #     data[3] = ID
        
    #     crc = self.generate_modbus_crc(data[:-2])   # skip empty CRC bytes
        
    #     print("Modbus CRC: 0x", format(crc, '04X'))
        
    #     data[datalength + 3] = (crc & 0xff);
    #     data[datalength + 4] = ((crc >> 8)& 0xff);
        
    #     self.print_array_as_hex(data)
        
    #     self.send_data(data)
        
    #     return

    def cmd_GetVersion(self):
        datalength = 0
        data = self.create_message(datalength)
        
        data[0] = 0x55
        data[1] = 0x40  
        data[2] = datalength
        
        crc = self.generate_modbus_crc(data[:-2])   # skip empty CRC bytes
        
        data[datalength + 3] = (crc & 0xff);
        data[datalength + 4] = ((crc >> 8)& 0xff);
        
        self.print_array_as_hex(data)
        
        self.send_data(data)
        
        return
    
    def cmd_Generic(self, cmd : int, datalength, payload : np.array):
        data = self.create_message(datalength)
        
        data[0] = 0x55
        data[1] = (cmd & 0xff) 
        data[2] = (datalength & 0xff)
        
        for i in range(datalength):
            data[3 + i] = (payload[i] & 0xff)
        
        crc = self.generate_modbus_crc(data[:-2])   # skip empty CRC bytes
        
        # print("> Modbus CRC: 0x", format(crc, '04X'))
        
        data[datalength + 3] = (crc & 0xff);
        data[datalength + 4] = ((crc >> 8)& 0xff);
        
        self.print_array_as_hex(data)
        
        self.send_data(data)
        
        return
