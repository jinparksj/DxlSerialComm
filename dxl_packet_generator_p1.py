__author__ = "Sungjin Park"
__email__ = "jinparksj@gmail.com"

"""

Packet generator for Protocol 1.0


"""

import time
import threading
import serial
from dxl_addr_table_p1 import *
import math

lock = threading.lock()

def checksum_generator(motor_id, length, instruction, param_n, byte_size):
    """
    Instruction Checksum: Check whether packet is broken during transmission
        Instruction Checksum = ~( ID + Length + Instruction + Parameter1 + Parameter N)
        ( ~: Not Bit Operator )
    """
    return 255 - ((motor_id + length + instruction + param_n + sum(byte_size, )) % 256)

def packet_generator(motor_id, length, instruction, param_n, byte_size, checksum, *args):
    """
        - Instruction Packet Structure
        Header 1    Header 2    ID  Length  Instruction     Param1  ...     ParamN  Checksum
        0xFF        0xFF        ID  Length  Instruction     Param1  ...     ParamN  CHKSUM
    """

    if param_n != 0 and byte_size != 0:
        packet = [0xFF, 0xFF, motor_id, length, instruction, param_n, sum(byte_size), checksum]
    elif motor_id == DXL_BROADCAST_ID:
        packet = [0xFF, 0xFF, motor_id, length, instruction, param_n, byte_size]
        packet.extend(args)
        packet.append(checksum)
    else:
        packet = [0xFF, 0xFF, motor_id, length, instruction, checksum]

    raw_packet = bytearray(packet)
    return raw_packet

class DXLPacketGenP1(object):
    def __init__(self, port, baudrate):
        TIMEOUT = 0.004

        #Setup serial node
        #Protocol: 8 bit, 1 stop bit, none parity, asynchronous serial communication
        self.ser = serial.Serial(
            port,
            baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        )

    def __del__(self):
        self.close()


    def close(self):
        """
        Close the serial port once done
        """
        if self.ser:
            self.ser.reset_input_buffer() # Flush input buffer, discarding all its contents.
            self.ser.reset_output_buffer() # Clear output buffer, aborting the current output
            # and discarding all that is in the buffer.
            self.ser.close()

    def __write_packet(self, packet):
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        self.ser.flush() # Flush of file like objects. In this case, wait until all data is written.
        self.ser.write(packet)

    def __read_packet(self):
        status_packet = []
        status_packet.extend(self.ser.read(4)) # Number of bytes to read: 4, Read size bytes from the serial port.
        if status_packet:
            status_packet.extend(self.ser.read(status_packet[3]))
            status_packet = [idx for idx in status_packet]

        return status_packet

    def ping(self, motor_id):
        """
        This command does not instruct anything.It is only used when receiving Status Packet
        or confirming the existence of Dynamixels with a specific ID.

        Protocol 1.0 - Instuction 1.
        Ping            0x01    No action and controller gets Status Packet         0

        Length: 0x02 or 2
        Instrunction: 0x01 or 1

        :param motor_id:
        :return:
        """
        # 1. Instrunction setting
        instruction = DXL_PING
        # 2. Length
        length = 2
        # 3. Checksum Generation
        checksum = checksum_generator(motor_id, length, instruction, 0, (0, ))
        # 4. Packet Generation
        packet = packet_generator(motor_id, length, instruction, 0, 0, checksum)
        # 5. Write packet
        self.__write_packet(packet)
        # 6. Read status
        status = self.__read_packet()

        return status

    def read_data(self, motor_id, paramN, paramLen):
        """
        This command is to read data in the Control Table
        Protocol 1.0 - Instuction 2.
        Read            0x02    Read Dynamixel data     2

            - Instruction Packet Structure
        Header 1    Header 2    ID  Length  Instruction     Param1  ...     ParamN  Checksum
        0xFF        0xFF        ID  Length  Instruction     Param1  ...     ParamN  CHKSUM

        Length: 0x04 or 4
        Instrunction: 0x02 or 2

        Param1: Start address of data to be read
        Param2: Length of data to be read

        """
        #1. Instrunction setting
        instruction = DXL_READ_DATA
        #2. Length
        length = 4 #Number of parameters + 2
        #3. Checksum Generation
        checksum = checksum_generator(motor_id, length, instruction, paramN, (paramLen, ))
        #4. Packet Generation
        packet = packet_generator(motor_id, length, instruction, paramN, (paramLen, ), checksum)
        #5. Write packet
        self.__write_packet(packet)
        #6. Read status
        status = self.__read_packet()

        return status

    def write_data(self, motor_id, paramN, paramData):
        """

        This command is to write data to the Control table
        Protocol 1.0 - Instuction 3.
        Write           0x03    Write Dynamixel data                                > 2

        Length: N + 3 (prameters N)
        Instruction: 0x03


        :param motor_id: Start address of data to be read
        :param paramN: First data to write, Nth data to write
        :param paramData: parameter Data to write
        :return:
        """

        # 1. Instrunction setting
        instruction = DXL_WRITE_DATA
        # 2. Length
        length = len(paramData) + 3  # Number of parameters + 2
        # 3. Checksum Generation, bit_length should be integer and (bit_length, ) is made for being iterable
        checksum = checksum_generator(motor_id, length, instruction, paramN, paramData)
        # 4. Packet Generation
        packet = packet_generator(motor_id, length, instruction, paramN, paramData, checksum)
        # 5. Write packet
        self.__write_packet(packet)
        # 6. Read status
        status = self.__read_packet()

        return status

    def reg_write(self, motor_id, paramN, paramData):
        """
        Similar to write_data in terms of function but differs in the timing.
        Protocol 1.0 - Instuction 4.

        Reg Write       0x04    Similar to WRTE_DATA, Waiting and acting            > 2
                                    when ACTION command received

        Length: N + 3 (writing parameters N)
        Instruction: 0x04

        :param motor_id:
        :param param_N:
        :param data:
        :return:
        """
        # 1. Instrunction setting
        instruction = DXL_REG_WRITE
        # 2. Length
        length = len(paramData) + 3  # Number of parameters + 3
        # 3. Checksum Generation, bit_length should be integer and (bit_length, ) is made for being iterable
        checksum = checksum_generator(motor_id, length, instruction, paramN, paramData)
        # 4. Packet Generation
        packet = packet_generator(motor_id, length, instruction, paramN, paramData, checksum)
        # 5. Write packet
        self.__write_packet(packet)
        # 6. Read status
        status = self.__read_packet()

        return status

    def action(self, motor_id):
        """
        This command is to execute the Write action registered by REG_WRITE
        Protocol 1.0 - Instuction 5.

        Action          0x05    Command to act the ACTION registered in REG WRITE   0

        Length: 0x02
        Instruction: 0x05

        :param motor_id:
        :return:
        """
        # 1. Instrunction setting
        instruction = DXL_ACTION
        # 2. Length
        length = 2
        # 3. Checksum Generation
        checksum = checksum_generator(motor_id, length, instruction, 0, (0, ))
        # 4. Packet Generation
        packet = packet_generator(motor_id, length, instruction, 0, 0, checksum)
        # 5. Write packet
        self.__write_packet(packet)
        # 6. Read status
        status = self.__read_packet()

        return status

    def factory_reset(self, motor_id):
        """
        Factory Reset
        Protocol 1.0 - Instuction 6.
        Factory Reset   0x06    Factory Reset        0

        Length: 0x02
        Instruction: 0x06

        :param motor_id:
        :return:
        """

        # 1. Instrunction setting
        instruction = DXL_RESET
        # 2. Length
        length = 2
        # 3. Checksum Generation
        checksum = checksum_generator(motor_id, length, instruction, 0, (0, ))
        # 4. Packet Generation
        packet = packet_generator(motor_id, length, instruction, 0, 0, checksum)
        # 5. Write packet
        self.__write_packet(packet)
        # 6. Read status
        status = self.__read_packet()

        return status


    def reboot(self, motor_id):
        """
        Reboot dynamixel

        Protocol 1.0 - Instuction 7.
        Factory Reset   0x06    Factory Reset        0

        Length: 0x02
        Instruction: 0x08

        :param motor_id:
        :return:
        """

        # 1. Instrunction setting
        instruction = DXL_REBOOT
        # 2. Length
        length = 2
        # 3. Checksum Generation
        checksum = checksum_generator(motor_id, length, instruction, 0, (0, ))
        # 4. Packet Generation
        packet = packet_generator(motor_id, length, instruction, 0, 0, checksum)
        # 5. Write packet
        self.__write_packet(packet)
        # 6. Read status
        status = self.__read_packet()

        return status

    def sync_write(self, control_address, total_data):
        """
        Control multiple Dynamixels simultaneousely using one instruction packet transmission.

        Can be used only if both of the address and length of the Control Talble to write is identical.
        Besides, ID should be transmitted as Broadcasting ID (DXL_BROADCAST_ID).

        Protocol 1.0 - Instuction 8.
        Sync Write      0x83    Simultaneously, control several dynamixels          > 4

        Instruction: 0x83
        Length: ((L+1) * N) + 4 (L: Data length, N: No. of Dynamixels)
        4 = Instruction + Length + Control Address + Data Length

        Param1: Control Address, ex) Goal Position
        Param2: Data Length, L
        Param3: First Dynamixel - ID
        Param4: First Dynamixel - First data byte
        Param5: First Dynamixel - Second data byte
        ...     ...
        ParamL+3: First Dynamixel - Lth data byte
        ParamL+4: Second Dynamixel - ID
        ParamL+5: Second Dynamixel - First data byte
        ...     ...
        Param2L+4: Second Dynamixel - Lth data byte

        IMPORTANT: In the data byte, there are High and Low
        EX) 0x150 -> 0x50(Low)   0x01(High)
        Length of data will be longer, if the data parameter is bigger, dividing High and Low

        control_address: The starting address of Control, such as Goal Position, that data will be written from
                        the dxl_addr_table_p1.py
        total_data: Tuple of values containing the following
                ((motor_id_1, data), (motor_id_2, data))
                Note that 'data' is predefined in different function
                (Tuple > List in speed)

        """

        # 1. Instrunction setting
        instruction = DXL_SYNC_WRITE
        # 2. Length
        param_data = [int(round(param)) for motors in total_data for param in motors]
        """
        for motors in total_data:
            for param in motors:
                list.append(param)
        """
        length = 4 + len(param_data) #param_data has ID and Data bytes
        # 3. Checksum Generation, bit_length should be integer and (bit_length, ) is made for being iterable
        len_param_data = len(total_data[0][1:])
        byte_size = (len_param_data + sum(param_data), )
        # Should make the size as tuple (byte_size, ) for checksum fxn
        # total_data[0][1:] means for param_data length per Dynamixel
        checksum = checksum_generator(DXL_BROADCAST_ID, length, instruction, control_address, byte_size)
        # 4. Packet Generation
        packet = packet_generator(DXL_BROADCAST_ID, length, instruction, control_address, len_param_data, checksum, param_data)
        # 5. Write packet
        self.__write_packet(packet)
        # 6. Read status -> Packet function using BROADCAST_ID has no status packet
        # that's why here is no return

    def bulk_read(self, read_address, address_length, motor_list):
        """

        Protocol 1.0 - Instuction 8.
        Bulk Read       0x92    Simultaneously, read data of dynamixels(only for MX)> 4

        :param read_address:
        :param address_length:
        :param motor_list:
        :return:
        """
























