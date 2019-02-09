__author__ = "Sungjin Park"
__email__ = "jinparksj@gmail.com"


"""
PROTOCOL 1.0

* http://emanual.robotis.com/docs/kr/dxl/protocol1/

1. Overview
    - Dynamixel motors operate with binary type of data.
    Main controller <------ Packet ------> Dynamixel motors
    There are two types of packet.
        1. Instruction packet: control the dynamixel motors
        2. Status packet: response from dynamixel motors to main controller

    - ID: Dynamixel identified number
    - Protocol: 8 bit, 1 stop bit, none parity, asynchronous serial communication
    - Half duplex: Single communication line that TxD and RxD share.
    Half duplex is used for connecting between several communication devices.
    When some devices transmit, other devices should be at the status of receiving.
    Therefore, it requires to have direction port to control the communication direction.

    <Direction Output Duration>
    ( Instruction Packet ) ---------------- ( Status Packet )
                        ->return delay time<-

    - Tx, Rx direction: BITs in REGISTER for expressing UART_STATUS
        1. TXD_BUFFER_READY_BIT: Status for storing Transmission DATA at Buffer, which means that SERIAL TX BUFFER is empty
        2. TXD_SHIFT_REGISTER_EMPTY_BIT: SET, when all Transmission Data is transmitted out from CPU

    - Byte to Byte Time: Delay time between byte and byte during transmitting instruction packet
    Transmission interrupt: > 100msec -> So, wait for packet header (0xff 0xff)

2. Instruction Packet: Command data from controller to Dynamixel motors
    - Instruction Packet Structure
        Header 1    Header 2    ID  Length  Instruction     Param1  ...     ParamN  Checksum
        0xFF        0xFF        ID  Length  Instruction     Param1  ...     ParamN  CHKSUM

    - 1. Header: Signal for beginning of Packet

    - 2. Packet ID: Dynamixel ID taking Instruction Packet
        * Normal ID: 0 ~ 253 (0x00 ~ 0xFD), total number 254
        * Broadcast ID: 254 (0xFE), All Dynamixel

    - 3. Length: Packet length, Data length of Instruction + Parameter + Checksum
        * Length = Parameter numbers (N) + 2

    - 4. Instruction: Command for controlling Dynamixels
            Command         Value   Description                                         No. of Parameters
        1.  Ping            0x01    No action and controller gets Status Packet         0
        2.  Read            0x02    Read Dynamixel data                                 2
        3.  Write           0x03    Write Dynamixel data                                > 2
        4.  Reg Write       0x04    Similar to WRTE_DATA, Waiting and acting            > 2
                                    when ACTION command received
        5.  Action          0x05    Command to act the ACTION registered in REG WRITE   0
        6.  Factory Reset   0x06    Factory Reset                                       0
        7.  Reboot          0x08    Rebooting instruction                               0
        8.  Sync Write      0x83    Simultaneously, control several dynamixels          > 4
        9.  Bulk Read       0x92    Simultaneously, read data of dynamixels(only for MX)> 4

    - 5. Parameters: If INSTRUCTION needs additional parameters, the INSTRUCTION use this one. (No. of Parameters)
    - 6. Instruction Checksum: Check whether packet is broken during transmission
        Instruction Checksum = ~( ID + Length + Instruction + Parameter1 + Parameter N)
        ( ~: Not Bit Operator )
        * If the result of ( ... ) is larger than 255 (0xFF), the checksum only use lower byte
        EXAMPLE:
            ID=1(0x01), Length=5(0x05), Instruction=3(0x03), Parameter1=12(0x0C), Parameter2=100(0x64), Parameter3=170(0xAA)
            Checksum=~(ID+Length+Instruction+Param1+Param2+Param3)=~(0x01+0x05+0x03+0x0C+0x64+0xAA)=~(0x123)
            Lower byte is 0x23 -> Not Operator -> 0xDC
            Therefore, Instruction Packet = 0x01 0x05 0x03 0x0C 0x64 0xAA 0xDC

3. Status Packet (Return Packet) : Dynamixel returns the result to Main Controller. The returned data is Status Packet.
    - Status Packet Structure
        Header 1    Header 2    ID  Length  Error   Param1  ...     ParamN  Checksum
        0xFF        0xFF        ID  Length  Error   Param1  ...     ParamN  CHKSUM

    - 3. Length: Length = Parameter Number (N) + 2

    - 4. Instruction: Command for controlling Dynamixels
            Command             Value   Description
        1.  0                   Bit7    -
        2.  Instruction Error   Bit6    Set 1, When transmitted undefined instruction
                                        / when transmitted ACTION command without REF_WRITE
        3.  Overload Error      Bit5    Set 1, When cannot control current load with current torque
        4.  Checksum Error      Bit4    Set 1, when checksum is not right in transmitted instruction packet
        5.  Range Error         Bit3    Set 1, when over the limit of range
        6.  Overheating Error   Bit2    Set 1, when internal temperature of Dynamixel is over the limit of range
        7.  Angle Limit Error   Bit1    Set 1, when Writing the range of goal position over the limit of CW and CCW Angle
        8.  Input Voltage Error Bit0    Set 1, when operating voltage is over the limit which set in Control table

    - EXAMPLE : 0xFF 0xFF 0x01 0x02 0x24 0xD8
    Dynamixel ID 01, 0x24 - (binary) -> 00100100 -> bit 5 & bit 2 -> Overload and overheating error

"""

#========================================#
#                 EEPROM                 #
#========================================#

MODEL_NUMBER_L = 0      #Lowest byte of model number
MODEL_NUMBER_H = 1      #Highest byte of model number
FIRMWARE_VERSION = 2
MOTOR_ID = 3
BAUD_RATE = 4
RETURN_DELAY_TIME = 0x05
CW_ANGLE_LIMIT_L = 0x06
CW_ANGLE_LIMIT_H = 0x07
CCW_ANGLE_LIMIT_L = 0x08
CCW_ANGLE_LIMIT_H = 0x09
DRIVE_MODE = 0x0A
HIGH_LIMIT_TEMP = 0x0B
LOW_LIMIT_VOLTAGE = 0x0C
HIGH_LIMIT_VOLTAGE = 0x0D
MAX_TORQUE_L = 0x0E
MAX_TORQUE_H = 0x0F
STATUS_RETURN_LEVEL = 0x10
ALARM_LED = 0x11
ALARM_SHUTDOWN = 0x12
MULTI_TURN_OFFSET_L = 0x14
MULTI_TURN_OFFSET_H = 0x15
RESOLUTION_DIVIDER = 0x16















#========================================#
#                   RAM                  #
#========================================#


