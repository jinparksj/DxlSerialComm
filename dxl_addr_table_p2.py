__author__ = "Sungjin Park"
__email__ = "jinparksj@gmail.com"


"""
PROTOCOL 2.0

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
        Header 1    Header 2    Header3     Reserved    ID  Length1 Length2  Instruction     Param1  ...     ParamN  CRC1   CRC2
        0xFF        0xFF        0xFD        0x00        ID  Length1 Length2  Instruction     Param1  ...     ParamN  CRC_L  CRC_H

    - 1. Header: Signal for beginning of Packet

    - 2. Reserved: 0x00 (0xFD cannot be used)

    - 3. Packet ID: Dynamixel ID taking Instruction Packet
        * Normal ID: 0 ~ 252 (0x00 ~ 0xFC), total number 253
        * Broadcast ID: 254 (0xFE), Allows all Dynamixel motors to operate Instruction Packet

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

#=======================================================#
#                       EEPROM                          #
#=======================================================#

DXL_MODEL_NUMBER_L = 0          #Lowest byte of model number
DXL_MODEL_NUMBER_H = 1          #Highest byte of model number
DXL_FIRMWARE_VERSION = 2        #Information on the version of firmware
DXL_MOTOR_ID = 3                #ID of Dynamixel
DXL_BAUD_RATE = 4               #Baud Rate of Dynamixel
DXL_RETURN_DELAY_TIME = 5       #Return Delay Time
DXL_CW_ANGLE_LIMIT_L = 6        #Lowest byte of clockwise Angle Limit
DXL_CW_ANGLE_LIMIT_H = 7        #Highest byte of clockwise Angle Limit
DXL_CCW_ANGLE_LIMIT_L = 8       #Lowest byte of counterclockwise Angle Limit
DXL_CCW_ANGLE_LIMIT_H = 9       #Highest byte of counterclockwise Angle Limit
DXL_DRIVE_MODE = 10             #Dual Mode Setting
DXL_HIGH_LIMIT_TEMP = 11        #Internal Limit Temperature
DXL_LOW_LIMIT_VOLTAGE = 12      #Lowest Limit Voltage
DXL_HIGH_LIMIT_VOLTAGE = 13     #Highest Limit Voltage
DXL_MAX_TORQUE_L = 14           #Lowest byte of Max. Torque
DXL_MAX_TORQUE_H = 15           #Highest byte of Max. Torque
DXL_STATUS_RETURN_LEVEL = 16    #Status Return Level
DXL_ALARM_LED = 17              #LED for Alarm
DXL_ALARM_SHUTDOWN = 18         #Shutdown for Alarm
DXL_MULTI_TURN_OFFSET_L = 20    #multi-turn offset least significant byte (LSB)
DXL_MULTI_TURN_OFFSET_H = 21    #multi-turn offset most significant byte (MSB)
DXL_RESOLUTION_DIVIDER = 22     #Resolution divider



#=======================================================#
#                       RAM                             #
#=======================================================#
DXL_TORQUE_ENABLE = 24          #Torque On/Off
DXL_LED = 25                    #LED On/Off
DXL_D_GAIN = 26                 #Derivative Gain
DXL_I_GAIN = 27                 #Integral Gain
DXL_P_GAIN = 28                 #Proportional Gain
DXL_GOAL_POSITION_L = 30        #Lowest byte of Goal Position
DXL_GOAL_POSITION_H = 31        #Highest byte of Goal Position
DXL_MOVING_SPEED_L = 32         #Lowest byte of Moving Speed (Moving Velocity)
DXL_MOVING_SPEED_H = 33         #Highest byte of Moving Speed (Moving Velocity)
DXL_TORQUE_LIMIT_L = 34         #Lowest byte of Torque Limit (Goal Torque)
DXL_TORQUE_LIMIT_H = 35         #Highest byte of Torque Limit (Goal Torque)
DXL_PRESENT_POSITION_L = 36     #Lowest byte of Current Position (Present Velocity)
DXL_PRESENT_POSITION_H = 37     #Highest byte of Current Position (Present Velocity)
DXL_PRESENT_SPEED_L = 38        #Lowest byte of Current Speed
DXL_PRESENT_SPEED_H = 39        #Highest byte of Current Speed
DXL_PRESENT_LOAD_L = 40         #Lowest byte of Current Load
DXL_PRESENT_LOAD_H = 41         #Highest byte of Current Load
DXL_PRESENT_VOLTAGE = 42        #Current Voltage
DXL_PRESENT_TEMPERATURE = 43    #Current Temperature
DXL_REGISTERED = 44             #Means if Instruction is registered
DXL_MOVING = 46                 #Means if there is any movement
DXL_LOCK = 47                   #Locking EEPROM
DXL_PUNCH_L = 48                #Lowest byte of Punch
DXL_PUNCH_H = 49                #Highest byte of Punch
DXL_CURRENT_L = 68              #Lowest byte of Consuming Current
DXL_CURRENT_H = 69              #Highest byte of Consuming Current
DXL_TORQUE_CONTROL_MODE_ENABLE = 70 #Torque control mode on/off
DXL_GOAL_TORQUE_L = 71          #Lowest byte of goal torque value
DXL_GOAL_TORQUE_H = 72          #Highest byte of goal torque value
DXL_GOAL_ACCELERATION = 73      #Goal Acceleration



