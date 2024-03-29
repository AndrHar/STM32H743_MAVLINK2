# STM32H743_MAVLINKV2

Example of implementing mavlink (v2) library.

#### Program functionality:

1. Mavlink communication occurs via UART

2. Receiving and sending HEARTBEAT messages

3. Receiving other messages (ATTITUDE, etc)

4. Request for sending message with determined interval

5. Read value of determined parameter

#### Base actions of this program:

- send CanMsg (0x600) in response to receiving a heartbeat_msg from mavlink (uart2);

- send CanMsg (0x510) in response to receiving a global_position_int_msg from mavlink (uart2);
* send CanMsg (0x520) in response to receiving a attitude_msg from mavlink (uart2);

* send own heartbeat_msg with 1 Hz interval;

* send MAV_Request_MSG (with ATTITUDE id) in response to CanMsg(0x100, byte0=0x11)

 

> There are 30 warnings `[-Waddress-of-packed-member]` during compile.
> 
> Due to documentation of `mavlink v2` it is normal to get this warnings. 

###### 

#### Used test equipment:

1. NUCLEO-H743ZI board from STM with separate external CAN-driver

2. Cube Orange (on Kore Carrier Board) with installed ArduPilot 

###### Connection:

Cube Orange side: 

<u>TELEM1 </u>connector

| Pin | Signal   |
| --- | -------- |
| 2   | Tx (Out) |
| 3   | Rx (In)  |
| 6   | GND      |

NUCLEO side: 

<u>CN9</u> connector

| Pin | Signal    |
| --- | --------- |
| 4   | USART2_RX |
| 6   | USART2_TX |
| 12  | GND       |

<u>CN12 </u>connector (for external CAN-driver)

| Pin | Signal    |
| --- | --------- |
| 12  | FDCAN1_TX |
| 14  | FDCAN1_RX |
