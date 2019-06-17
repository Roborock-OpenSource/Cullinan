# Cullinan
## Preface
A Lidar based on Roborock LDS (Laser Distance Sensor) for STEM.

Since Roborock launched it's first robot vacuum there were many colledge students and robot fans from all world asking for more information about the LDS, and some engineers bought our robot vacuum and then disassembled it just for the LDS because even the price of the whole robot is much cheaper than the other LiDARs.

In fact the original LDS is not very friendly for fans becasue of the limited IO and you have to make a PWM singal to control the DC motor's speed by yourself. So we are going to develop a STEM friendly version for a very low price or even maybe free for the students and research purpose.

So, be free to tell us what do you think about that and what feature do you expect for the LDS?
![image](https://github.com/Roborock-OpenSource/Cullinan/blob/master/Gif/pic.gif?raw=true)
![image](https://github.com/Roborock-OpenSource/Cullinan/blob/master/Gif/3D.gif?raw=true)
## Feature
* 360 degree 2D laser scanner
* 150mm~6000mm detection distance
* 1.8kHz sampling rate
* 3-axis gyroscope and 3-axis accelerometer motion data output
![image](https://github.com/Roborock-OpenSource/Cullinan/blob/master/Gif/demo.gif?raw=true)
## Introduction
Cullinan is self-developed by Roborock with independent intellectual property rights. The system consists of two parts: the LDS ranging system and the IMU inertial measurement unit. The LDS ranging system can scan and measure the environment within the range of 360° 15cm~6m. 360 samples for one round, each sample corresponds to about 1°. The scanning frequency is 5Hz. The IMU inertial measurement unit enables real-time acquisition of motion data such as acceleration and angular velocity. The measurement system interacts with external applications through the serial port.
## LDS distance measurement system
### Specification

Parameter|Value|Unit
---------|-----|----
Distance Rang (D)|0.15~6|Meter (m)
Angular Range (A)|0~360|Degree
Angular Resolution (Ar)|1|Degree
Sample Duration|0.5|Millisecond (ms)
Sample Frequency|1800|Hz
Scan Rate|5|Hz

### LDS serial data
#### Serial communication format
LDS measurement system sends distance data through serial port. The serial format is set to 115200 baud rate, 8 data bits, 1 stop bit, no parity bit, no flow control.
#### Serial data format
There are two kinds of data from LDS. One is property information, the other is distance information. The property information is sent only before rotation, and the interval is 500 ms. When LDS starts to rotate, it stops to send property information and starts to send distance sample.
#### Property data format

Field Name|Size / byte|Description
----------|-----------|-----------
LDS_INFO_START|1|synchronization character, fixed 0xAA. If the property packet contains synchronization character (0xAA), it must be escaped before sending. 0xAA is replaced by 0xA901, and 0xA9 is replaced by 0xA900. **Note**: 1. MSB is sent first in escape sequence. 2. When sending, calculate the checksum first, then perform the escaping operation; When receiving, perform the escaping operation first, then calculate the checksum.
PACKET_SIZE|2|paceket size in byte except for LDS_INFO_START, set to 84
SOFTWARE_VERSION|2|firmware version, bit[15:12] = major versio, bit[11:0] = minor version. Example: 0x1001 is interpreted as V1.1
UID_SIZE|2|UID size, set to 18
UID|18|UID value
RSVD|58|Reserved
CHECK_SUM|2|packet checksum, see [The checksum algorithm for property packet](#The-checksum-algorithm-for-property-packet)

##### The checksum algorithm for property packet
```` c
uint16_t checksum = 0;
 for(i = 0; i < size; i++){ // size is 41
  checksum += *(lds_info_data + i);  // lds_info_data is declared as uint16_t* pointer and points to property packet buffer.
 }
````
#### Measurement data format
**Note**: Measurement packet does not contain escape character.

Field Name|Size / byte|Description
----|----|----
Start|1|synchronization character, fixed 0xFA
Index|1|sample index, from 0xA0 to 0xF9. There are 4 samples in a measurement packet, and 90 packets for one round.
Speed|2|little endian, real speed value * 64, in rpm
Data 0|4|sample data 0, see [Sample data format](#Sample-data-format)
Data 1|4|sample data 1, see [Sample data format](#Sample-data-format)
Data 2|4|sample data 2, see [Sample data format](#Sample-data-format)
Data 3|4|sample data 3, see [Sample data format](#Sample-data-format)
Checksum|2|packe checksum, see [The checksum algorithm for measurement packet](#The-checksum-algorithm-for-measurement-packet)
##### Sample data format
Offset|Description
----|----
0|distance[7:0]
1|bit7 = data invalid flag, bit6 = strength warning flag, bit[5:0] = distance[13:8]
2|strength[7:0]
3|strength[15:8]
##### The checksum algorithm for measurement packet
```` c
uint32_t i, checksum = 0;

for (i = 0; i < 20; i += 2)
{
    checksum = (checksum << 1) + *(uint16_t *)&lds_buf[i];
}
checksum = (checksum + (checksum >> 15)) & 0x7FFF;
````
## Inertial measurement Unit (IMU)
### Introduction
IMU is based on Bosch BMI160 6-axis MEMS sensor which integrates 16-bit 3-axis accelerometer with ultra-low-power 3-axis gyroscope. The BMI160 is available in a compact 14-pin 2.5 × 3.0 × 0.83 mm<sup>3</sup> LGA package. When accelerometer and gyroscope are in full operation mode, power consumption is typically 925 μA, enabling always-on applications in battery driven devices.
## BMI160 API
### Register operation API
#### BMI160_ReadReg
Prototype:````int32_t BMI160_ReadReg(uint8_t RegAddr, uint8_t *RegData)````  
Description: Read a register  
Parameter:````RegAddr````- register address  
&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;````RegData````- register data pointer   
Return:````STATE_OK````- state OK  
&emsp;&emsp;&emsp;&nbsp;````STATE_PENDING````- state pending  
#### BMI160_ReadRegSeq
Prototype:````int32_t BMI160_ReadRegSeq(uint8_t RegAddr, uint8_t *RegData, uint8_t len)````  
Description: Read multiple registers  
Parameter:````RegAddr````- register start addrss  
&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;````RegData````- register data pointer  
&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;````len````- the number of register to be read  
Return:````STATE_OK````- state OK  
&emsp;&emsp;&emsp;&nbsp;````STATE_PENDING````- state pending  
#### BMI160_WriteReg
Prototype:````int32_t BMI160_WriteReg(uint8_t RegAddr, uint8_t RegData)````  
Description: Write a register  
Parameter:````RegAddr````- register address  
&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;````RegData````- register data  
Return:````STATE_OK````- state OK  
&emsp;&emsp;&emsp;&nbsp;````STATE_PENDING````- state pending  
#### BMI160_WriteRegSeq
Prototype:````int32_t BMI160_WriteRegSeq(uint8_t RegAddr, uint8_t *RegData, uint8_t len)````  
Description: Write multiple registers  
Parameter:````RegAddr````- register start addrss  
&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;````RegData````- register data pointer  
&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;````len````- the number of register to be written  
Return:````STATE_OK````- state OK  
&emsp;&emsp;&emsp;&nbsp;````STATE_PENDING````- state pending  
### BMI160 API
#### BMI160_SetAccelNormal
Prototype:````int32_t BMI160_SetAccelNormal(void)````  
Description: Set accelerometer sensor in normal power mode  
Parameter: None  
Return:````STATE_OK````- state OK  
&emsp;&emsp;&emsp;&nbsp;````STATE_PENDING````- state pending  
#### BMI160_SetGyroNormal
Prototype:````int32_t BMI160_SetGyroNormal(void)````  
Description: Set gyroscope sensor in normal power mode  
Parameter: None  
Return:````STATE_OK````- state OK  
&emsp;&emsp;&emsp;&nbsp;````STATE_PENDING````- state pending  
### BMI160 API example
```` c
    /* Select the Output data rate, range of accelerometer sensor */
    BMI160_REG.ACC_CONF.acc_odr = BMI160_ACCEL_ODR_100HZ;
    BMI160_REG.ACC_CONF.acc_bwp = BMI160_ACCEL_BW_NORMAL_AVG4;
    BMI160_REG.ACC_CONF.acc_us = 0;
    BMI160_REG.ACC_RANGE.acc_range = BMI160_ACCEL_RANGE_2G;
    while (BMI160_WriteRegSeq(REG(ACC_CONF), (uint8_t *)&BMI160_REG.ACC_CONF, 2) == STATE_PENDING) ;
    while (BMI160_SetAccelNormal() == STATE_PENDING) ; // set normal power mode
    
    /* Select the Output data rate, range of Gyroscope sensor */
    BMI160_REG.GYR_CONF.gyr_odr = BMI160_GYRO_ODR_100HZ;
    BMI160_REG.GYR_CONF.gyr_bwp = BMI160_GYRO_BW_NORMAL_MODE;
    BMI160_REG.GYR_RANGE.gyr_range = BMI160_GYRO_RANGE_2000_DPS;
    while (BMI160_WriteRegSeq(REG(GYR_CONF), (uint8_t *)&BMI160_REG.GYR_CONF, 2) == STATE_PENDING) ;
    while (BMI160_SetGyroNormal() == STATE_PENDING) ; // set normal power mode
````
### Inertial measurement interface
#### Serial communication format
Inertial measurement unit sends raw data to external application through an USB VCP. The serial format is not important.
#### Data format

Offset|Name|Size / byte|Description
------|----|-----------|----
0|GYR_X[7:0]|1|Low byte of gyroscope x-axis data
1|GYR_X[15:8]|1|High byte of gyroscope x-axis data
2|GYR_Y[7:0]|1|Low byte of gyroscope y-axis data
3|GYR_Y[15:8]|1|High byte of gyroscope y-axis data
4|GYR_Z[7:0]|1|Low byte of gyroscope z-axis data
5|GYR_Z[15:8]|1|High byte of gyroscope z-axis data
6|ACC_X[7:0]|1|Low byte of accelerometer x-axis data
7|ACC_X[15:8]|1|High byte of accelerometer x-axis data
8|ACC_Y[7:0]|1|Low byte of accelerometer y-axis data
9|ACC_Y[15:8]|1|High byte of accelerometer y-axis data
10|ACC_Z[7:0]|1|Low byte of accelerometer z-axis data
11|ACC_Z[15:8]|1|High byte of accelerometer z-axis data
12|SENSORTIME[7:0]|1|Low byte of sensor time stamp
13|SENSORTIME[15:8]|1|Median byte of sensor time stamp
14|SENSORTIME[23:16]|1|High byte of sensor time stamp

## Debug interface
### Serial communication format
The user can enter some debug commands through serial terminal. The serial format is set to 115200 baud rate, 8 data bits, 1 stop bit, no parity bit, no flow control.
### BMI160 operation command
#### rd160
Format:  
````rd160 reg_addr````  
Description: Read a register from address````reg_addr````and print the value.  
Example:
````
rd160 40
40=28
````
#### wr160
Format:  
````wr160 reg_addr reg_data````  
Description: Write register value ````reg_data```` to register address````reg_addr````and read back.  
Example:
````
wr160 40 29
read back 40=29
````
#### dump160
Format:  
````dump160````  
Description: Dump all 128 registers  
Example:
````
dump160
     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
[00]:D1 21 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[10]:00 00 00 00 00 00 00 00 6F 77 2C 10 00 00 00 00
[20]:00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[30]:00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[40]:29 03 28 00 0B 88 80 10 00 00 00 20 80 42 4C 00
[50]:00 00 00 00 00 00 00 00 00 00 07 30 81 0B C0 00
[60]:14 14 24 04 0A 18 48 08 11 00 00 00 00 00 00 00
[70]:00 00 00 00 51 96 09 00 00 00 15 03 00 00 00 00
````
#### set160
Format:  
````set160````  
Description: Configure sensor as follow table

Parameter | Value
---- |-----
active high level for INT1 pin|1 (active high level for INT1 pin)
I/O output mode for INT1 pin|0 (push-pull)
Output enable for INT1 pin|1 (output enabled)
Data ready interrupt mapped to pin INT1|1 (enabled)
accelerometer ODR|BMI160_ACCEL_ODR_100HZ
accelerometer bandwidth|BMI160_ACCEL_BW_NORMAL_AVG4
accelerometer undersampling|0 (disabled)
accelerometer g-range|BMI160_ACCEL_RANGE_2G
accelerometer power mode|BMI160_ACCEL_NORMAL_MODE
gyroscope ODR|BMI160_GYRO_ODR_100HZ
gyroscope bandwidth|BMI160_GYRO_BW_NORMAL_MODE
gyroscope angular rate range|BMI160_GYRO_RANGE_2000_DPS
gyroscope power mode|BMI160_GYRO_NORMAL_MODE

Example:  
````set160````
#### data160
Format:  
````data160````  
Description: Poll bit````drdy_acc````in register````STATUS````in endless, if set read 3-axis data from accelerometer, 3-axis data from gyroscope and time stamp, and print all data in a line.  
Example:
````
data160
ACC.x= -0.056152 ACC.y=  0.035828 ACC.z=  1.008545 GYR.x= -0.121951 GYR.y= -0.426829 GYR.z= -0.182927 @  7277825
ACC.x= -0.055176 ACC.y=  0.036072 ACC.z=  1.007935 GYR.x= -0.182927 GYR.y= -0.426829 GYR.z= -0.182927 @  7278081
ACC.x= -0.056335 ACC.y=  0.032349 ACC.z=  1.007690 GYR.x= -0.121951 GYR.y= -0.426829 GYR.z= -0.182927 @  7278337
......
````
#### temp160
Format:  
````temp160````  
Description: Read temperature data in endless loop and print in degree Celsius (°C).  
Example:
````
temp160
T= 33.335938
T= 33.314453
T= 33.373047
......
````
## Copyright © Beijing Roborock Technology Co., Ltd.
