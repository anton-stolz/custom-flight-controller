# Custom Flight Controller
![IMG_20251206_011033_922~2](https://github.com/user-attachments/assets/61efc68e-6de6-4825-99dd-7efb872fe9f3)



https://github.com/user-attachments/assets/e3cfa4be-3901-4735-b42a-417750daa059

## Hardware
- The core is an STM32F4 Microchip
- For the IMU a simple MPU-6500 is connected via I2C
- For radio control, a 6 Channel FlySky receiver is used, and each individual PWM channel is connected to the stm32
- The motors are controlled via the 'jhemcu ruibet 45A ESC 4 in 1' Motor controller, with the DShot Protocol
- 3s 2200mAh Lipo Battery
- Brushless Motors xa2212 1400kv 3-4s 
- 6030 Propellers.

- Frame build with light wood, glued into T-Beams

## Software
Very Simple PID controller, the code is mostly glue code.\
The control loop runs at 500Hz.\
The IMU Data is converted to a 3d Rotation using the Mahony AHRS Library, which does simple Sensor Fusion.\
The current angle which was calculated from the IMU data is compared with the goal angle which is read from the radio receiver, and then PID calculates the Motor speeds. 
Those motor speeds are send to the ESC via the Dshot Protokoll.

## Future Plans
add a barometer, so the Quadcopter can easily stay at a constant height.
add a gps (ideally with dgnss) and make the Quadcopter autonomos, maybe with a RaspberryPI and ROS.

## License
This code / the whole bundle is GPL 3.0, because of the Mahony AHRS version I used.

- MIT Dshot implementation by mokhwasomssi https://github.com/mokhwasomssi/stm32_hal_dshot
- GNU GPL Mahony AHRS by Madgwick https://github.com/PaulStoffregen/MahonyAHRS/blob/master/src/MahonyAHRS.cpp
- Modified, but strongly inspired by https://github.com/bolderflight/invensense-imu/blob/main/src/mpu6500.cpp

## LLM Use
The Translation of the mpu6500 library from arduino cpp to stm32 and c was done with the help of ai, as well as the wiring of mahony ahrs library and the formula for converting from quaternion to euler angle. 





