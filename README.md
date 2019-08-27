# SmartWatch_withRTOS
In this project I've designed a RTOS to take care of the various tasks of a smart watch

## Specifications
- Interactive GUI
- Step Count
- Temperature Sensor
- BLE support
- File System
- Real time task managment

## GUI
For display I've used a 128x128 SSI LCD. The menu can be traversed using a joystick and two buttons.

## Step counting
For counting steps I've used a 3-axis accelerometer and applied an algorithm to detect when the sensor reads reading greater the a threshold the count a step.

## Temperature sensor
The temperature sensor I've used communicates over I2C.

## BLE support
For BLE I've used CC2650 launchpad as my network processor, I've not developed the mobile application so currently we can see the data sent by the watch in any BLE application like LightBlue or BLE Scanner. 

## RTOS
I have used a priority scheduler to schedule the diffrent tasks like step calculation, Temperature sensing, Display the visuals, etc.

## File System
Currently I've only one sector to store the step count so that total number of steps can be retained even after the power is cut down.

## Project structure
src folder contains all the source files and inc contains all the header files.
- main.c: Application Layer
- BSP.c: Board support package for MK-2 booster pack from TI
- OS.c: RTOS with only necessary and sufficient required for this project.
- GPIO.c: GPIO devvice driver
- UART0.c: UART0 device driver
- UART1.c: UART1 device driver
- eDisk.c: Memory managment driver
- eFile.c: File managment driver


