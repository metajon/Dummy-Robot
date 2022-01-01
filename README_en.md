# Dummy-Robot: Super compact smart robotic-arm
> **My super mini robotic arm robot project. **
>
> Video introduction: [[Homemade] I made a robotic arm of the Iron Man! [Hardcore]](https://www.bilibili.com/video/BV12341117rG)
>
> Video: [I made a DUMMY ROBOTIC ARM from scratch! -YouTube](https://www.youtube.com/watch?v=F29vrvUwqS4)

![](5.Docs/1.Images/dummy1.jpg)

### Information description (updated 21-12-24)

* 3D model design source files have been added.
* Added gripper hardware design files and LED light ring PCB
* The PCB file of the wireless spatial positioning controller has been added
* Wireless teach pendant Peak software and hardware project has been added (as a submodule)
* Hardware design files of REF have been added
* DummyStudio host computer has been added
* The firmware source code of the Dummy core controller has been added (see later for instructions)
* 42 stepper motor driver hardware engineering has been added
* 20 stepper motor driver hardware engineering has been added
* The firmware source code for 42/20 stepper motor driver has been added



> This is the complete design scheme of the original robot arm in the video. The cost and production difficulty of this scheme are relatively high. Therefore, students who want to reproduce it are recommended to wait for the **Dummy Youth Edition** that I will release later. This version will have The improvements are as follows:
>
> 1. The structure of the whole machine is redesigned, and 3D printing is used as the manufacturing solution (the original version is aluminum CNC), which greatly reduces the manufacturing cost
> 2. Use my own designed small cycloid reducer to replace the original harmonic reducer, greatly reducing the cost of parts
> 3. All software and firmware are common to the original version, and the functions are exactly the same
> 4. Add my own PC-side host computer and mobile phone APP (try to add the user's initial setting guide)
> 5. Improve the wiring method of the original motor driver. The original power wiring is welded, which is not easy to install and disassemble. The youth version will use 4p connector (power supply + CAN bus) to connect
> 6. Strive to achieve the cost of the whole machine within 2000
> 7. ** Most importantly, someone will find a nanny-level video tutorial! **



## About structural design

The original design of my video uses `stepping motor` + Harmonic `harmonic deceleration module`, of which the latter is more expensive (the second-hand one I bought is about 600 yuan), so in order to allow everyone to reproduce the copy as much as possible For the project, I will add a low-cost solution of `homemade cycloid reducer`+`3D printing` later.

> At present, the cycloid reducer has been designed and is being verified. It is expected to use PC (or acrylic) cutting and 3D printing. The accuracy has decreased but the functions remain unchanged. The hardware cost of the whole machine is hoped to be controlled within 2,000 yuan.

See my other warehouse for the designed cycloid reducer: [peng-zhihui/CycloidAcuratorNano](https://github.com/peng-zhihui/CycloidAcuratorNano)

![](5.Docs/1.Images/cycliod-nano.jpg)

## About the circuit module

In order to realize the main motion control function of the robotic arm, the circuit actually has 4 boards at its core:

* REF core board
* REF base plate is the controller circuit board inside the robot arm base
* Stepper motor drive
* Peak teach pendant

Among them, the first two and Peak are open source, and I refer to it when designing the stepper drive: https://github.com/unlir/XDrive This project, which is an open-source closed-loop drive by a friend of mine, based on STM32. The driver is divided into an open source version and a closed source version. The closed source version is based on a discrete MOSFET with extremely strong performance and complete functions. The open source version uses the ADC + chopper driver chip, which has basic functions and does not have a CAN protocol.

I redesigned the PCB circuit based on XDrive, added hardware support for the CAN bus, and completely refactored the original core code. **It also provides compiled binary files that can be burned directly:**

![](5.Docs/1.Images/fw2.png)

**The main improvements are as follows:**

1. Use C++11 to refactor the code, introduce many high-level language features, and mix the bottom part with C, which does not affect code performance
2. The hardware dependency is completely decoupled, which can be easily transplanted to the MCU of other platforms in the future, and the redundant code code is removed, and the structured logic is clearer
3. Added custom templates for CAN protocol and UART protocol
4. Added the parameter storage of simulated EEPROM, which can save data after power off
5. Added the setting of zero point at any position, and it is guaranteed to return to zero within a half circle in both directions (instead of one-way return to zero)
6. Fully compatible with STM32-HAL library, you can use STM32CubeMX to directly generate configuration code
7. Other improvements, you only need to pay attention to the files in the UserApp folder for secondary development

The Ctrl-Step driver is relatively simple to use. After downloading the firmware, the encoder will be calibrated when the motor is powered on for the first time. If it is successfully powered on the next time, pressing button 1 will enter the closed loop mode and send commands via CAN or serial port. You can control the motor. For instructions on the instructions, please refer to `interface_can.cpp` and `interface_uart.cpp` in the source code `UserApp` folder:

![](5.Docs/1.Images/fw3.png)

> The role of other buttons:
>
> * Press and hold two buttons at the same time to power on, it will automatically calibrate the encoder, if the first calibration fails, you can recalibrate this way
> * Short press button 1 to switch between **enable closed loop/disabled closed loop**
> * Long press button 1 to restart the board
> * Short press button 2 to clear the stall protection
> * Long press button 2 to reset the target value to zero (for example, if it is in position mode, the position will be reset to zero)
>
> Other functions should be set by code or communication protocol, such as setting **home zero**, **PID parameter**, CAN node ID, **various motion parameters**, etc., you can study the code yourself.

Of course, another way is that you can also use GRBL-type drivers to drive your robot. The problem with this solution is that the grbl firmware is relatively strong (after all, it is not designed for robot arms but CNC applications). It is easy to expand, and the pulse control method makes the wiring extremely inelegant (each joint has to pull the `step/dir` line to the controller separately, which causes the last few joints to be routed very long).

But I can use the integrated closed loop method to connect all the motors in series. The CAN bus makes the overall wiring only need four wires (two positive and negative power supply wires and two CAN signal wires). In addition, the bus model This enables the motor to work in the modes of `torque`, `speed`, `position`, and `trajectory`, while the pulse mode can only work in the position and trajectory mode, and complex control cannot be performed.

**For Peak, I have already open sourced both hardware and software before. You can go to the SubModules folder to see the README instructions over there. **

## About core firmware

The core of the firmware of this robotic arm is the kinematics attitude calculation, ~~I am still finishing this piece, and will be packaged more complete later for open source~~, **Already open source**, many of the hard-coded parameters will be designed into It is configurable, ** it is convenient for everyone to use this project to migrate to the robotic arm of their own design**.

**REF firmware instructions:**

The firmware mainly includes several major functional modules:

* BSP driver: various on-board hardware drivers such as OLED, IMU, LED, buzzer, non-volatile storage, etc.
* 3rdParty library: including U8G2 graphics library and Fibre serialization/deserialization library
* Core: ST official HAL library
* Driver: ARM's CMSIS driver
* Midwares: FreeRTOS support package
* Robot: core robot library, including various algorithms and driver codes
* UserApp: Upper-level application, you can develop other applications based on the API interface provided by me

> * The OLED is transplanted from the U8G2 library of Arduino, which can easily display various debugging and system information. In addition, because of the STM32 hardware I2C and BUG, ​​the software I2C is used to drive the screen, and the measured frame rate is higher than the hardware I2C.

The `DummyRobot` class is where the complete definition of Dummy is located. When initializing, you need to set up **stepper motor drive information** and **own DH parameters**:

![](5.Docs/1.Images/fw1.jpg)

The driver information includes: CAN node ID, **reverse direction**, reduction ratio of the reducer, **movement limit range**.

The meaning of DH parameters is as follows:

![](5.Docs/1.Images/fw2.jpg)

The configuration of the robotic arm needs to meet the Pieper criterion (the three adjacent joint axes of the robot intersect at one point or the three axes are parallel) to get an analytical solution, so you can modify it according to the structure of the Dummy, and then replace the DH parameters yourself. Can transplant my code.

> Regarding position memory, and power-on zero calibration:
>
> **Because the position of the absolute value encoder is only valid within one circle, the industrial manipulator is generally coded at the output end in order to obtain the absolute position after deceleration, but the accuracy is reduced by 30 times (reduction ratio), so it is more reasonable It is to do `dual encoder` or low-power encoder + battery; and the dual encoder in this project affects the compact design, so I used a more clever way: using the current loop driven by the motor to control the power-on Carry out low-torque zero-point directional motion, confirm the coarse zero point after touching the limit of the mechanical arm (the infinite position switch returns to zero), and then fine-tune the zero point according to the position of the single-turn absolute encoder. There is no error in the zero point of this method, and it is almost unaffected by the machining accuracy, because within 12 degrees (360/30) are the effective accuracy ranges of the absolute encoder. **

**Peak's firmware description:**

Peak is based on the X-Track project, you can go to the Peak warehouse to check it out.

## About the host computer

The software simulation in the video is based on RoboDK. In the video, I developed a Driver that connects to Dummy (the driver is described in the official document. The original version is based on the TCP network interface. I modified it to a serial port and compatible with the dummy protocol). However, because this software is charged, I also developed my own host computer based on Unity3D, which has been released in the warehouse.

There is no plan to open source the host computer at the moment, because there are still many functions to be added. I also hope that a general-purpose software similar to RoboDK can be made in the end. You can also use it when you build your own robotic arm. Of course, the software will definitely be free.

## About the control algorithm

First of all, the kinematics part has been implemented. Both the forward and inverse solutions are calculated by traditional DH parameters. The positive solution (joint angle to find the end pose) is the only solution that is easier to handle, and the inverse solution (end pose to find the joint angle) will involve Multiple solutions (usually 8), the algorithm I use here is ** to solve the smallest group of the 6 joints in the previous pose and the target pose as the inverse solution config**. In this way, it can be ensured that the robot arm always switches at the minimum angle of rotation.

Then, for the conversion of the joint angle to the input signal of the motor driver, I used the trapezoidal acceleration and deceleration curve for speed position planning. For example, in the MoveJ command, when a joint angle motion command is received, the controller will calculate the motion angle difference to obtain 6 motion difference angles, and then take the largest angle θ among the 6 difference angles, and at the same time according to the set JointSpeed Calculate the time required for the movement angle θ (consider the acceleration and deceleration). Use this time as the movement parameters of the remaining 5 motors to calculate their respective acceleration and deceleration speeds & maximum speeds, and then the 6 motors can move synchronously according to the calculated parameters. Ensure its synchronization and fluency.

The other six motors are connected by CAN bus, and each motor receives two ID numbers (own ID, ID 0), and ID 0 is used for information broadcasting and synchronization. After the motor receives the motion instruction, it stores the information in the shadow register, and starts to move after receiving the broadcast synchronization signal, which can further ensure the synchronization of the motor.

Finally, the dynamics part is still under development, and this one has not yet been fully realized. The kinematics and dynamics algorithms described above are strongly recommended to read the book "Introduction to Robotics"**, which is very detailed.

## Command mode

Dummy firmware supports three different command modes (commands can be received by USB, serial port, CAN), and the characteristics of different modes are different, see the following table:

| | Command sending frequency | Command execution mode | Can be interrupted by new commands | Pause between commands | Suitable for scenarios |
| --------------- | ----------------- | --------------- ------- | -------------- | ---------- | ---------------- -------------------------------------------- |
| SEQ (Sequence command) | Random, low (<5Hz) | FIFO queue executes in sequence| No| Yes| Send several key point poses at once, wait for execution in sequence to ensure that the key points arrive; but because between the key points There is a process of deceleration to 0 so there is a certain pause; suitable for scenes such as **visual grasping, palletizing and other applications**. |
| INT (real-time instruction) | Random, unlimited frequency | Instruction coverage, immediate execution | Yes | None | For real-time control, new instructions will overwrite the executing instructions and get an immediate response; but if a series of instructions are sent at once The effect will be to execute only the last one; suitable for scenes such as **action synchronization**. |
| TRJ (trajectory tracking) | Fixed, high (200Hz) | Automatic interpolation, fixed cycle execution | No | None | Suitable for applications that require precise trajectory tracking, the speed will slow down; example scenes such as **3D printing, engraving , Painting, etc.**. |

---



> **Thanks to the authors of the following projects:**
>
> * [unlir/XDrive: Stepper motor with multi-function interface and closed loop function. Stepper motor with multi-function interface and closed loop function. (github.com)](https://github.com/unlir/XDrive)
> * [odriverobotics/ODrive: High performance motor control (github.com)](https://github.com/odriverobotics/ODrive)
> * [olikraus/u8g2: U8glib library for monochrome displays, version 2 (github.com)](https://github.com/olikraus/u8g2)
> * [samuelsadok/fibre: Abstraction layer for painlessly building object oriented distributed systems that just work (github.com)](https://github.com/samuelsadok/fibre)
