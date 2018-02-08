# stm32duino_2DCMotor_Encoders
Control with STM32F103C8 2 DC Motors with enconders and use PID to control positions.

Using inexpensive Mabutchi 5v motors with encoders, and an MX1508 H-bridge (L294D compatible),
a control system can by build, with fast and accurate motion, without losing steps (no stepper motors)
and auto return to position when a external force bring motors out of its target position.

There is a version of the program for tuning PID parameters, with 1 sec cycle, and another one without delay for
fast response.

The current interface is base in serial UART1 of STM32F103C8, but the intention is to couple either a HC05 bluetooth
serial converter for Mobile phone operation, or an ESP8266 for Wifi IoT interface. 

The project could be adapted for a single ESP8266 but Wifi interfere with Encoders interrups and the speed is lower.
STM32F103C8 is a cheap MCU that has special support for Encoders using Timers, but this implementation only use
standard ISR interrupt service routine with an array that contain the quadrature states, that avoid any "if" on the ISR
outperforming any other algorithm.

There are several projects on top of this controller, but I prefered isolate this as an standard "DC Servo Controller" 
using a very cheap and powerful STM32 Mcu.

The code is stm32duino, an implementation of arduino API in C for easy development.

Have fun,

Agustin

https://github.com/agnunez/stm32duino_2DCMotor_Encoders 2018
