# IR_LED

This is an open-source infra-red illumnination solution (LED panel and driver) for machine vision.
An arduino nano every sits at the core of the driver board, and allows remote control from a PC through serial communication

The main features are: 

- infra-red LED panel, with 3 separately addressable concentric rings  
- manual or remote control of the brightness for the 3 channels
- LED driver can be strobed with the open-collector trigger output of a machine vision camera,
which significantly reduces heat output
- the temperature of the LED panel is measured in real time and can by controlled with a fan. 
The fan PWM can be set manually or via a software PID controller on the arduino


## hardware

PCB design files (Kicad v10)

## firmware

Arduino code (Arduino Nano Every)
 
## software

Python driver and GUI on the PC side 