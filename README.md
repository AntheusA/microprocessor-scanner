# microprocessor-scanner

This is my microprocessor systems final project for 2DX3.

The project uses a TI MSP432E401Y microcontroller and a ToF (time-of-flight) sensor to gather distance measurements.
These measurements are then sent to a Python program using UART where they are plotted in 3D space to create a scan.

The ToF sensor has been mounted on top of a stepper motor which rotates in 360° to make vertical slices.
The system is moved in between each scan and the vertical slices can be put together to make a graphical view of the area.

Check out the report document to see a more detailed explanation of how the system works and what it can create!

<img width="238" alt="Scan Image" src="https://github.com/AntheusA/microprocessor-scanner/assets/83626108/dbf473ad-7a89-40dc-a506-01bb90102d59">

*scan of a hallway*
