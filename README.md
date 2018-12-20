# PIC-and-Pi-Interface
To combine the realtime PIC32 and the powerful Raspberry Pi

The idea is to use a Raspberry Pi running a Linux distribution to perform high level tasks,
provide connectivity and user interface and use the PIC32 microcontroller to perform low level
and time critical tasks. This requires a hardware dependent firmware on the PIC32 capable of
responding to commands, and application specific software running on Raspberry Pi over the
Linux OS and a high speed interface between the two to enable the Pi to control the PIC. The
command line between the Pi and PIC will be encapsulated into a library running on the Pi,
which will abstract the functions of the PIC microcontroller and act as driver for the embedded
hardware. The application can perform low level tasks by making library calls without worrying
about the actual implementation.

Both the devices are combined to take advantage of the PIC32 peripherals for
interfacing, and the high performance of Pi on computation. The plan is to use the PIC32 for
input reading, output generation, and use the Pi for decision making and user-interface. The
resultant system would have a master slave configuration with Raspberry Pi as Master and PIC32
as slave, and command response interface between the two.
