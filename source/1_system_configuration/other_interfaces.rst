Other interfaces
--------------------------------

7. Mini-HDMI
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The RZ/V2H Robotic Development Kit features a Mini-HDMI port for video output to an external display. To use this interface, connect a Mini-HDMI cable from the board to a compatible monitor.

TODO: Add more information, with Ubuntu Core, there is no display output by default.

8. USB-UART
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The RZ/V2H RDK includes a USB-UART interface for serial communication and debugging purposes. This interface allows you to connect the board to a host computer via a USB cable and access the serial console.

We recommend using a terminal emulator such as `minicom` or `Tera Term` to connect to the USB-UART interface.

The configuration settings for the serial connection are as follows:

- Baud Rate: 115200
- Data Bits: 8
- Parity: None
- Stop Bits: 1
- Flow Control: None

9.  JTAG 10-pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The RZ/V2H RDK provides a JTAG 10-pin interface for debugging and programming CM33 and two CR8 cores.

This interface is essential for low-level debugging and development tasks in Multi-Core applications.

For more information on using the JTAG interface, please refer to the RZ/V2H RDK `Multi-OS Development Section`.

TODO: Add link to Multi-OS Development Section