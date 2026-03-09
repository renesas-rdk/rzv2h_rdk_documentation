Other interfaces
----------------

The RZ/V2H Robotic Development Kit (RDK) is equipped with several additional interfaces to enhance its functionality and connectivity options. This section provides an overview of these interfaces, including Micro-HDMI, USB-UART, and JTAG 10-pin.

Micro-HDMI
~~~~~~~~~~

The RZ/V2H Robotic Development Kit features a Micro-HDMI port for video output to an external display. To use this interface, connect a Micro-HDMI cable from the board to a compatible monitor.

.. note::

    - The display supports a maximum resolution of 1920x1920.
    - Before powering on the board, ensure that the HDMI cable is securely connected to avoid any display issues.

The Micro-HDMI interface also supports audio output, allowing you to transmit both video and audio signals through the same connection.

This is particularly useful for multimedia applications.

.. warning:: HDMI Connection

   An error log may appear for the HDMI connection when audio is enabled. This issue does not affect HDMI functionality, and you can safely ignore the log message. We are actively working to resolve this issue in a future update.

   For example, the following log messages may appear:

   .. code-block:: bash

      # At boot time, you may see log messages similar to the following:
      [    6.464257] rzv2h-cpg 10420000.clock-controller: Failed to enable CLK_ON (____ptrval____)
      [    6.477102] rzv2h-cpg 10420000.clock-controller: Failed to enable CLK_ON (____ptrval____)
      [    6.489992] rzv2h-cpg 10420000.clock-controller: Failed to enable CLK_ON (____ptrval____)
      [    6.502829] rzv2h-cpg 10420000.clock-controller: Failed to enable CLK_ON (____ptrval____)

      # At runtime, you may see log messages similar to the following:
      [   30.416502] rzv2h-cpg 10420000.clock-controller: Failed to enable CLK_ON 00000000d485f897
      [   30.424814] rcar_sound 13c00000.sound: __pm_clk_enable: failed to enable clk 00000000acf14272, error -110
      [   30.439032] rzv2h-cpg 10420000.clock-controller: Failed to enable CLK_ON 00000000d485f897
      [   30.447179] rcar_sound 13c00000.sound: __pm_clk_enable: failed to enable clk 0000000081ff9831, error -110
      [   30.461295] rzv2h-cpg 10420000.clock-controller: Failed to enable CLK_ON 00000000d485f897
      [   30.469440] rcar_sound 13c00000.sound: __pm_clk_enable: failed to enable clk 00000000869af618, error -110
      [   30.636770] hdmi-audio-codec hdmi-audio-codec.0.auto: Only one simultaneous stream supported!
      [   30.645891] hdmi-audio-codec hdmi-audio-codec.0.auto: ASoC: error at snd_soc_dai_startup on i2s-hifi: -22

   To avoid these log messages, **access the board over SSH**, because they are displayed only on the USB-UART interface.

.. _usb_uart:

USB-UART
~~~~~~~~

The RZ/V2H RDK includes a USB-UART interface for serial communication and debugging purposes. This interface allows you to connect the board to a host computer via a USB cable and access the serial console.

We recommend using a terminal emulator such as `minicom` or `Tera Term` to connect to the USB-UART interface.

The configuration settings for the serial connection are as follows:

- Baud Rate: 115200
- Data Bits: 8
- Parity: None
- Stop Bits: 1
- Flow Control: None

.. tip::

   If your PC cannot recognize the USB-UART interface, **power-off then power-on** (do not use the ``sudo reboot`` command) the board and try again.

JTAG 10-pin
~~~~~~~~~~~

The RZ/V2H RDK provides a JTAG 10-pin interface for debugging and programming CM33 and two CR8 cores.

This interface is essential for low-level debugging and development tasks in Multi-Core applications.

For more information on using the JTAG interface, please refer to the RZ/V2H RDK :ref:`Multi-OS Development Section <multi_os>`.