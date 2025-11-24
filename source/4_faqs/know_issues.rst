Know Issues
---------------------------

**1. USB camera warning and error messages when running AI applications**

The following warning and error messages may appear in the terminal when running AI applications that use USB cameras 2.0 on the RZ/V2H RDK:

.. code-block:: text

    [ 3495.792814] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
    [ 3495.801878] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
    [ 3495.810911] xhci-renesas-hcd 15860000.usb: ERROR Transfer event TRB DMA ptr not part of current TD ep_index 2 comp_code 13
    [ 3495.821917] xhci-renesas-hcd 15860000.usb: Looking for event-dma 000000004b0b1190 trb-start 000000004b0b11a0 trb-end 000000004b0b11a0 seg-start 000000004b0b1000 seg-end 000000004b0b1ff0
    [ 3496.032127] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
    [ 3496.041226] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
    [ 3496.299376] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2

We have observed that these warning and error messages do not impact the functionality of the USB camera or the performance of the AI applications.

The applications continue to operate as expected despite the presence of these messages.

There is no issue if you use the USB camera 3.0 on the RZ/V2H RDK.

We are actively investigating the root cause of these messages and working towards a resolution.

**2. Some SD cards may not work properly with the RZ/V2H RDK**

Some SD cards may not function correctly with the RZ/V2H RDK, leading to issues such as failure to boot or read/write errors.

To ensure compatibility, we recommend using **SD cards which accept the high-speed mode** and are from reputable brands, such as SanDisk, Samsung, or Kingston.