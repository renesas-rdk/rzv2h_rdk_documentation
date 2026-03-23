Known Issues
------------

#. **USB camera warning and error messages when running AI applications**

   The following warning and error messages may appear in the terminal when running AI applications that use USB 2.0 cameras on the RZ/V2H RDK:

   .. code-block:: text

      [ 3495.792814] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3495.801878] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3495.810911] xhci-renesas-hcd 15860000.usb: ERROR Transfer event TRB DMA ptr not part of current TD ep_index 2 comp_code 13
      [ 3495.821917] xhci-renesas-hcd 15860000.usb: Looking for event-dma 000000004b0b1190 trb-start 000000004b0b11a0 trb-end 000000004b0b11a0 seg-start 000000004b0b1000 seg-end 000000004b0b1ff0
      [ 3496.032127] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3496.041226] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3496.299376] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2

   These messages do not affect the functionality of the USB camera or the performance of the AI applications.
   The applications continue to operate as expected.

   .. note::

      This issue does not occur with USB 3.0 cameras on the RZ/V2H RDK.

   We are actively investigating the root cause and working towards a resolution.

#. **Some SD cards may not work properly with the RZ/V2H RDK**

   Some SD cards may not function correctly with the RZ/V2H RDK, leading to issues such as failure to boot or read/write errors.

   To ensure compatibility, use **SD cards that support high-speed mode** from reputable brands such as SanDisk, Samsung, or Kingston.

#. **The Ethernet may not work properly when booting up the board**

   The following error may occur when booting Ubuntu 24.04 on the RZ/V2H RDK, causing no Internet connection even though the Ethernet cable is connected:

   .. code-block:: bash

      [   17.664297] dwc-eth-dwmac 15c30000.ethernet end0: __stmmac_open: Cannot attach to PHY (error: -110)

   To resolve this issue:

   #. Power off the board.
   #. Unplug the Ethernet cable.
   #. Power on the board.
   #. Wait until the system fully boots.
   #. Plug the Ethernet cable back in.