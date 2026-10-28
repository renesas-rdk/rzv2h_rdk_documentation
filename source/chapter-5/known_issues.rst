Known Issues
------------

#. **Some microSD cards may not work properly with the RZ/V2H RDK**

   Some microSD cards may not function correctly with the RZ/V2H RDK, leading to issues such as failure to boot or read/write errors.

   To ensure compatibility, use **microSD cards that support high-speed mode** from reputable brands such as SanDisk, Samsung, or Kingston.

#. **The Ethernet may not work properly when booting up the board**

   The following error may occur when booting Ubuntu 24.04 on the RZ/V2H RDK, causing no internet connection even though the Ethernet cable is connected:

   .. code-block:: text

      [   17.664297] dwc-eth-dwmac 15c30000.ethernet end0: __stmmac_open: Cannot attach to PHY (error: -110)

   **Cause**: the Ethernet PHY is reset only at power-on. If the board is powered on again too quickly after power-off, the supply does not discharge completely, the PHY is not reset properly, and Linux cannot attach to it.

   **Workaround**: power off the board, wait at least 5 seconds, and power it on again. Do not power-cycle the board too quickly.

#. **The balenaEtcher may not work properly with *.img.xz files**

   When flashing the RZ/V2H RDK image to a microSD card using balenaEtcher, if you select the compressed ``.img.xz`` file, the flashing process may fail or not complete successfully.

   To avoid this issue, **decompress the image file** first and then use the resulting ``.img`` file for flashing with balenaEtcher.

   You can decompress the image file using the following command on Linux:

   .. code-block:: bash

      xz -dk ubuntu-24.04-server-arm64-rzv2h-rdk-<ver>.img.xz   # <ver>: ver1 or ver101

   Or use a decompression tool on Windows or macOS to extract the ``.img`` file.