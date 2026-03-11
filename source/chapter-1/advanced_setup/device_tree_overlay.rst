Device Tree Overlay
-------------------

Device Tree Overlay (DTO) is a mechanism used in embedded systems to modify the device tree at runtime.

It allows developers to add, remove, or change the properties of devices without needing to recompile the entire device tree.
This is particularly useful for systems that support hot-plugging of devices or for testing new hardware configurations.

On the RZ/V2H RDK, you can enable or disable specific device tree overlays by editing the ``/boot/uEnv.txt`` file.

Each overlay corresponds to a specific hardware configuration, such as enabling the CAN interface or the audio codec.

How to Configure Environment Variables in ``uEnv.txt``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. caution::

   Only comment or uncomment device tree overlay settings in this file.
   Do not modify any other entries, as doing so may cause unpredictable
   boot behavior.

The ``/boot/uEnv.txt`` file defines U-Boot environment variables. You can edit
this file from Linux user space to match your desired U-Boot configuration.

You can also define additional U-Boot environment variables in ``/boot/uEnv.txt``
to override the default values compiled into U-Boot.

The default U-Boot environment variables defined in ``/boot/uEnv.txt`` are listed below:

.. code-block:: text
   :emphasize-lines: 3,4,5

   # uEnv.txt for RZV2H RDK board
   # Refer to readme.txt for more information on setting up U-Boot Env
   #enable_overlay_audio_codec=1
   enable_overlay_can=1
   #enable_overlay_spi=1
   #
   # Config for loading the dtbo
   #
   # Other settings...

.. tip::

   To enable an overlay, remove the comment symbol ``#`` from the beginning of
   the corresponding ``enable_overlay_*`` line. To disable it, add the ``#``
   symbol back.

RZ/V2H RDK U-Boot Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following table describes the available overlay loading options.

+-------------------------------+---------------+-----------------------------------+
| Configuration                 | Valid Values  | Overlay Loaded                    |
+===============================+===============+===================================+
| ``enable_overlay_audio_codec``| ``1`` or      | ``rzv2h-rdk-1.0-audio-codec.dtbo``|
|                               | ``yes``       |                                   |
+-------------------------------+---------------+-----------------------------------+
| ``enable_overlay_can``        | ``1`` or      | ``rzv2h-rdk-1.0-can.dtbo``        |
|                               | ``yes``       |                                   |
+-------------------------------+---------------+-----------------------------------+
| ``enable_overlay_spi``        | ``1`` or      | ``rzv2h-rdk-1.0-ext-spi.dtbo``    |
|                               | ``yes``       |                                   |
+-------------------------------+---------------+-----------------------------------+

**Overlay descriptions:**

- ``enable_overlay_audio_codec``

  Enables the external audio codec overlay. When this overlay is enabled,
  micro HDMI audio output is disabled.

  Please refer to the :ref:`Audio Interface Section <audio_interface>` for more details about using the audio interface.

- ``enable_overlay_can``

  Enables the CAN interface overlay. Please refer to the :ref:`CAN FD Interface Section <can_interface>` for more details about using the CAN interface.

- ``enable_overlay_spi``

  Enables the external SPI interface overlay for ``rsci_spi0``.

.. note::

   When ``enable_overlay_spi`` is enabled, the ``rsci_spi0`` interface uses
   the following pins:

   - ``P50 - GPIO25 - Pin number 22``: MOSI
   - ``P51 - GPIO16 - Pin number 36``: MISO
   - ``P52 - GPIO26 - Pin number 37``: SCK
   - ``P53 - GPIO6  - Pin number 31``: SS (used only in slave mode)

Additional variables:

- ``fdtfile``

  Specifies the base DTB file. Set this variable to
  ``rzv2h-rdk-ver1.dtb``.

- U-Boot environment variables

  You can also define standard U-Boot environment variables here, please refer to the U-Boot documentation for more details.