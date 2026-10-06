.. _device_tree_overlay:

Device Tree Overlay
^^^^^^^^^^^^^^^^^^^

Device Tree Overlay (DTO) is a mechanism used in embedded systems to modify the device tree at runtime.

It allows developers to add, remove, or change the properties of devices without needing to recompile the entire device tree.
This is particularly useful for systems that support hot-plugging of devices or for testing new hardware configurations.

On the RZ/V2H RDK, you can enable or disable specific device tree overlays by editing the ``/boot/uEnv.txt`` file.

Each overlay corresponds to a specific hardware configuration, such as enabling the CAN interface, the audio codec, or remoteproc for the CM33/CR8 cores.

How to Configure Environment Variables in ``uEnv.txt``
""""""""""""""""""""""""""""""""""""""""""""""""""""""

.. caution::

   Only comment or uncomment the device tree overlay settings and ``rootdev`` in this file.
   Do not modify any other entries, as doing so may cause unpredictable
   boot behavior.

The ``/boot/uEnv.txt`` file defines U-Boot environment variables. You can edit
this file from Linux user space to match your desired U-Boot configuration.

You can also define additional U-Boot environment variables in ``/boot/uEnv.txt``
to override the default values compiled into U-Boot.

The default U-Boot environment variables defined in ``/boot/uEnv.txt`` are listed below:

.. code-block:: text
   :emphasize-lines: 4,5,6,10

   # ---------------------------------------------------------------------------
   # DT overlays (boot/dtb/renesas/overlays/*.dtbo), set to 1 or yes to enable
   # ---------------------------------------------------------------------------
   #enable_overlay_audio_codec=1
   enable_overlay_can=1
   #enable_overlay_spi=1
   # remoteproc (CM33, CR8 core0/core1): only with the remoteproc IPL
   # (RZ_SRAM_REGION_ACCESS + RZ_REMOTEPROC). Leave it off for CM33 cold boot,
   # CM33 firmware load/CA55 1.8GHz or CM33/CR8 started from U-Boot.
   enable_overlay_remoteproc=1

.. tip::

   To enable an overlay, remove the comment symbol ``#`` from the beginning of
   the corresponding ``enable_overlay_*`` line. To disable it, add the ``#``
   symbol back.

RZ/V2H RDK U-Boot Environment
"""""""""""""""""""""""""""""

The following table describes the available overlay loading options.

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Configuration
     - Valid values
     - Overlay loaded
   * - ``enable_overlay_audio_codec``
     - ``1`` or ``yes``
     - ``rzv2h-rdk-audio-codec.dtbo``
   * - ``enable_overlay_can``
     - ``1`` or ``yes``
     - ``rzv2h-rdk-can.dtbo``
   * - ``enable_overlay_spi``
     - ``1`` or ``yes``
     - ``rzv2h-rdk-ext-spi.dtbo``
   * - ``enable_overlay_remoteproc``
     - ``1`` or ``yes``
     - ``rzv2h-rdk-remoteproc.dtbo``

**Overlay descriptions:**

- ``enable_overlay_audio_codec``

  Enables the external audio codec overlay. When this overlay is enabled,
  Micro-HDMI audio output is disabled.

  Refer to the :ref:`Audio Interface Section <audio_interface>` for more details about using the audio interface.

- ``enable_overlay_can``

  Enables the CAN interface overlay. Refer to the :ref:`CAN-FD Interface Section <can_interface>` for more details about using the CAN interface.

- ``enable_overlay_spi``

  Enables the external SPI interface overlay for ``rsci_spi0``.

- ``enable_overlay_remoteproc``

  Adds the remoteproc nodes for the CM33 and CR8 cores (``cm33``, ``cr8_0``, ``cr8_1``) and their OpenAMP shared memory (``0x43000000``-``0x434FFFFF``). Linux can then load and start the CM33/CR8 firmware through ``/sys/class/remoteproc/``. Enabled by default.

  Refer to the :ref:`Remoteproc Support Section <multi_os_remoteproc>` for more details.

  .. warning::

     Enable this overlay **only** with the default remoteproc IPL (``RZ_SRAM_REGION_ACCESS`` + ``RZ_REMOTEPROC``). Disable it with any other Multi-OS IPL mode (CM33 cold boot, CM33 firmware load/CA55 1.8 GHz, CM33/CR8 started from U-Boot). In these modes, the IPL, U-Boot, or CM33 already starts the remote cores, and a remoteproc ``stop``/``start`` resets them. See :ref:`build_ipl`.

.. note::

   When ``enable_overlay_spi`` is enabled, the ``rsci_spi0`` interface uses
   the following pins:

   - ``P50 - GPIO25 - Pin number 22``: MOSI
   - ``P51 - GPIO16 - Pin number 36``: MISO
   - ``P52 - GPIO26 - Pin number 37``: SCK
   - ``P53 - GPIO6  - Pin number 31``: SS (used only in slave mode)

Additional variables:

- U-Boot environment variables

  You can also define standard U-Boot environment variables here. Refer to the U-Boot documentation for more details.
