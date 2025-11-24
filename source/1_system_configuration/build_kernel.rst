.. _build_kernel:

Custom Linux Kernel and Device Tree
------------------------------------

This section describes how to customize and rebuild the Linux kernel or device tree blob for the RZ/V2H RDK board using the Yocto eSDK and devtool.

.. note::

   Before proceeding, ensure that you have set up the eSDK environment as described in the :ref:`eSDK Setup <esdk_setup>`  section above.

1. modify the Linux kernel recipe:

   Setup the eSDK environment in the current terminal session:

   .. code-block:: bash

      renesas@docker-pc:~$ source ~/poky_sdk/environment-setup-cortexa55-poky-linux

   Use the devtool modify command to check out the linux-yocto recipe for modification:

   .. code-block:: bash

      renesas@docker-pc:~$ devtool modify linux-yocto

When executed, this:

- Creates a workspace copy of the kernel source under: `~/poky_sdk/workspace/sources/linux-yocto/`

- Generates a .bbappend for linux-yocto in the workspace’s recipe area.

- Prepares the environment for kernel modifications.

2. Applying Patches for linux-yocto

Unlike most recipes, linux-yocto in this BSP is implemented as an out-of-tree kernel.

- Kernel modifications are stored as patches inside workspace/sources/linux-yocto/.kernel-meta/

- The kernel default configuration (renesas_defconfig) is also managed out-of-tree.

- To ensure the workspace matches the recipe, patches must be applied after running devtool modify.

Procedure, applying patches to the kernel linux-yocto source

.. code-block:: bash

   renesas@docker-pc:~$ cd ~/poky_sdk/workspace/sources/linux-yocto/.kernel-meta
   renesas@docker-pc:~/poky_sdk/workspace/sources/linux-yocto$ git am $(cat patch.queue)

This applies the patch series defined in .kernel-meta/patches/ to the kernel workspace.

After applying the patches, developers may perform the following steps:

- Provide kernel configuration fragments (``.cfg`` files) to adjust features
  or enable additional built-in kernel drivers.

- Modify the kernel source code as needed for custom functionality.

- Continue with the ``devtool build linux-yocto`` command
  to compile the kernel with the applied modifications.

3. Adding Kernel Configuration Fragments

.. tip::

   If you're unsure whether the config has been added to the kernel configuration,

   you can use `zcat /proc/config.gz | grep <CONFIG_NAME>` in the target machine to check whether it is enabled.

There are two possible methods to add kernel configuration for linux-yocto:

- Edit the out-of-tree defconfig directly: `~/poky_sdk/layers/meta-renesas/recipes-kernel/linux/rz-cmn/common/kernel-common.cfg`

- Adding a configuration fragment (.cfg) file
    - Create the append structure

    .. code-block:: bash

        renesas@docker-pc:~$ mkdir -p ~/poky_sdk/workspace/appends/linux-yocto/

    - Create a configuration fragment file, e.g., `myconfig.cfg`, inside the `~/poky_sdk/workspace/appends/linux-yocto/` directory. For example:

    .. code-block:: bash

        CONFIG_USB_SERIAL=y
        CONFIG_USB_SERIAL_FTDI_SIO=y

    .. note::

       Please include all dependency configs as well.

    - Edit the `~/poky_sdk/workspace/appends/linux-yocto_6.10.bbappend` file to include the new configuration fragment at the end of file:

    .. code-block:: bash

        renesas@docker-pc:~$ vi ~/poky_sdk/workspace/appends/linux-yocto_6.10.bbappend

    - Add the following line to the bbappend file:

    .. code-block:: bash

        SRC_URI:append = " file://myconfig.cfg"

.. _modify_dts:

4. Modify the DTS file:

   .. code-block:: bash

      renesas@docker-pc:~$ cd ~/poky_sdk/workspace/sources/linux-yocto/
      renesas@docker-pc:~$ vi arch/arm64/boot/dts/renesas/r9a09g057h4-rdk-ver1.dts

   Make the necessary changes to the Device Tree Source file (``.dts``) according to your requirements.

5. Build the Modified Kernel

After making changes in the workspace, use devtool build to compile the recipe.

.. code-block:: bash

    renesas@docker-pc:~$ devtool build -c linux-yocto
    renesas@docker-pc:~$ export DISTRO=ubuntu-tiny
    renesas@docker-pc:~$ devtool build linux-yocto

.. attention::

   Before building, always **clean the previous build artifacts** to avoid conflicts.

   .. code-block:: bash

      renesas@docker-pc:~$ devtool build -c linux-yocto

   Always set the ``DISTRO`` variable to ``ubuntu-tiny`` to ensure compatibility
   with the Ubuntu-based root filesystem.

   Otherwise, the generated artifacts may **not be compatible** with the Ubuntu image.

6. Collect the Built Kernel Artifacts

Once the build is complete, collect the built kernel artifacts for deployment to the target hardware.

When building ``linux-yocto``, the generated artifacts can be collected from the following locations: `~/poky_sdk/workspace/sources/linux-yocto/oe-workdir/image`

Please copy those files to the appropriate boot media (e.g., SD card) as per your deployment requirements.

.. tip::

   1. Make sure to copy the updated kernel image and device tree blob to the correct locations
      in the root file system on the SD card.

      For example:

      .. list-table:: File Mapping Between eSDK Build Output and Target RootFS
         :header-rows: 1
         :widths: 45 45

         * - **eSDK Build Output Path**
           - **Target RootFS Location**
         * - ``~/poky_sdk/workspace/sources/linux-yocto/oe-workdir/image/boot/Image-6.10.14-yocto-standard``
           - ``/boot/Image-6.10.14-yocto-standard``
         * - ``~/poky_sdk/workspace/sources/linux-yocto/oe-workdir/image/boot/dtb/renesas/r9a09g057h4-rdk-ver1.dtb``
           - ``/boot/dtb/renesas/r9a09g057h4-rdk-ver1.dtb``
         * - ``~/poky_sdk/workspace/sources/linux-yocto/oe-workdir/image/usr/lib/modules``
           - ``/usr/lib/modules``

      .. important::

         On the original RZ/V2H RDK image, the kernel image is named ``Image`` which link to the ``Image-6.10.14-yocto-standard`` file.

         The u-boot configuration on the RZ/V2H RDK board loads the kernel image with the name ``Image`` from the ``/boot`` directory.

         Please make sure to maintain this naming convention when copying the new kernel image to the target root filesystem.

         If your new kernel image has a different name, please rename it to ``Image`` after copying it to the target root filesystem.

   2. On the target device, run the following command to update the module
      dependencies after deployment:

      .. code-block:: bash

         $ sudo depmod -a
