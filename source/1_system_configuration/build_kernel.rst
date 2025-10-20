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

   renesas@docker-pc:~$ cd ~/poky_sdk/workspace/sources/linux-yocto/
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

        renesas@docker-pc:~$ mkdir -p ~/poky_sdk/workspace/recipes-kernel/linux/linux-yocto/files/

    - Create a configuration fragment file, e.g., `myconfig.cfg`, inside the `files/` directory. For example:

    .. code-block:: bash

        CONFIG_USB_SERIAL=y
        CONFIG_USB_SERIAL_FTDI_SIO=y

    .. note::

       Please include all dependency configs as well.

    - Edit the `linux-yocto_%.bbappend` file in the workspace’s recipe area to include the new configuration fragment:

    .. code-block:: bash

        renesas@docker-pc:~$ vim ~/poky_sdk/workspace/appends/linux-yocto/linux-yocto_6.10.bbappend

    - Add the following line to the bbappend file:

    .. code-block:: bash

        SRC_URI:append = " file://myconfig.cfg"

4. Modify the DTS file:

   .. code-block:: bash

      renesas@docker-pc:~$ cd ~/poky_sdk/workspace/sources/linux-yocto/
      renesas@docker-pc:~$ vi arch/arm64/boot/dts/renesas/r9a09g057h4-rdk-ver1.dts

   Make the necessary changes to the Device Tree Source file (``.dts``) according to your requirements.

5. Build the Modified Kernel

After making changes in the workspace, use devtool build to compile the recipe.

.. code-block:: bash

    renesas@docker-pc:~$ devtool build -c clean linux-yocto
    renesas@docker-pc:~$ export DISTRO=ubuntu-tiny
    renesas@docker-pc:~$ devtool build linux-yocto

.. attention::

   Before building, always **clean the previous build artifacts** to avoid conflicts.

   .. code-block:: bash

      renesas@docker-pc:~$ devtool build -c clean linux-yocto

   Always set the ``DISTRO`` variable to ``ubuntu-tiny`` to ensure compatibility
   with the Ubuntu-based root filesystem.

   Otherwise, the generated artifacts may **not be compatible** with the Ubuntu image.

1. Collect the Built Kernel Artifacts

Once the build is complete, collect the built kernel artifacts for deployment to the target hardware.

When building ``linux-yocto``, the generated artifacts can be collected from the following locations: `~/poky_sdk/workspace/sources/linux-yocto/oe-workdir/image`

Please copy those files to the appropriate boot media (e.g., SD card) as per your deployment requirements.

.. tip::

   On the target device, make sure to run the following command to update the module
   dependencies after deployment:

   .. code-block:: bash

      sudo depmod -a