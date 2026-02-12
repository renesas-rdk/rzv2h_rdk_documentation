Custom Linux Real-time Kernel and Device Tree
------------------------------------------------------------------

This section describes how to customize and rebuild the **Linux Real-time kernel** for the RZ/V2H RDK board using the Yocto eSDK and devtool.

.. important::

   Before proceeding, ensure that you have deploy ``board_setup/rt_kernel`` to your RZ/V2H RDK board as described in the ``board_setup/rt_kernel/Readme.md``. You can find the ``board_setup/rt_kernel`` in the software release package.

   This image includes the real-time kernel and necessary configurations.

   If you want to build your own real-time kernel, follow the steps below.

To enable support for Linux real-time kernel, you need to patch the kernel with the necessary patch. Follow these steps on the host machine where the Yocto eSDK is set up:

1. Download the following real-time kernel patch files from **Common Utils** repository:

   - `0000-rz-cmn-rz-Add-realtime-kernel-patch.patch <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/utility/common_utils/-/blob/main/linux_utils/kernel_patches/0000-rz-cmn-rz-Add-realtime-kernel-patch.patch?ref_type=heads>`_
   - `rt_kernel.cfg <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/utility/common_utils/-/blob/main/linux_utils/kernel_cfg/rt_kernel.cfg?ref_type=heads>`_

2. Complete the **Step 1 and 2: Applying Patches for linux-yocto** in the :ref:`Custom Linux Kernel and Device Tree <build_kernel>` section.
3. Copy the ``rt_kernel.cfg`` and ``0000-rz-cmn-rz-Add-realtime-kernel-patch.patch`` files to the `~/poky_sdk/workspace/appends/linux-yocto/` directory.

   .. code-block:: bash
       :emphasize-lines: 2

       renesas@docker-pc:~$ mkdir -p ~/poky_sdk/workspace/appends/linux-yocto/
       renesas@docker-pc:~$ cd ~/downloads  # Change to the directory where you downloaded the files

       renesas@docker-pc:~$ cp rt_kernel.cfg ~/poky_sdk/workspace/appends/linux-yocto/
       renesas@docker-pc:~$ cp 0000-rz-cmn-rz-Add-realtime-kernel-patch.patch ~/poky_sdk/workspace/appends/linux-yocto/

4. Edit the `~/poky_sdk/workspace/appends/linux-yocto_6.10.bbappend` file to include the new configuration fragment at the end of file:

   .. code-block:: bash

      renesas@docker-pc:~$ vi ~/poky_sdk/workspace/appends/linux-yocto_6.10.bbappend

   - Add the following line to the bbappend file:

   .. code-block:: bash

        SRC_URI:append = " file://rt_kernel.cfg "

5. Apply the real-time kernel patch to the kernel source:

   .. code-block:: bash

       renesas@docker-pc:~$ cd ~/poky_sdk/workspace/sources/linux-yocto
       renesas@docker-pc:~/poky_sdk/workspace/sources/linux-yocto$ git am ~/poky_sdk/workspace/appends/linux-yocto/0000-rz-cmn-rz-Add-realtime-kernel-patch.patch

6. **Skip step 3 and 4**, continue build the kernel and deploy it by following **Step 5: Build the Modified Kernel** in the :ref:`Custom Linux Kernel and Device Tree <build_kernel>` section.

.. important::

   For deploying real-time kernel step:

   - The ``~/poky_sdk/workspace/sources/linux-yocto/oe-workdir/image/boot/Image-6.10.14-rt14-yocto-standard`` should be copied to the target board's ``/boot`` directory as ``Image`` not ``Image-6.10.14-rt14-yocto-standard``.
   - Please don't remove the existing real-time kernel module driver under ``/lib/modules/6.10.14-rt14-yocto-standard/kernel/drivers/`` on the target board.

   As there are some kernel module drivers (build with 'm' type) are built with the real-time kernel image. Building the linux-yocto recipe will not build these module drivers again.

   If you want to add new module drivers, please copy the new module drivers only to the target board.

   If you want to switch back to the standard kernel, remove the commit added in ``Step 5`` from the linux-yocto source tree in your workspace, and delete the kernel configuration added in ``Step 4`` from the linux-yocto_6.10.bbappend file.