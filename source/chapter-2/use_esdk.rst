Using devtool in the Yocto eSDK
--------------------------------
The devtool utility is part of the Yocto Project’s Extensible SDK (eSDK). It provides an isolated workspace for modifying, testing, and maintaining recipes without altering upstream metadata.

This section focuses on the core commands used in day-to-day development, especially for Linux kernel, device trees, and driver modifications on Renesas RZ Common System.

1. devtool modify

   The devtool modify command checks out the source code for a recipe into a local workspace, allowing changes without touching upstream layers.
   This is usually the first step in customizing the kernel, device trees, or drivers.

   Example: To modify the Linux kernel recipe:

   .. code-block:: bash

       renesas@docker-pc:~$ devtool modify linux-yocto

   This command sets up a workspace where you can make changes to the Linux kernel source code.

2. devtool build

   After making changes to the source code, use this command to build the modified recipe.

   Example: To build the modified Linux kernel:

   .. code-block:: bash

       renesas@docker-pc:~$ devtool build linux-yocto

   This command compiles the changes and prepares them for deployment.

3. devtool reset

   If you want to discard your changes and revert to the original recipe, use this command.

   Example: To reset the Linux kernel recipe:

   .. code-block:: bash

       renesas@docker-pc:~$ devtool reset linux-yocto

   This command removes your modifications and restores the original source code.

4. devtool build-image

   The devtool build-image command builds a complete target image that includes all recipes currently under modification.

   This is useful to verify integration of changes into a full root filesystem, not just individual components.

   Example: To build a new renesas-core-image-weston:

   .. code-block:: bash

       renesas@docker-pc:~$ devtool build-image renesas-core-image-weston

   This command generates an updated image that can be deployed to the target hardware.

**Known Issue**

When working with the **Yocto eSDK**, you might encounter the following warning:

.. code-block:: text

   WARNING: You are using a local hash equivalence server but have configured an sstate mirror.
   This will likely mean no sstate will match from the mirror.
   You may wish to disable the hash equivalence use (BB_HASHSERVE), or use a hash equivalence server alongside the sstate mirror.

   The ros2-control:do_package_qa sig is computed to be ea8f7e910d566912b932cbe602d93b93502064e293d1f4f1f569a67ee49f1c72,but the sig is locked to fd89fc1eb9961fd4ccddf16ea2ca1b73b5480ce1670ebb07a8075603bb645bc8 in SIGGEN_LOCKEDSIGS_t-cortexa55

.. note::

   This warning can be **safely ignored**.
   It does not affect the build process or output artifacts when using the Yocto eSDK.
