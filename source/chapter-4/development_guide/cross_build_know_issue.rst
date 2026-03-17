Known Issue: Non-Relocatable Sysroot CMake Paths
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In some cases, cross-builds fail because a package installed into the sysroot provides CMake export files that still contain hardcoded absolute paths such as ``/opt/ros/jazzy`` or ``/usr/lib/aarch64-linux-gnu/...``.

These paths may be valid on the original target filesystem, but they are not valid after the package is relocated into the cross-build sysroot.
As a result, CMake cannot resolve imported targets correctly during the build.

How to Recognize This Issue
"""""""""""""""""""""""""""

You may be hitting this issue if your build fails during ``cross-colcon-build`` with an error like:

.. code-block:: text

   CMake Error at /opt/rzv2h-sysroot/opt/ros/jazzy/lib/aarch64-linux-gnu/cmake/pinocchio/pinocchioTargets.cmake:146 (message):
     The imported target "pinocchio::pinocchio_pywrap_default" references the
     file
        "/opt/ros/jazzy/lib/python3.12/site-packages/pinocchio/pinocchio_pywrap_default.cpython-312-aarch64-linux-gnu.so.3.8.0"
     but this file does not exist.

Common signs include:

- the error points to a file inside ``/opt/rzv2h-sysroot/.../cmake/...``,
- the message says that an imported target references a file that does not exist, but you already verified that the file exists in the sysroot, and
- the missing file path starts with an absolute path such as ``/opt/ros/${ROS_DISTRO}`` or ``/usr/lib/...``, and
- the failing package is found by ``find_package()`` in CMake.

Why This Happens
""""""""""""""""

Some packages install CMake files such as:

- ``*Targets*.cmake``
- ``*Export.cmake``

These files may contain absolute paths generated at package build time.

In a relocated sysroot, those absolute paths may no longer point to the correct location.
Instead, the paths should usually resolve relative to ``${_IMPORT_PREFIX}``, which is how relocatable imported CMake targets are normally expressed.

What Happens Automatically
""""""""""""""""""""""""""

The ``sysroot-rosdep-install`` command automatically runs the ``sysroot-fix`` script.

That script:

- reads patch rules from ``/home/ubuntu/toolchains/sysroot-fix.yaml``,
- searches known CMake export files inside the sysroot, and
- replaces known hardcoded paths with relocatable paths.

However, if a package introduces a new broken path or uses a different CMake file name than expected, the existing rules may not match.
In that case, you need to add a new rule manually.

How to Fix It Yourself
""""""""""""""""""""""

Follow the steps below to identify and fix the issue.

Step 1: Identify the failing package and broken path
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Read the build error carefully and note:

- the package name,
- the CMake file where the error occurs, and
- the missing absolute path.

Example:

.. code-block:: text

   CMake Error at /opt/rzv2h-sysroot/opt/ros/jazzy/lib/aarch64-linux-gnu/cmake/pinocchio/pinocchioTargets.cmake:146 (message):
     The imported target "pinocchio::pinocchio_pywrap_default" references the file
     "/opt/ros/jazzy/lib/python3.12/site-packages/pinocchio/pinocchio_pywrap_default.cpython-312-aarch64-linux-gnu.so.3.8.0"
     but this file does not exist.

From this example, you should extract:

- package: ``pinocchio``
- CMake file: ``pinocchioTargets.cmake``
- broken path prefix: ``/opt/ros/jazzy``

Step 2: Search the sysroot for the hardcoded path
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Search the package CMake directory inside the sysroot to find which file contains the broken reference.

Example:

.. code-block:: bash

   grep -RIn "/opt/ros/${ROS_DISTRO}" ${V2H_SYSROOT}/opt/ros/${ROS_DISTRO}/lib/aarch64-linux-gnu/cmake/pinocchio

If the package is not under that path, search more broadly:

.. code-block:: bash

   grep -RIn "/opt/ros/${ROS_DISTRO}" ${V2H_SYSROOT}/opt/ros/${ROS_DISTRO}

or:

.. code-block:: bash

   grep -RIn "/usr/lib/aarch64-linux-gnu" ${V2H_SYSROOT}/usr/lib ${V2H_SYSROOT}/opt/ros/${ROS_DISTRO}

Look for the exact file that contains the unwanted absolute path.

Step 3: Decide the replacement path
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In most ROS 2 CMake export files, the correct replacement is one of the following:

- ``${_IMPORT_PREFIX}``
- ``${_IMPORT_PREFIX}/../../..``

Which one is correct depends on how the file refers to its installed artifacts.

A good rule of thumb is:

- if the original path starts with ``/opt/ros/${ROS_DISTRO}``, first try ``${_IMPORT_PREFIX}``
- if the original path starts with ``/usr/lib/...``, inspect nearby entries in the same file and match the existing style

If you are unsure, check surrounding entries in the CMake file to see how other imported targets are written.

Step 4: Add a new rule to ``sysroot-fix.yaml``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open the fix rule file:

.. code-block:: bash

   editor /home/ubuntu/toolchains/sysroot-fix.yaml

Add a new rule in this format:

.. code-block:: yaml

   <package_name>:
     - file: "<path relative to V2H_SYSROOT or absolute path with ${V2H_SYSROOT}>"
       find: "<string to replace>"
       replace: "<replacement string>"

Example:

.. code-block:: yaml

   some_package:
     - file: "${V2H_SYSROOT}/opt/ros/${ROS_DISTRO}/lib/aarch64-linux-gnu/cmake/some_package/some_packageTargets-none.cmake"
       find: "/opt/ros/${ROS_DISTRO}"
       replace: "${_IMPORT_PREFIX}"

Another real example:

.. code-block:: yaml

   hardware_interface:
     - file: "${V2H_SYSROOT}/opt/ros/${ROS_DISTRO}/share/hardware_interface/cmake/export_hardware_interfaceExport.cmake"
       find: "/opt/ros/${ROS_DISTRO}"
       replace: "${_IMPORT_PREFIX}"

.. note::

   The variables ``V2H_SYSROOT`` and ``ROS_DISTRO`` are already provided by the ``sysroot-fix`` script.
   You do not need to define them in the YAML file.

Step 5: Test the rule
~~~~~~~~~~~~~~~~~~~~~

Run the fix script in dry-run mode first:

.. code-block:: bash

   sudo -E /usr/local/bin/sysroot-fix <package_name> --dry-run

Example:

.. code-block:: bash

   sudo -E /usr/local/bin/sysroot-fix pinocchio --dry-run

If the rule matches, you should see output similar to:

.. code-block:: text

   [sysroot-fix] Package: pinocchio
     - PATCH: /opt/rzv2h-sysroot/opt/ros/jazzy/lib/aarch64-linux-gnu/cmake/pinocchio/pinocchioTargets-none.cmake
       find:    /opt/ros/jazzy
       replace: ${_IMPORT_PREFIX}

If you see:

.. code-block:: text

   INFO: file not found, skipping: ...

then the file path in the rule does not match the current sysroot layout.

If you see no patch output for the package, then:

- the file exists but the ``find`` string is not present, or
- the wrong file was selected.

In that case, inspect the actual file again and adjust the rule.

Step 6: Apply the fix
~~~~~~~~~~~~~~~~~~~~~

Once the dry run looks correct, apply the fix:

.. code-block:: bash

   sysroot-fix

Or rerun the usual command:

.. code-block:: bash

   sysroot-rosdep-install

Because ``sysroot-rosdep-install`` automatically invokes ``sysroot-fix``, rerunning it is usually the easiest option.

When a patch is applied, the script creates a backup file automatically.

Example:

.. code-block:: text

   backup: /opt/rzv2h-sysroot/.../some_file.cmake.bak.20260316-032342

Step 7: Rebuild and verify
~~~~~~~~~~~~~~~~~~~~~~~~~~

After applying the fix, rebuild the workspace:

.. code-block:: bash

   cross-colcon-build

If the build still fails, repeat the same process for the next missing path.
Some packages may require more than one rule.

Examples of Existing Rules
""""""""""""""""""""""""""

The following examples show real rules already used to fix known sysroot relocation problems:

.. code-block:: yaml

   pinocchio:
     - file: "${V2H_SYSROOT}/opt/ros/${ROS_DISTRO}/lib/aarch64-linux-gnu/cmake/pinocchio/pinocchioTargets-none.cmake"
       find: "/opt/ros/${ROS_DISTRO}"
       replace: "${_IMPORT_PREFIX}"

   hardware_interface:
     - file: "${V2H_SYSROOT}/opt/ros/${ROS_DISTRO}/share/hardware_interface/cmake/export_hardware_interfaceExport.cmake"
       find: "/opt/ros/${ROS_DISTRO}"
       replace: "${_IMPORT_PREFIX}"

   flann:
     - file: "${V2H_SYSROOT}/usr/lib/cmake/flann/flann-targets-none.cmake"
       find: "/usr/lib/aarch64-linux-gnu/liblz4.so"
       replace: "${_IMPORT_PREFIX}/lib/aarch64-linux-gnu/liblz4.so"

How to Interpret Script Output
""""""""""""""""""""""""""""""

Examples of useful output from ``sysroot-fix``:

- ``INFO: file not found, skipping: ...``

  - The file path in the rule does not exist in the current sysroot.
  - Check whether the package version uses a different file name.

- ``PATCH: ...``

  - The rule matched successfully and will be applied.
  - In normal mode, a backup file is created before writing changes.

- ``matched_rules=1/3``

  - One rule matched and was applied out of three total rules processed.
  - Unmatched rules are not necessarily errors.

When You Should Update the YAML File
""""""""""""""""""""""""""""""""""""

You should add or adjust a rule when:

- the build error clearly points to a CMake imported target with a broken absolute path,
- the current ``sysroot-fix`` output shows that no existing rule matched,
- the package version changed and now uses a different CMake export file name, or
- a new package introduces a similar relocation problem.

Summary
"""""""

If your cross-build fails with an imported CMake target error that references a missing absolute path inside ``/opt/ros/...`` or ``/usr/lib/...``, the sysroot likely contains a non-relocatable CMake export file.

To fix it yourself:

#. Read the error and identify the broken path,
#. Search the sysroot for the file containing that path,
#. Add a matching rule to ``/home/ubuntu/toolchains/sysroot-fix.yaml``,
#. Test it with ``--dry-run``,
#. Apply the fix, and
#. Rebuild the workspace.

Because ``sysroot-rosdep-install`` already runs ``sysroot-fix`` automatically, this process integrates directly into the normal cross-build workflow.