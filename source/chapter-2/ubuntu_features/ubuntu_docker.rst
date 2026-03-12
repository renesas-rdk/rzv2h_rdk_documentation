Docker with Ubuntu on RZ/V2H RDK
--------------------------------

Docker is available on Ubuntu for container-based development and deployment.

Main points:

- Install and run Docker on Ubuntu
- Build and run containers for applications
- Useful for packaging development environments
- Helps with reproducible workflows

This guide explains how to install and verify Docker on the RZ/V2H RDK running Ubuntu.

Prerequisites
^^^^^^^^^^^^^

- RZ/V2H RDK running Ubuntu
- Internet connection
- User account with ``sudo`` privileges

Install Docker
^^^^^^^^^^^^^^

Follow the steps below to install Docker on the RZ/V2H RDK:

.. code-block:: bash

   sudo apt update

   # Make sure you can access the internet
   ping bing.com

   # Install Docker
   curl -fsSL https://get.docker.com | sudo sh

   # Add user to docker group
   sudo usermod -aG docker $USER

.. note::

   After adding your user to the ``docker`` group, log out and log back in for the group change to take effect.

Verify Docker Installation
^^^^^^^^^^^^^^^^^^^^^^^^^^

Check that Docker is installed correctly:

.. code-block:: bash

   docker --version
   docker run hello-world

If the installation is successful, the ``hello-world`` container runs and prints a confirmation message.

Notes
^^^^^

- Ensure the board has a stable internet connection before starting the installation.
- The ``get.docker.com`` installation script installs Docker automatically with recommended settings.
- If ``docker run hello-world`` fails after installation, log out and log back in, then try again.

Troubleshooting
^^^^^^^^^^^^^^^

- If ``apt update`` fails, verify network connectivity.
- If ``ping bing.com`` fails, check DNS and internet access.
- If you get a permission error when running Docker commands, confirm that your user was added to the ``docker`` group and that you have logged in again.