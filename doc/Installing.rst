Installing Dingo Software
=============================

.. note::

  The physical Dingo robot comes pre-configured with ROS and the necessary Dingo packages already installed; therefore, you will only need to follow the instructions below if you are re-installing software on the Dingo.

There are three ways to install Dingo software on the physical Dingo robot.

The first two ways are using the Clearpath Robotics ISO image, and using Debian (``.deb``) packages. These two ways are covered in this section.

The third way is installing from source by directly cloning Clearpath Robotics Github repositories and building them in your ROS (``catkin``) workspace; however, this way is not recommended and will not be covered in this section.

Installing with ISO Image
--------------------------

.. Warning::

  Installing with the Clearpath Robotics ISO image will completely wipe data on the Dingo's computer, since the ISO image will install Ubuntu 20.04 (Focal), ROS Noetic, and Dingo-specific packages.

.. note::
  The Clearpath Robotics ISO image only targets Intel-family computers (``amd64`` architecture). If your Dingo runs on an Nvidia Jetson computer, see :doc:`Jetson Xavier AGX <JetsonXavier>` or :doc:`Jetson Nano <JetsonNano>`.

If you are installing Dingo software on a physical Dingo robot through the Clearpath Robotics ISO image, you will first need a USB drive of at least 2GB to create the installation media, an ethernet cable, a monitor, and a keyboard.

You can download the Ubuntu 20.04 (Focal) ROS Noetic ISO image from `here <https://packages.clearpathrobotics.com/stable/images/latest/noetic-focal/>`_.

On a separate computer:

1. Download the ``.iso`` file from the link above.

2. Insert the USB drive.

3. Write the downloaded ``.iso`` file to the USB drive using a software such as ``Rufus``, ``Etcher``, or ``UNetbootin``. This will erase all data already on the USB drive, so make sure you have backed up anything important!

On the physical Dingo robot's computer:

1. Ensure that it is turned off.

2. Connect it to the internet via the ethernet cable.

3. Insert the newly formatted USB drive.

4. Turn it on and choose to begin the installation process. The installer should run automatically. 

.. note::

  You may need to configure the computer's BIOS to prioritize booting from the USB drive. On most common motherboards, pressing ``delete`` during the initial startup will open the BIOS for configuration.

5. Step through any prompts that come up. Make sure you select the installation option corresponding to the Dingo, and give the Dingo an appropriate hostname. The computer will turn off automatically when the installation completes.

6. Once the computer turns off, remove the USB drive and turn on the computer. It will now be running your fresh installation of Ubuntu 20.04 (Focal) with ROS Noetic, as well as your Dingo-specific packages.

7. Install Dingo's ``robot_upstart`` job and configure the Dingo bringup service, so that ROS will launch each time the robot starts. In terminal, run:

.. code-block:: bash

  rosrun dingo_bringup install
  sudo systemctl daemon-reload

8. Finally, start ROS for the first time. In terminal, run:

.. code-block:: bash
  
  sudo systemctl start ros

Installing with Debian Packages
--------------------------------

If you are installing Dingo software on a physical Dingo robot through Debian packages, you will first need to ensure that the Dingo robot's computer is running Ubuntu 20.04 (Focal) and ROS Noetic.

**Add Clearpath Debian Package Repository**

Before you can install the Dingo packages, you need to configure Ubuntu's APT package manager to
add Clearpath's package server:

1. Install the authentication key for the packages.clearpathrobotics.com repository. In terminal, run:

.. code-block:: bash

    wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -

2. Add the debian sources for the repository. In terminal, run:

.. code-block:: bash

    sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

3. Update your computer's package cache. In terminal, run:

.. code-block:: bash

    sudo apt-get update

**Installing Debian Packages**

After the Dingo's computer is configured to use Clearpath's debian package repository, you can install the Dingo packages. 

1. On a physical Dingo robot, you should only need the Dingo robot packages. In terminal, run:

.. code-block :: bash

    sudo apt-get install ros-noetic-dingo-robot

2. Install Dingo's ``robot_upstart`` job and configure the Dingo bringup service, so that ROS will launch each time the robot starts. In terminal, run:

.. code-block:: bash

  rosrun dingo_bringup install
  sudo systemctl daemon-reload

3. Finally, start ROS for the first time. In terminal, run:

.. code-block:: bash
  
  sudo systemctl start ros

Installing Desktop Software
----------------------------

It is useful to install Dingo's software on your computer for the purpose of interfacing with the physical Dingo robot and/or to run simulations of Dingo.

If you are installing Dingo's software on your computer, you will first need to ensure that your computer is running Ubuntu 20.04 (Focal) and ROS Noetic.

1. On your computer, you should only need the Dingo desktop packages. In terminal, run:

.. code-block :: bash

  sudo apt-get install ros-noetic-dingo-desktop ros-noetic-dingo-simulator