Installing a Jetson Xavier AGX
==============================

Step 1: Open up Dingo
-----------------------

Start by removing Dingo's side panels, yellow cover plates, and removing the center channel covers.

.. image:: images/center-channel.jpg
  :alt: Dingo's center channel

Step 2: Install the Xavier
--------------------------

Dingo includes pre-threaded mounting holes for the Jetson Xavier.  We recommend using 20mm standoffs to raise the board.

.. image:: images/computer-bay.jpg
  :alt: Dingo's computer bay

Connect the Jetson's power cable to the 12V output from Dingo's MCU.

.. image:: images/placeholder.png

The ethernet port on the Jetson will eventually need to be connected to the ethernet port of Dingo's MCU.  If you have
a network switch installed you can connect the MCU and the Jetson to the switch.  Otherwise leave the MCU disconnected
for now so that the Jetson's ethernet port can be used for downloading the software from the internet.


Step 3: Installing the Software
--------------------------------

`Download the latest version of Nvidia's SDK Manager <https://developer.nvidia.com/nvidia-sdk-manager>`_ on a PC running Ubuntu 18.04.  While that's downloading, put the Xavier into reovery mode by following these steps:

1.  Use the included USB cable to connect the Linux host computer to the front USB Type-C connector on the developer kit.
2.  Make sure the Xavier is powered off
3.  Connect a monitor, mouse, and keyboard to the Jetson.  (The mouse is optional, but recommended.  If you do not have an all-in-one mouse+keyboard you will need to use a small USB hub, as the Jetson Xavier only has a single USB port.)
4.  Press and hold the REC button
5.  Press the power button.

Install the SDK Manager by running the following commands:

.. code-block:: bash

    cd <folder where you downloaded SDK manager>
    sudo dpkg -i sdkmanager_<version>_amd64.deb

.. note::

    If your system is missing dependencies you may see error messages in the output of the ``dpkg`` command.  To resolve these, run ``sudo apt-get -f install``.

Login the the SDK Manager using your NVIDIA developer credentials.

.. image:: images/Xavier/Software/1.png

You do not need to setup your Host Machine unless you are planning on doing Cuda work on your local computer.  This can usually be disabled.  Under the Target Hardware, make sure to choose Xavier.

.. image:: images/Xavier/Software/2.png

Click Next and accept the terms.  Make sure the Download and Target directories are in locations that you have write-access to and that your hard drive has enough space for the files.

.. image:: images/Xavier/Software/3.png

Enter your sudo password

.. image:: images/Xavier/Software/4.png

The SDK manager will download the necessary files and install the image on the Jetson.

.. image:: images/Xavier/Software/5.png

During the install, make sure to plug a keyboard and monitor into the Jetson. On first boot, it will go through the usual Ubuntu setup steps.  Accept the Licenses

.. image:: images/Xavier/Software/6.png

Choose your language

.. image:: images/Xavier/Software/7.png

Choose your keyboard layout

.. image:: images/Xavier/Software/8.png

Set your location.

.. image:: images/Xavier/Software/9.png

Pick a hostname, username, and password for the machine.

.. note::

    For compatibility with older versions of the Jetson Xavier software, set the username and password to ``nvidia``.
    To standardize with other Clearpath Robotics products, set the username to ``administrator`` and the password to ``clearpath``.

.. image:: images/Xavier/Software/10.png

It will complete the installation and install the remaining standard packages.

.. image:: images/Xavier/Software/11.png

Once the OS is setup, you will be brought to the desktop.

.. image:: images/Xavier/Software/12.png

Open a terminal and run ``ifconfig`` to see the IP address it is using.  You will need to connect it to network through wireless or ethernet.

.. image:: images/Xavier/Software/13.png

Back in your host machine, it will be waiting to install the extra SDK components on your Jetson.  Enter the username, password, and IP address you found above.

.. image:: images/Xavier/Software/14.png

The install will connenct to the remote Jetson over the network.  It will continue the install by transferring the files and install them.

.. image:: images/Xavier/Software/15.png

You can check the terminal window to see the progress of individual commands.  This process will take a while, so it can be nice to verify that the process isn't stalled.

.. image:: images/Xavier/Software/16.png

Once the process is done, you can click FINISH to close the window.

.. image:: images/Xavier/Software/17.png

Once the OS has been written to the Xavier, log into it and run the following commands to configure it for use with Dingo:

.. code-block:: bash

    wget -c https://raw.githubusercontent.com/clearpathrobotics/ros_computer_setup/main/install.sh && bash install.sh

.. note::

    If ``curl`` is not installed on your Jetson by default you can install it by running ``sudo apt-get install curl``

.. image:: images/Xavier/Software/18.png

These commands will download and install ROS along with the necessary APT packages to get Dingo up and running.  Depending on your network speed it may take a long time for everything to install.  Reboot the Xavier after these commands are done to complete the configuration.

When the Jetson starts up again, it should be connected to the Dingo. To see that the Dingo is connected by opening a terminal and executing "rostopic echo /status". You should see a 1hz message containing the Dingo's diagnostic information.

Your Jetson Xavier should now be configured to operate as the Dingo's main PC.

Enabling Automatic Power-On
-----------------------------------

We recommend configuring the Jetson to automatically boot when it receives power. To enable automatic power-on, connect pins 5 and 6 of the J508 header on the Jetson:

.. image::
  images/xavier-auto-power-on.png
  :alt: Xavier J508 header
