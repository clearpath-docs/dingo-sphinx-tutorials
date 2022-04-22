Dingo Tests
============

Dingo robots come preinstalled with a set of test scripts as part of the ``dingo_tests`` ROS package, which can be run to verify robot functionality at the component and system levels. 

If your Dingo does not have the ``dingo_tests`` ROS package installed already, you can manually install it by opening terminal and running:

.. code-block:: bash

  sudo apt-get install ros-noetic-dingo-tests

ROS Tests
----------

The ``ros_tests`` script exposes a set of useful tests at the ROS-level, via ROS topics, to verify the functionality of core features. In addition, these tests together serve as a useful robot-level diagnostic tool for identifying the root cause of problems, or at the very least, narrowing down on where the root cause(s) may be. 

To run the ``ros_tests`` script on a Dingo robot, open terminal and run:

.. code-block:: bash

  rosrun dingo_tests ros_tests

The **Lighting Test** checks that all 4 lights are working properly. It publishes lighting commands to the ``/cmd_lights`` ROS topic and checks that the lights change to the expected colours.

The **E-Stop Test** checks that the E-Stop is working properly. It subscribes to the ``/mcu/status`` ROS topic and checks that when the E-Stop is manually engaged by the user, the E-Stop state is correctly reported on the ``/mcu/status`` ROS topic, and the lights flash red.

The **ADC Test** subscribes to the ``/mcu/status`` ROS topic and checks that the voltage and current values across the internal hardware are within expected tolerances.

The **Rotate Test** subscribes to the ``/imu/data`` and ``/odometry/filtered`` ROS topics, and publishes drive commands to the ``/cmd_vel`` ROS topic. This test attempts to rotate the Dingo for 2 full revolutions using the IMU's Gyroscope's angular velocity measurements on the ``/imu/data`` ROS topic. In addition, it also tracks the angular velocity estimations from the ``robot_localization`` EKF's odometry on the ``/odometry/filtered`` ROS topic for comparison.

The **Drive Test** subscribes to the ``/feedback`` and ``/dingo_velocity_controller/odom`` ROS topics, and publishes drive commands to the ``/cmd_vel`` ROS topic. This test attempts to drive the Dingo forward 1 metre based on the linear displacement estimations from the encoder-fused odometry on the ``/dingo_velocity_controller/odom`` ROS topic. In addition, it also tracks each individual encoder's linear displacement measurements on the ``/feedback`` ROS topic for comparison.

The **Cooling Test** is an optional test that only applies to Dingo's with an external fan connected to the MCU. This test publishes to the ``/mcu/cmd_fans`` ROS topic to verify that the fan spins at the expected speeds.

CAN Bus Test
-------------

The ``check_can_bus_interface`` is useful in verifying the Puma motors and encoders are sending and receiving data between the Dingo's PC, via the CAN bus interface. 

This script verifies that the ``can0`` interface is detected and activated, then proceeds to check the output of ``candump`` to verify that good CAN packets are being transmitted. Based on the Dingo configuration, either Dingo-D or Dingo-O, this script will know to check for good CAN packets from 2 or 4 encoders, respectively.

To run the ``check_can_bus_interface`` script on a Dingo robot, open terminal and run:

.. code-block:: bash

  rosrun dingo_tests check_can_bus_interface