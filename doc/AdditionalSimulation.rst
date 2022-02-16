Additional Simulation Worlds
================================

In addtion to the default ``dingo_world.launch`` file, ``dingo_gazebo`` contains two additional launch files:
``empty_world.launch``, which spawns Dingo in a featureless, infinite plane and ``spawn_dingo.launch`` which is
intended to be included in any custom world to add a Dingo simulation to it.

.. image:: images/dingo_empty_world.png
  :alt: Dingo in the Empty World environment

To add a Dingo to any of your own worlds, simply include the ``spawn_dingo.launch`` file in your own world's launch:

.. code-block:: xml

  <include file="$(find dingo_gazebo)/launch/spawn_dingo.launch">
    <!-- Optionally configure the spawn position -->
    <arg name="x" value="$(arg x)"/>
    <arg name="y" value="$(arg y)"/>
    <arg name="z" value="$(arg z)"/>
    <arg name="yaw" value="$(arg yaw)"/>
  </include>

Finally, Clearpath provides an additional suite of simulation environments that can be downloaded separately and used
with Dingo, as described below.

Clearpath Gazebo Worlds
------------------------

The Clearpath Gazebo Worlds collection contains 4 different simulation worlds, representative of different
environments our robots are designed to operate in:

* Inspection World: a hilly outdoor world with water and a cave
* Agriculture World: a flat outdoor world with a barn, fences, and solar farm
* Office World: a flat indoor world with enclosed rooms and furniture
* Construction World: office world, under construction with small piles of debris and partial walls

Dingo is supported in the Office and Construction worlds.

Installation
---------------

To download the Clearpath Gazebo Worlds, clone the repository from github into the same workspace as your Dingo:

.. code-block:: bash

  cd ~/catkin_ws/src
  git clone https://github.com/clearpathrobotics/cpr_gazebo.git

Before you can build the package, make sure to install dependencies.  Because Clearpath Gazebo Worlds depends on
all of our robots' simulation packages, and some of these are currently only available as source code, installing
dependencies with ``rosdep install --from-paths [...]`` will likely fail.

To simulate Dingo in the Office and Construction worlds the only additional dependency is the ``gazebo_ros`` package.

Once the dependencies are installed, you can build the package:

.. code-block:: bash

  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash

Running the Office Simulation
--------------------------------

.. image:: images/dingo_office_world.png
  :alt: Dingo in the Office World

To launch Office World with a Dingo, run the following command:

.. code-block:: bash

  roslaunch cpr_office_gazebo office_world.launch platform:=dingo

You should see Dingo spawn in the office world, as pictured.  You can see the complete layout of the office below:

.. image:: images/office_world.png
  :alt: The layout of Office World

To add sensors to Dingo, use the environment variables described in :doc:`DescriptionPackage`.  For example, to simulate
Dingo with a Sick LMS-1xx lidar, run:

.. code-block:: bash

  export DINGO_LASER=1
  roslaunch cpr_office_gazebo office_world.launch platform:=dingo

You will see Dingo spawn with a lidar sensor mounted to it, which can be used for navigation as described in
:doc:`Simulating`.

.. image:: images/dingo_office_laser.png
  :alt: Dingo in Office World with a lidar sensor

Running the Construction Simulation
--------------------------------------

.. image:: images/dingo_construction_world.png
  :alt: Dingo in the Construction World

To launch Construction World with a Dingo, run the following command:

.. code-block:: bash

  roslaunch cpr_office_gazebo office_construction_world.launch platform:=dingo

You should see Dingo spawn in the construction world, as pictured.  You can see the complete layout of the office below:

.. image:: images/construction_world.png
  :alt: The layout of Construction World

To add sensors to Dingo, use the environment variables described in :doc:`DescriptionPackage`.  For example, to simulate
Dingo with a Sick LMS-1xx lidar, run:

.. code-block:: bash

  export DINGO_LASER=1
  roslaunch cpr_office_gazebo office_construction_world.launch platform:=dingo

You will see Dingo spawn with a lidar sensor mounted to it, which can be used for navigation as described in
:doc:`Simulating`.
