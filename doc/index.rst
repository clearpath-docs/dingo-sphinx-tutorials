Dingo Tutorials
======================

.. image:: images/dingo_banner.png
    :alt: Dingo Robot

This package supplies Sphinx-based tutorial content to assist you with setting up and operating your Dingo_
mobile robot. The tutorials topics are listed in the left column, and presented in the suggested reading order.

.. _Dingo: https://clearpathrobotics.com/dingo-indoor-mobile-robot/

.. Warning::

  These tutorials assume that you are comfortable working with ROS.  We recommend starting with our
  `ROS tutorial <./../ros>`_ if you are not familiar with ROS already.

.. Note::

  Dingo is still under active development.  As such these tutorials may be incomplete in certain areas and are subject to change.
  We apologize for any inconvenience this may cause.

.. Note::

  Dingo currently runs on ROS Melodic on Ubuntu 18.04 "Bionic".  Other versions of ROS and Ubuntu have not been tested yet.

:doc:`Simulation <simulation>` is a logical place for most users to start, as this is universally applicable;
understanding how to effectively operate Dingo in simulation is valuable whether you are in the testing
phase with software you intend to ultimately deploy on a real Dingo, or you do not have one and are
simply exploring the platform's capabilities.

:doc:`Navigation <navigation>` is a follow-on to what is learned in the simulation tutorial, as navigation and
map-making may be run in the simulated environment. However, this content is applicable to both the simulator
and the real platform, if equipped with a laser scanner.

Additional topics coming soon!


.. toctree::
    :titlesonly:
    :hidden:
    :caption: Getting Started

    Overview <self>
    simulation
    software_setup
    navigation
    controllers
    network
    driving
    human_machine_interface

.. toctree::
    :titlesonly:
    :maxdepth: 0
    :caption: Hardware Customization

    computer_installation
    payloads
    manipulation

.. toctree::
    :titlesonly:
    :maxdepth: 0
    :caption: NVIDIA Jetson

    jetson_nano
    jetson_xavier

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Testing Dingo

    DingoTests
    
.. toctree::
    :titlesonly:
    :hidden:
    :caption: Dingo Packages

    description
    additional_sim_worlds
