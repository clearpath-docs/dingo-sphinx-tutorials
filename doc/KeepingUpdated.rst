Keeping Dingo Updated
==========================

.. note:: If you are upgrading your Dingo from an older version of ROS, please refer to `our upgrade instructions here <https://clearpathrobotics.com/assets/guides/noetic/melodic-to-noetic/index.html>`_.

Dingo is always being improved, both its own software and the many community ROS packages upon which it depends! You can use the apt package management system to receive new versions all software running on the Dingo.

Getting New Packages
--------------------

Each Dingo leaves the factory already configured to pull packages from http://packages.ros.org as well as http://packages.clearpathrobotics.com. To update your package and download new package versions make sure that Dingo is connected to the internet and run the following commands:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get dist-upgrade

If you see any errors, please `get in touch`_ and we'll see if we can get you sorted out.

.. _get in touch: https://support.clearpathrobotics.com/hc/en-us/requests/new