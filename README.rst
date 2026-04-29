BrachioGraph - the cheapest, simplest possible pen-plotter
==========================================================

`BrachioGraph <https://www.brachiograph.art/>`_ is an ultra-cheap (total cost of materials: ~€14) plotter that can be built with minimal skills.

At its heart is a Raspberry Pi Zero and some relatively simple custom software, driving three servo motors.

The mechanical hardware can be built from nothing but two sticks, a pen or pencil and some glue. No tools are required.

Almost everything required can be found in a desk or kitchen drawer. The entire device can be built with no special skills in about an hour.


.. image:: docs/images/readme_combined_image.png
    :width: 100%


Documentation
-------------

`Complete documentation for the project, with detailed instructions on how to build and use it <https://www.brachiograph.art/>`_


Arch Linux quick start
----------------------

- Create the environment: ``./bgraph.sh setup``
- Run the backend in virtual mode: ``./bgraph.sh backend-virtual``
- Run the backend against the ESP32: ``./bgraph.sh backend``
- Smoke-test the ESP32 serial link: ``./bgraph.sh esp-test``
- Start or restart the backend in the background: ``./bgraph.sh start`` / ``./bgraph.sh restart``
