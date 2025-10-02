.. zephyr:board:: arduino_uno_q

Overview
********

The Arduino UNO Q is a development board featuring a Qualcomm QRB2210
processor (Quad core ARM Cortex-A53) and an STM32U585 microcontroller.
The board is designed around the Arduino form factor and is compatible
with traditional Arduino shields and accessories.
This port targets the STM32U585 microcontroller on the board.

Hardware
********

- Qualcomm QRB2210 Processor (Quad core ARM Cortex-A53)
- STM32U585 Microcontroller (ARM Cortex-M33 at 160 MHz)
- 2 Mbyte of Flash memory and 786 Kbytes of RAM
- 2 RGB user LEDs
- One 13x8 LED Matrix
- Internal UART and SPI busses connected to the QRB2210
- Built-in CMSIS-DAP debug adapter (through QRB2210)

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and debugging
*************************

.. zephyr:board-supported-runners::

Debug adapter
=============

The QRB2210 microprocessor can act as an SWD debug adapter for the STM32U585.
This is supported by the onboard openocd.
Flashing is not yet integrated with the ``west flash`` command, while debugging is supported.

Debugging
=========

Debugging can be done with ``west debug`` command.
The following command is debugging the :zephyr:code-sample:`blinky` application.

.. code-block:: console

   adb forward tcp:3333 tcp:3333 && adb shell arduino-debug
   # in a different shell
   west build -b arduino_uno_q samples/basic/blinky
   west debug -r openocd

Restoring Arduino Bootloader
============================

If you corrupt the Arduino bootloader, you can restore it with the following command.

.. code-block:: console

   adb shell arduino-cli burn-bootloader -b arduino:zephyr:unoq -P jlink
