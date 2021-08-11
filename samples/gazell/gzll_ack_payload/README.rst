.. _gzll_ack_payload:

Gazell ACK Payload
##################

.. contents::
   :local:
   :depth: 2

Overview
********

The sample shows basic Gazell communication and demonstrates how to send payloads and acknowledgments.
It consists of two applications, one running on the device and one running on the host.

Device
======

Device sends a packet and adds a new packet to the TX queue every time it receives an ACK from Host.
Before adding a packet to the TX queue, the contents of the buttons is copied to the first payload byte (byte 0).
When an ACK is received, the contents of the first payload byte of the ACK are output to LEDs.

Host
====

This example listens for a packet and sends an ACK when a packet is received.
The contents of the first payload byte of the received packet is output to LEDs.
The contents of buttons are sent in the first payload byte (byte 0) of the ACK packet.

Requirements
************

The sample supports the following development kits:

.. table-from-rows:: /includes/sample_board_rows.txt
   :header: heading
   :rows: nrf52840dk_nrf52840, nrf52833dk_nrf52833, nrf52dk_nrf52832

You can use any two of the development kits listed above and mix different development kits.

User interface
**************

LED 1-4:
   Indicate that packets are received.
   A LED is turned off when the corresponding button is pressed on the other kit.

Button 1-4:
   The button pressed state bitmask is sent to the other kit.
   A button pressed is sent as 0 and vice versa.

Building and running
********************

The Device sample can be found under :file:`samples/gazell/gzll_ack_payload/device` in the |NCS| folder structure.
The Host sample can be found under :file:`samples/gazell/gzll_ack_payload/host` in the |NCS| folder structure.

See :ref:`gs_programming` for information about how to build and program the application.

Testing
=======

After programming the Device sample on one of the development kits and the Host sample on the other kit, test them by performing the following steps:

1. Power on both kits.
#. Observe that all the LEDs light up on both kits.
#. Press **Button 1** for Device sample.
   Observe that Host sample turns off **LED 1** on the other kit.
#. Press **Button 2** for Host sample.
   Observe that Device sample turns off **LED 2** on the other kit.
#. Optionally, connect to the kits with a terminal emulator (for example, PuTTY).
   See :ref:`putty` for the required settings.
#. Observe the logging output for both kits.

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`dk_buttons_and_leds_readme`

In addition, it uses the following :ref:`nrfxlib` libraries:

* :ref:`nrfxlib:gzll`

In addition, it uses the following Zephyr libraries:

* ``include/zephyr/types.h``
* :ref:`zephyr:logging_api`
* :ref:`zephyr:kernel_api`:

  * ``include/kernel.h``
  * ``include/irq.h``
