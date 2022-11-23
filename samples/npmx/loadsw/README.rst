.. _nPM1300 LOADSW:

nPM1300 LOADSW
################

Overview
********
This sample demonstrates the usage of nPM1300 PMIC LOADSWs.
Connect the NordicSemiconductor nRF5340 DK (nrf5340dk_nrf5340) to host device and open COM port. 
For testing, connect voltage multimeter to LS1OUT (output voltage pin of LOADSW1) and LS2OUT (output voltage pin of LOADSW2) and observe the indications.
Based on following GoBoard configuration, input of LOADSW1 is connected to output of BUCK1 (default enabled with 1.0 V). Input of LOADSW2 is connected to VSYS (+5.0V from USB).
In main.c file select your test case to be executed. 

Testcases
---------

Test enable and disable load switch with software control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In line 28 set SELECTED_TESTCASE to TESTCASE_SW_ENABLE.
Now only test_sw_enable() function is called.
In an infinite loop, switches are enabled and disabled alternately.

Test enable and disable load switch with GPIO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In line 28 set SELECTED_TESTCASE to TESTCASE_GPIO_ENABLE.
Now only test_gpio_enable() function is called.
In lines 77 and 81 select GPIOs to be used as inputs control.
Connect GPIO to load switch 1 in line 86.
Connect GPIO to load switch 2 in line 91.
When you change the input states of the selected GPIOs you should see voltage changes in the load switches 1 and 2 outputs. 

Requirements
************
This sample has been tested on the Nordic NordicSemiconductor nRF5340 DK (nrf5340dk_nrf5340) board with nPM1300 GoBoard.

.. list-table:: Connection table
   :widths: 25 25
   :header-rows: 1

   * - nrf5340dk
     - nPM1300 GoBoard
   * - P1.02
     - SDA
   * - P1.03
     - SCL
   * - VDD nRF (P20)
     - VDDIO (P3)
   * - External supply (P21)
     - VOUT2, GND (P6)

On GoBoard:
 - On P12, PVDD should be connected with VSYS
 - VBUS should be connected with VBUS USBC
 - On P6 connect with jumpers: (VOUT1 - LS1IN), (LS2IN - VSYS), (LS1OUT - ), (LS2OUT - )
 - On P8 connect with jumpers: (LED0, LED1, LED2), (VLED - VSYS)
 - On P5 connect with jumpers: (VSET1, VSET2, SDA, SCL)

On nrf5340dk:
 - Connect USB to host device (J2)
 - Switch nRF power source (SW9) to VDD
 - Switch VEXT -> nFR (SW10) to ON
