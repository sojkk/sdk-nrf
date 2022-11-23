.. _nPM1300 BUCK:

nPM1300 BUCK
################

Overview
********
This sample demonstrates the usage of nPM1300 PMIC BUCKs.
Connect the NordicSemiconductor nRF5340 DK (nrf5340dk_nrf5340) to host device and open COM port. 
For testing connect voltage multimeter to VOUT1 (output voltage pin of BUCK1) and VOUT2 (output voltage pin of BUCK2).
In main.c file select your test case to be executed. 
You can also modify expected output voltages and selected GPIOs to control the system.
But be careful, BUCK_INSTANCE_2 is the main source of nRF5340 DK so the voltage of BUCK2 should be at least 2.0 V.

Testcases
---------

Test set output BUCKs voltage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In line 30 set SELECTED_TESTCASE to TESTCASE_SET_BUCK_VOLTAGE.
In lines 230 and 231 you can select the output voltage for each BUCK converter.

Test output voltage range
~~~~~~~~~~~~~~~~~~~~~~~~~

In line 30 set SELECTED_TESTCASE to TESTCASE_OUTPUT_VOLTAGE.
Now only test_output_voltage() function is called.
In line 121 you can select the start test voltage for BUCK1. 
The output voltage on BUCK1 will decrease from the selected start voltage to the maximum voltage (3.3 V).

The same is for BUCK2 (line 124). But here start output voltage can be a minimum of 2.0 V, because of DK board requirements.

Test retention voltage using GPIO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In line 30 set SELECTED_TESTCASE to TESTCASE_RETENTION_VOLTAGE.
Now only test_retention_voltage() function is called.
In line 149 select GPIO to be used as a retention input selector.
The same GPIO should be connected to BUCK1 in line 170.
Select normal and retention voltage in lines 154 and 159.
When you change the input state of the selected GPIO you should see voltage changes in the BUCK1 output. 

Test enable BUCK using GPIO
~~~~~~~~~~~~~~~~~~~~~~~~~~~

In line 30 set SELECTED_TESTCASE to TESTCASE_ENABLE_VIA_PINS.
Now only test_enable_bucks() function is called.
In line 191 select GPIO to be used as a enable input selector.
The same GPIO should be connected to BUCK1 in line 199.
Select output voltage in line 196.
When you change the input state of the selected GPIO you should see voltage changes in the BUCK1 output - from 0 V to previously selected voltage. 


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
