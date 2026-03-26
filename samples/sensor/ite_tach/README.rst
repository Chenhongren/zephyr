.. zephyr:code-sample:: ite_tach
   :name: ITE tachometer sensor
   :relevant-api: sensor_interface

   Validate tachometer RPM by driving PWM signal.

Overview
********

This sample demonstrates how to generate PWM signal and measure the
resulting frequency using tachometer (fan) sensor.

The application:

- Configure PWM signal with varying periods (1 ms to 4 ms)
- Use 50% duty cycle
- Read RPM from tachometer sensor and convert RPM to frequency
- Compare measured frequency against expected PWM frequency

This is useful for validating PWM output and tachometer input using
loopback connection.

Requirements
************

- Board with:

  - PWM driver support (``drivers/pwm/pwm_ite_it51xxx.c``)
  - Tachometer (sensor) driver support (``drivers/sensor/ite/ite_tach_it51xxx/tach_ite_it51xxx.c``)

- Devicetree aliases must be defined:

  - ``pwm_test``: PWM device
  - ``tach_test``: Tachometer sensor. The tachometer node must define ``pulses-per-round`` property.

- Hardware setup: Connect PWM output pin directly to tachometer input pin. If the connection is missing or incorrect, the uart console will report ``<err> main: tach pin not wired to pwm pin? (tach rpm 0)`` when the measured RPM is zero.


Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/ite_tach
   :board: it51xxx_evb/it51526aw
   :goals: build
   :compact:

Sample Output
=============

.. code-block:: console

   *** Booting Zephyr OS build v4.3.0-8125-g20e50732dfd8 ***
   [00:00:00.052,581] <inf> main: rpm 29947.00, freq 998.23 (exp 1000)
   [00:00:00.103,515] <inf> main: rpm 14973.00, freq 499.10 (exp 500)
   [00:00:00.154,235] <inf> main: rpm 9982.00, freq 332.73 (exp 333)
   [00:00:00.206,481] <inf> main: rpm 7486.00, freq 249.53 (exp 250)
