#!/usr/bin/python
# coding: utf-8

from time   import sleep

from ev3dev.auto import *

left_motor = LargeMotor(OUTPUT_B);  assert left_motor.connected
right_motor = LargeMotor(OUTPUT_C); assert right_motor.connected
col= ColorSensor();		 assert col.connected
col.mode = 'COL-REFLECT'

def run():
  left_motor.run_timed(duty_cycle_sp=10, time_sp=4000)
