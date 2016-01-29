#!/usr/bin/python
# coding: utf-8

from time   import time, sleep

from ev3dev.auto import *

left_motor = LargeMotor(OUTPUT_B);  assert left_motor.connected
right_motor = LargeMotor(OUTPUT_C); assert right_motor.connected
col= ColorSensor();		 assert col.connected
col.mode = 'COL-REFLECT'

def run():
  left_motor.run_direct(duty_cycle_sp=30)
  right_motor.run_direct(duty_cycle_sp=30)
  max_ref = 0
  min_ref = 100
  end_time = time() + 5
  while time() < end_time:
    read = col.value()
    if max_ref < read:
      max_ref = read
    if min_ref > read:
      min_ref = read
  left_motor.stop()
  right_motor.stop()
  print 'Max: ' + str(max_ref)
  print 'Min: ' + str(min_ref)
  sleep(1)

run()
