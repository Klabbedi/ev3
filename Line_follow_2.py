#!/usr/bin/python
# coding: utf-8

from time import sleep

from ev3dev.auto import *

# ------Input--------
print 'Setting input values'
power = 60
target = 55
kp = float(0.65) # Start value 1
kd = 1           # Start value 0
ki = float(0.02) # Start value 0
direction = -1
minRef = 41
maxRef = 63
# -------------------

# Connect two large motors on output ports B and C and check that
# the device is connected using the 'connected' property.
print 'Connecting motors'
left_motor = LargeMotor(OUTPUT_B);  assert left_motor.connected
right_motor = LargeMotor(OUTPUT_C); assert right_motor.connected
# One left and one right motor should be connected

# Connect color and touch sensors and check that they
# are connected.
print 'Connecting sensors'
#ir = InfraredSensor(); 	assert ir.connected
ts = TouchSensor();    	assert ts.connected
col= ColorSensor(); 	assert col.connected

# Change color sensor mode
print 'Setting color sensor mode'
col.mode = 'COL-REFLECT'

# Adding button so it would be possible to break the loop using
# one of the buttons on the brick
print 'Adding button'
btn = Button()

def steering2(course, power):
	if course >= 0:
		if course > 100:
			pr = 0
			pl = power
		else:	
			pl = power
			pr = power - ((power * course) / 100)
	else:
		if course < -100:
			pl = 0
			pr = power
		else:
			pr = power
			pl = power + ((power * course) / 100)
	return (int(pl), int(pr))

def run(power, target, kp, kd, ki, direction, minRef, maxRef):
	"""
	Description...
	"""
	lastError = 0
	error = 0
	integral = 0
	left_motor.run_direct()
	right_motor.run_direct()
	lap = 1
	while not btn.any() :
		if ts.value():
			print 'Breaking loop'
			break
		refRead = col.value()
		error = target - (100 * ( refRead - minRef ) / ( maxRef - minRef ))
		derivative = error - lastError
		lastError = error
		integral = float(0.5) * integral + error
		course = (kp * error + kd * derivative +ki * integral) * direction
		for (motor, pow) in zip((left_motor, right_motor), steering2(course, power)):
			motor.duty_cycle_sp = pow
		lap = lap + 1
		sleep(0.01)

run(power, target, kp, kd, ki, direction, minRef, maxRef)

# Stop the motors before exiting.
print 'Stopping motors'
left_motor.stop()
right_motor.stop()
