#!/usr/bin/python
# coding: utf-8

from time import sleep

from ev3dev.auto import *

# ------Input--------
print 'Setting input values'
power = 40
target = 55
kp = float(0.65) # Start value 1
kd = 3           # Start value 0
ki = float(0.02) # Start value 0
direction = -1
minRef = 44
maxRef = 60
# -------------------

# Connect two large motors on output ports B and C and check that
# the device is connected using the 'connected' property.
print 'Connecting motors'
left_motor = LargeMotor(OUTPUT_B);  assert left_motor.connected
right_motor = LargeMotor(OUTPUT_C); assert right_motor.connected
# One left and one right motor should be connected

# Connect infrared, color and touch sensors and check that they
# are connected.
print 'Connecting sensors'
ir = InfraredSensor(); 	assert ir.connected
ts = TouchSensor();    	assert ts.connected
col= ColorSensor(); 	assert col.connected

# Change color sensor mode
print 'Setting color sensor mode'
col.mode = 'COL-REFLECT'

# Adding button so it would be possible to break the loop using
# one of the buttons on the brick
print 'Adding button'
btn = Button()

def steering(course, power):
	"""
	Computes how fast each motor in a pair should turn to achieve the
	specified steering.
	Input:
		direction [-100, 100]:
		* -100 means turn left as fast as possible,
		*  0   means drive in a straight line, and
		*  100 means turn right as fast as possible.
	power: the power that should be applied to the outmost motor (the one
		rotating faster). The power of the other motor will be computed
		automatically.
	Output:
		a tuple of power values for a pair of motors.
	Example:
		for (motor, power) in zip((left_motor, right_motor), steering(50, 90)):
			motor.run_forever(speed_sp=power)
	"""

	pl = power
	pr = power
	s = (50 - abs(float(course))) / 50

	if course >= 0:
		pr *= s
		if course > 100:
			pr = - power
	else:
		pl *= s
		if course < -100:
			pl = - power

	return (int(pl), int(pr))
	
def steering2(course, power):
	if course >= 0:
		if course > 100:
			pr = 0
		else:	
			pl = power
			pr = power - ((power * course) / 100)
	else:
		if course < -100:
			pl = 0
		else:
			pr = power
			pl = power + ((power * course) / 100)
	return (int(pl), int(pr))

def run(power, target, kp, kd, ki, direction, minRef, maxRef):
	"""
	Description...
	"""
	print 'Run module..'
	lastError = 0
	error = 0
	integral = 0
	left_motor.run_direct()
	right_motor.run_direct()
	lap = 1
	while True :
		print "Lap: " + str(lap)
		if btn.any():
			print 'Breaking loop'
			break
		refRead = col.value()
		print 'Reflective value: ' + str(refRead)
		error = target - (100 * ( refRead - minRef ) / ( maxRef - minRef ))
		print 'Error: ' + str(error)
		derivative = error - lastError
		print 'Derivative: ' + str(derivative)
		lastError = error
		print 'Last error: ' + str(lastError)
		integral = float(0.5) * integral + error
		print 'Integral: ' + str(integral)
		course = (kp * error + kd * derivative +ki * integral) * direction
		print 'Course: ' + str(course)
		print 'Steering: ' + str(steering(course,power))
		for (motor, pow) in zip((left_motor, right_motor), steering(course, power)):
			motor.duty_cycle_sp = pow
			print 'Pow: ' + str(pow)
		lap = lap + 1
		print '-----'
		sleep(1)

run(power, target, kp, kd, ki, direction, minRef, maxRef)

# Stop the motors before exiting.
print 'Stopping motors'
left_motor.stop()
right_motor.stop()
