# PID Controlled line follower

I recently read the “The Art of LEGO MINDSTORMS EV3 Programming (Full Color)” book by Terry Griffin and tried to recreate the PID controlled line follower (chapter 19) in Python instead of in the Mindstorms environment. I have created three small programs here but with very limited documentation.

The first one, max_min_finder.py, could be used to find max and min readings from the color sensor in reflective mode. Place the robot ~10cm from the line (perpendicular) and run the robot from Stash ssh. Max and Min values will be printed in the console.

The second one, Line_follow.py uses a PID algorithm to calculate the necessary direction based on the readings from the color sensor, history etc. There is a much better explanation in chapter 19 in the above book. The steering module in this program computes how fast (-100 to 100) each motor in a pair should turn to achieve the specified steering based on the direction from the PID algorithm.

The third one, Line_follow_2.py is like the above but with another steering function that maybe are easier on the motors, I don't know, but it does not work as good as the first one.

Before it works any good you need to tune the controller.

* Set the minRef and maxRef values according to max_min_finder program.
* Below are some tips from the book.
* Set the Power to 50.
* Start with kd and ki at 0 and kp at 1.
* Start by testing with a straight line. Progressively reduce kp by 0.05 until the robot follows a line with no side-to-side movement or only small movement to one side of the edge.
* Progressively increase ki by 0.01 until the robot follows the edge of a straight line with no oscillation. If the robot does not constantly drift to one side, you may be able to leave ki at 0. Be aware that setting ki too high (above 0.05) will cause the oscillations to grow bigger.
* Test the program on a line with curves. Increase the power variable until the robot is unable to make the turn.
* Progressively increase kd by 1 until the robot can traverse the entire path.
I will try to create a better step by step explanation with pictures/videos when I have a little more time.

Also see: https://forum.omz-software.com/topic/2677/best-practice-using-github
