# line_follower

This ROS package use open_cv to follow a yellow line. 
follower_p.py takes a compressed image from the camera and converts to numpy array which open cv can use. A color mask alters all shades of yellow
to create an image that is more easy for the robot to process, and the robot uses this image to create a centroid on the line. A simple PD controller
uses to position of the centroid to direct the robot. If the robot does not start in a position where the line is visible, the robot moves in a circle until it sees the line. 
If the robot reaches the end of the line, it will turn around until the line is visible again and resume following. 

A second script no_vision_move.py is triggered in the case that a robot detects an obstacle while following the line. This script passively
tracks the robots change in position every second. When triggered, it uses the robots current position and previous position to interpolate
its linear trajectory and sets its goal to a point along that trajectory about 2 units a way. While following the line the first few times, I had
the robot build a map of the space around the line. The robot does seem to have trouble localizing itself within this map which can create some undesirable behavior. 
There is another flag in the follower_p script which triggers in the case that the robot lost the line and now sees one, where a signal is sent
no_vision_move to cancel the move_base action and let the robot continue following the line. This also behaves unreliably. 

I found the computer vision aspect fairly straightforward and was able to use much of the book's code for line following. Integrating these different components
was more difficult and time-consuming than I anticipated, but I think in the end it was helpful to reason about how move_base and vision might be able to work together
in cases where the robot can no longer rely on vision. 
