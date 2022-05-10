# BallTracker
Publishes (ROS1) ball coordinates with respect to a robotic arm using OpenCV, from the perspective of a static overhead camera.

## Running the ball tracker:
To run the script, open roscore in a terminal and change device to the appropriate one. For webcam, use /dev0, otherwise search for the device you would like to use.

`ls /dev`

For screenshot images, rather than live camera feeds, modify the file to read from file:


'image_path = r'/home/usr/Screenshot/image2.png''
'directory = r'/home/usr/Screenshot/build''
