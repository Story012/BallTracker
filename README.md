# BallTracker
Publishes (ROS1) ball coordinates with respect to a robotic arm using OpenCV, from the perspective of a static overhead camera.

![Successful detection of table edges and centoids](https://github.com/Story012/BallTracker/blob/main/Screenshot/Color%20Tracking_screenshot_27.04.2022.png)

## Running the ball tracker:
To run the script, open roscore in a terminal and change device to the appropriate one. For webcam, use /dev0, otherwise search for the device you would like to use with `ls /dev`.

For screenshot images, rather than live camera feeds, modify the file to read from file:

```
image_path = r'/home/usr/Screenshot/image2.png'
directory = r'/home/usr/Screenshot/build
img = cv2.imread(image_path)
```
