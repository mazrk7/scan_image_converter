# Scan-Image Converter

The `scan_image_converter` ROS package contains a node that takes as input a `sensor_msgs/LaserScan` message and converts it into a `sensor_msgs/Image` (polar to Cartesian space translation).

You can simply run it as follows:
```shell
rosrun scan_image_converter scan_image_converter
```
Which will listen for a laser reading on the topic `base_scan` and publish its corresponding image representation to the `scan_image` topic. The black and white output image also has a flipped frame of reference published to `scan_image/flipped`.

Parameters `image_size_x` & `image_size_y` can be used to adjust the size of the output image.

Any contributions/feedback will be well-received.
