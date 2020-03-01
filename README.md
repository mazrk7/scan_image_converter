# Scan-Image Converter

The `scan_image_converter` ROS package contains a node that takes as input a `sensor_msgs/LaserScan` message and converts it into a `sensor_msgs/Image` (polar to Cartesian space translation).

You can simply run it as follows:
```shell
rosrun scan_image_converter scan_image_converter
```
Which will listen for a laser reading on the topic `base_scan` and publish its corresponding image representation to the `scan_image` topic. The black and white output image also has a flipped frame of reference published to `scan_image/flipped`.

Parameters `image_size_x` & `image_size_y` can be used to adjust the size of the output image.

Illustrations of its use for applications such as Augmented Reality (AR) and robot navigation are available in the following academic papers:

```
@inproceedings{Zolotas2018,
    author = {Zolotas, M and Elsdon, J and Demiris, Y},
    booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    pages = {1823--1829},
    title = {{Head-Mounted Augmented Reality for Explainable Robotic Wheelchair Assistance}},
    year = {2018}
}

@inproceedings{Zolotas2019,
    author = {Zolotas, M and Demiris, Y},
    booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    pages = {3020--3026},
    title = {{Towards Explainable Shared Control using Augmented Reality}},
    year = {2019}
}
```
Which also provide a high-level description of the package's functionality.

Finally, please note that any contributions/feedback will be well-received.
