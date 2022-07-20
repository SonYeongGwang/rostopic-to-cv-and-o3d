# rostopic-to-cv-and-o3d
This repository helps user whose application runs in Python3 environment easily convert ros-simulated-data into opencv and open3d format.

This repository features:
* **data_convert.py** - python3 users have difficulty in using `cv_bridge`. This node helps convert simulated images using `cv_bridge` and re-publish.
* **data_getter.py** - Node for getting re-published data converted from `data_convert` node. User application could be placed here.

Tested with Ubuntu 18.04
