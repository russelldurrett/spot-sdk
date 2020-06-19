<!--
Copyright (c) 2020 Boston Dynamics, Inc.  All rights reserved.

Downloading, reproducing, distributing or otherwise using the SDK Software
is subject to the terms and conditions of the Boston Dynamics Software
Development Kit License (20191101-BDSDK-SL).
-->

# Spot SDK

Develop applications and payloads for Spot using the Boston Dynamics Spot SDK.

The SDK consists of:
*  [Conceptual documentation](docs/concepts/README.md). These documents explain the key abstractions used by the Spot API.
*  [Python client library](docs/python/README.md). Applications using the Python library can control Spot and read sensor and health information from Spot. A wide variety of example programs and a QuickStart guide are also included.
*  [Payload developer documentation](docs/payload/README.md). Payloads add additional sensing, communication, and control capabilities beyond what the base platform provides. The Payload ICD covers the mechanical, electrical, and software interfaces that Spot supports.
*  [Spot API protocol definition](protos/bosdyn/api/README.md). This reference guide covers the details of the protocol applications used to communicate to Spot. Application developers who wish to use a language other than Python can implement clients that speak the protocol.
*  [Spot SDK Repository](https://github.com/boston-dynamics/spot-sdk). The GitHub repo where all of the Spot SDK code is hosted. 

This is version 2.0.0 of the SDK. Please review the [Release Notes](docs/release_notes.md) to see what has changed.

## Contents

* [Concepts](docs/concepts/README.md)
* [Python](docs/python/README.md)
* [Payloads](docs/payload/README.md)
* [API Protocol](docs/protos/README.md)
* [Release Notes](docs/release_notes.md)
* [SDK Repository](https://github.com/boston-dynamics/spot-sdk)


# Added Functionality
## Importing TF
In order to import tf, as done in the streaming_image.py:
1) make sure you are in py36 virtual env.
2) in the workspace where you have the cv_bridge: cd ~/ws/src
3) git clone https://github.com/ros/geometry
4) git clone https://github.com/ros/geometry2
5) pip install catkin_pkg pyyaml empy rospkg numpy
6) catkin build geometry geometry2
Then, once you want to run a python script in spot-sdk with tf, you simply need to ensure that the workspace with cv_bridge and geometry and geometry2 are sourced:
	i.e. source ~/ws/install/setup.bash
