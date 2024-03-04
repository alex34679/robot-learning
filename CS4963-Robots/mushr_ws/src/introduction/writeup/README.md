# Project 1: Introduction [![tests](../../../badges/submit-proj1/pipeline.svg)](../../../pipelines/submit-proj1/latest)

## Qs
 - A Node is an individual process running apart of the ROS system that handles a specific task. These nodes can communicate via a message queue/broker
 - A Topic is a specific “channel” within the ROS message queue that pertains to a specific input/output, for example the “/car/car_pose” topic pushes an object containing the current position of the robot.
 - A publisher is a type of client that can push messages/object onto a specific topic in the ROS message queue, where subscribers can then listen for these messages.
 - A subscriber is a type of client that listens to messages on a specific topic pushed by publishers. This client requires you to specific a callback function which is called on each new message consumed.
 - A launch file is an XML file that depicts how ROS should start a particular node / set of nodes as well as their arguments, parameters and default values.

## Rviz alternative (Foxglove via rosbridge server)
![Foxglove](rvizalt.png)

## Runtime Comparison
![runtime](runtime_comparison.png)

## Figure 8

### Locations
![Figure8](figure_8_locations.png)

### Distances
![Figure8](figure_8_distances.png)

## Tight 8

### Locations
![Tight8](Tight_8_locations.png)

### Distances
![Tight8](tight_8_distances.png)