# Comms Heartbeat Package

## Description
This package contains two nodes, one to be run on a base station and one to be run on the rover. The Base station tries to call the service on the rover at regular intervals. If it does not recieve a response for 5 seconds, it stops trying to call said service and waits for it to become available. At this time it starts publishing a 'down' message on its own topic. On the Rover side, if it does not get a service request for 5 seconds, it will start publishing its own 'down' message. These two status topics should not be crossed over whatever communications band exists, as that will cause problems.

There also exists functionality for manually setting the system to the 'down' state without connection loss allowing for a sort of software estop to be implemented.

## Known Limitations
This package was not extensively tested, it worked on a small scale but was not integrated well enough to make it to the full version of the rover.

If either node is started without all computers being present on the network, it will not be visible from said unconnected computer when it comes online. This is a limitation of ROS2 Humble and no workaround (other than make sure everything is connected at startup) was worked on.