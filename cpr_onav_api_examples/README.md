# OutdoorNav API Examples

These examples outline how to use the API to define and execute missions.

## Building the Examples

Use the steps below to download and build the examples. This can be performed
directly on the target robot or on a separate computer with ROS Noetic
installed.

```
mkdir -p ~/outdoornav_ws/src
cd ~/outdoornav_ws
catkin_init_workspace src
cd src
git clone https://github.com/clearpathrobotics/CPR-OutdoorNav.git cpr_outdoornav
cd ~/outdoornav_ws
rosdep install -y --from-paths src --ignore-src
catkin build
```

## Running the First Example

The commands below outline how to run one of the example programs when
the examples have been downloaded and built on the target robot directly.
The remaining examples can be run in a similar manner.
If the examples have been downloaded and built on a remote computer,
following the instructions at [this tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) to
have the remote computer point to the ROS Master on the target robot.

```
source ~/outdoornav_ws/devel/setup.bash
rosrun cpr_onav_api_examples simple_mission_1
```

## Running an Inspection Example

The commands below outline how to run an example program that represents a
simple inspection mission in the Ag World in which the robot drives between
rows of solar panels and collects inspection data at a few key points
along the route.

```
source ~/outdoornav_ws/devel/setup.bash
rosrun cpr_onav_api_examples inspection_mission_1
```

The time to run the mission will depend on the speed of your simulation,
but will normally be in the range of 5-20 minutes, depending on
the compute power of the node running the simulation.
The console output should be similar to:

```
$ rosrun cpr_onav_api_examples inspection_mission_1
[INFO] [1674669519.255228, 234.054000]: Solar Panel ID: 26.000000 Status: Operating normally
[INFO] [1674669708.237271, 272.053000]: Solar Panel ID: 21.000000 Status: Fault detected
Mission completed successfully
```

If the UI is open while the mission is running, you should see something
similar to the following sequence of images. Note how the robot navigates between
two rows of solar panels and stops at two specific panels (one near the midpoint
and one near the end of the row) to allow an inspection to be performed.

![inspection_mission_1 Ready to start](../../images/inspection_mission_1/api/inspection_mission_demo_step1.png?raw=true)
![inspection_mission_1 Toward mission start](../../images/inspection_mission_1/api/inspection_mission_demo_step2.png?raw=true)
![inspection_mission_1 Toward panel 26](../../images/inspection_mission_1/api/inspection_mission_demo_step3.png?raw=true)
![inspection_mission_1 Toward panel 21](../../images/inspection_mission_1/api/inspection_mission_demo_step4.png?raw=true)
![inspection_mission_1 Toward end](../../images/inspection_mission_1/api/inspection_mission_demo_step5.png?raw=true)
![inspection_mission_1 End](../../images/inspection_mission_1/api/inspection_mission_demo_step6.png?raw=true)

## Running an Area Coverage Example

The commands below outline how to run an example program that represents a
simple area coverage mission in the Ag World in which the user defines a
bounding region and then the robot drives back and forth to ensure that
the simulated scanner provides coverage of the entire area.

```
source ~/outdoornav_ws/devel/setup.bash
rosrun cpr_onav_api_examples simple_area_coverage
```

The time to run the mission will depend on the speed of your simulation,
but will normally be in the range of 5-20 minutes, depending on
the compute power of the node running the simulation.
The console output should be similar to:

```
$ rosrun cpr_onav_api_examples simple_area_coverage
[INFO] [1675994738.888331, 45.537000]: Computing waypoints for (-65.134179, -0.264033), (-65.134179, 14.735967), (-25.134179, 14.735967), (-25.134179, -0.264033)
[INFO] [1675996427.370797, 260.487000]: Adding result: 99 at (50.109474, -97.319886)
[INFO] [1675996477.491238, 271.487000]: Adding result: 92 at (50.109458, -97.319889)
...
Mission completed successfully. Scanner hits:
Scanner reading 99 at Lat: 50.109474 Lon: -97.319886
Scanner reading 92 at Lat: 50.109458 Lon: -97.319889
...
```

If the UI is open while the mission is running, you should see something
similar to the following images.

![simple_area_coverage Area Coverage](../../images/simple_area_coverage/api/simple_area_coverage_1.png?raw=true)

## Reading and Writing Mission Configuration

Missions can be saved to and loaded from YAML configuration files.
Refer to the **mission\_from\_yaml\_file** example for details.

## Obtaining Goalpoint and Viapoint Coordinates

_Missions_ are made up of one or more _Goals_, each of which has a
_Goalpoint_ (the endpoint of the path to the Goal) and zero or more
_Viapoints_ (intermediate points leading to the Goal). The essential
properties of both _Goalpoints_ and _Viapoints_ are the coordinates of
the point. When using the Web UI, the coordinates are determined based
on the map location where the operator clicks to add points. However,
since there is no UI for the API, how does a user get coordinates to
build and run missions via the API? There are several options.

1. Use a third party resource, such as Google Maps, to manually identify
   the coordinates for your points. See **simple\_mission\_1** and
   **simple\_mission\_2** for examples of mission that rely on
   external sources for coordinates and use those to build up the mission.

2. Build the mission using the Web UI, then import the saved Web UI configuration
   into your API code. Refer to the **mission\_from\_webui\_config** example.

3. Manually navigate the robot through the mission one time, collecting the
   coordinates at the points of interest.

    * Use the joystick or the Web UI to teleoperate the robot. Begin at the
      starting point, record the coordinates, drive to the next point,
      record the coordinates, and so on.

    * To collect the coordinates, you can use the **getRobotPosition** tool,
      which extracts the current coordinates and can be used to append to a
      CSV file. For goalpoints, also provide a name; for viapoints, omit the
      name. See **sample_mission.csv** for an example CSV file.

      ```
      # Drive the robot to the start point
      getRobotPosition > mission1.csv
      # Drive the robot to the next viapoint for Goal A
      getRobotPosition >> mission1.csv
      # Drive the robot to the goalpoint A
      getRobotPosition A >> mission.csv
      ## Drive the robot to a new goalpoint B (assume no viapoints)
      getRobotPosition B >> mission.csv
      ```

    * Once the CSV has been created, import it as a mission. Refer to the
      **mission\_from\_csv\_config** example.
