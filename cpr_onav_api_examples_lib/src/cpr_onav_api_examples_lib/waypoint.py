import rospy
import clearpath_navigation_msgs.msg


class Waypoint:
    """A single waypoint (part of a mission).

    Contains a name, uuid, latitude, longitude, optional tasks to run
    at the point, and various other optional parameters related
    to the specifics at the waypoint.

    A series of waypoints can be grouped in sequence to build a larger mission.
    """

    def __init__(self, name, uuid, latitude, longitude, heading=-1, tasks=[],
                 position_tolerance=0.1, yaw_tolerance=0.2):
        """Creates a Waypoint object from the given parameters.

        Parameters
        ----------
        name : str
            The name or ID of the goal, used for tracking/reporting

        uuid : str
            Unique UUID string for the waypoint

        latitude : float64
            Latitude for the waypoint (in degrees)

        longitude : float64
            Longitude for the waypoint (in degrees)

        heading : float64, optional
            Heading for the waypoint (0-360 degrees)

        tasks : clearpath_navigation_msgs.msg.Task[], optional
            A list of tasks to run at the waypoint (default is no tasks)

        position_tolerance : float64, optional
            The pose (x,y) tolerance, in meters, to apply at the goalpoint (default is 0.1)

        yaw_tolerance : float64, optional
            The yaw tolerance, in radians, to apply at the goalpoint (default is 0.2)
        """

        self._waypoint_msg = clearpath_navigation_msgs.msg.Waypoint()
        self._waypoint_msg.name = name
        self._waypoint_msg.uuid = uuid
        self._waypoint_msg.latitude = latitude
        self._waypoint_msg.longitude = longitude
        self._waypoint_msg.heading = heading
        self._waypoint_msg.tasks = tasks
        self._waypoint_msg.position_tolerance = position_tolerance
        self._waypoint_msg.yaw_tolerance = yaw_tolerance

    def getName(self):
        """Gets the name of the waypoint.

        Returns
        -------
        str
            the name of the waypoint
        """

        return self._waypoint_msg.name

    def getWaypointMsg(self):
        """Gets the waypoint message.

        Returns
        -------
        clearpath_navigation_msgs.msg.Waypoint
            the waypoint message
        """

        return self._waypoint_msg

    def toYaml(self):
        """Converts the waypoint to YAML format, for writing to disk.

        Returns
        -------
        dict
            A YAML-compatible dictionary representation of the object.
        """

        tasks_yaml = []
        for task in self._waypoint_msg.tasks:
            tasks_yaml.append({
                'name': task.name,
                'uuid': task.uuid,
                'action_server_name': task.action_server_name,
                'version': task.version,
                'floats': task.floats,
                'strings': task.strings
            })
        waypoint_yaml = {
          'name': self._waypoint_msg.name,
          'uuid': self._waypoint_msg.uuid,
          'latitude': self._waypoint_msg.latitude,
          'longitude': self._waypoint_msg.longitude,
          'heading': self._waypoint_msg.heading,
          'tasks': tasks_yaml,
          'position_tolerance': self._waypoint_msg.position_tolerance,
          'yaw_tolerance': self._waypoint_msg.yaw_tolerance,
        }
        return waypoint_yaml

    @staticmethod
    def fromYaml(waypoint_yaml):
        """Creates a Waypoint object from a YAML-compatible dictionary representation.

        Parameters
        ----------
        goal_yaml : dict
            The YAML-compatible dictionary representation of a Waypoint object
        """

        try:
            name = waypoint_yaml['name']
            uuid = waypoint_yaml['uuid']
            latitude = waypoint_yaml['latitude']
            longitude = waypoint_yaml['longitude']
            heading = waypoint_yaml['heading']
            position_tolerance = waypoint_yaml['position_tolerance']
            yaw_tolerance = waypoint_yaml['yaw_tolerance']

            tasks_yaml = waypoint_yaml['tasks']
            tasks = []
            for task_yaml in tasks_yaml:
                task = clearpath_navigation_msgs.msg.Task()
                task.name = task_yaml['name']
                task.uuid = task_yaml['uuid']
                task.action_server_name = task_yaml['action_server_name']
                task.version = task_yaml['version']
                task.floats = task_yaml['floats']
                task.strings = task_yaml['strings']
                tasks.append(task)

            return Waypoint(name, uuid, latitude, longitude, heading, tasks,
                            position_tolerance, yaw_tolerance)

        except KeyError as e:
            rospy.logerr('Unable to parse yaml for waypoint: %s' % e)
        return None
