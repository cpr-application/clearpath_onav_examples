import rospy
import clearpath_navigation_msgs.msg


class Goal:
    """A single goal (part of a mission).

    Contains a goalpoint, optional viapoints, optional tasks to run
    at the goalpoint, and various other optional parameters related
    to the specifics at the goal point.

    A series of goals can be run in sequence to build a larger mission.
    """

    @staticmethod
    def createGoal(name, datum_latlon, goalpoint_latlon, viapoints_latlon=[], tasks=[],
                   enable_final_heading=False, goalpoint_heading=0,
                   enable_goal_tolerance=False, position_tolerance=0.1, yaw_tolerance=0.2):
        """Creates a Goal object from the given parameters.

        Parameters
        ----------
        name : str
            The name or ID of the goal, used for tracking/reporting

        datum_latlon : CoordinateLatLon
            The latitude/longitude position of the datum

        goalpoint_latlon : CoordinateLatLon
            The latitude/longitude position of the goalpoint

        viapoints_latlon : CoordinateLatLon[], optional
            A list of latitude/longitude positions of the viapoints,
            starting from the beginning of the path and finishing
            toward the goalpoint (default is no viapoints)

        tasks : clearpath_navigation_msgs.msg.Task[], optional
            A list of tasks to run at the goalpoint (default is no tasks)

        enable_final_heading : bool, optional
            Whether or not to apply a heading at the goalpoint (default is False)

        goalpoint_heading : float64, optional
            The heading, in degrees, to apply at the goalpoint (default is 0);
            only gets applied when enable_final_heading is True

        enable_goal_tolerance : bool, optional
            Whether or not to enforce a tolerance at the goalpoint (default is False)

        position_tolerance : float64, optional
            The pose (x,y) tolerance, in meters, to apply at the goalpoint (default is 0.1);
            only gets applied when enable_goal_tolerance is True

        yaw_tolerance : float64, optional
            The yaw tolerance, in radians, to apply at the goalpoint (default is 0.2);
            only gets applied when enable_goal_tolerance is True

        Returns
        -------
        Goal
            the created goal
        """

        goalpoint_xy = goalpoint_latlon.toXY(datum_latlon)
        goal_msg = clearpath_navigation_msgs.msg.MissionGoal()
        goal_msg.mission.enable_final_heading = enable_final_heading
        goal_msg.mission.goalpoint_heading = goalpoint_heading
        goal_msg.mission.enable_goal_tolerance = enable_goal_tolerance
        goal_msg.mission.position_tolerance = position_tolerance
        goal_msg.mission.yaw_tolerance = yaw_tolerance
        goal_msg.mission.goalpoint.x = goalpoint_xy.getX()
        goal_msg.mission.goalpoint.y = goalpoint_xy.getY()

        # Need viapoints in reverse order here to match what is expected by the API
        goal_msg.mission.viapoints = []
        goal_msg.mission.tasks = tasks
        for viapoint_latlon in reversed(viapoints_latlon):
            viapoint_xy = viapoint_latlon.toXY(datum_latlon)
            viapoint = clearpath_navigation_msgs.msg.Waypoint()
            viapoint.x = viapoint_xy.getX()
            viapoint.y = viapoint_xy.getY()
            goal_msg.mission.viapoints.append(viapoint)

        return Goal(name, goal_msg)

    def __init__(self, name, goal_msg):
        """A new Goal object.

        Parameters
        ----------
        name : str
            The name of the goal

        goal_msg : clearpath_navigation_msgs.msg.MissionGoal
            The goal message to be sent to the action server
        """

        self._name = name
        self._goal_msg = goal_msg

    def getName(self):
        """Gets the name of the goal.

        Returns
        -------
        str
            the name of the goal
        """

        return self._name

    def getGoalMsg(self):
        """Gets the goal message.

        Returns
        -------
        clearpath_navigation_msgs.msg.MissionGoal
            the goal message
        """

        return self._goal_msg

    def sendGoal(self, actionlib_client, progress_callback=None, done_callback=None):
        """Sends to goal to be executed.

        This function returns immediately once the goal has been sent to
        the navigation engine; it does not wait for the goal to complete.

        Parameters
        ----------
        actionlib_client : actionlib.SimpleActionClient reference
            Reference to an instance of actionlib.SimpleActionClient use to make calls to
            the appropriate actionlib server

        progress_callback : fn(feedback_str), optional
            A callback function that is called at regular intervals while the goal
            is being executed to provide progress updates. The function should take
            the "feedback_str" string parameter. If set to None, no feedback will be provided
            and the goal will execute to completion.

        done_callback : fn(enum, bool), optional
            A callback function that is called when the goal is complete. The function should take
            two parameters: the terminal state (as an integer from actionlib_msgs/GoalStatus) and
            the result.

        Returns
        -------
        bool
            True on successful goal dispatch, else False
        """

        try:
            # Waits until the action server has started up and started
            # listening for goals, then sends the goal
            if not actionlib_client.wait_for_server(timeout=rospy.Duration()):
                rospy.logerr("Failed to connect to server")
                return False
            actionlib_client.send_goal(self._goal_msg,
                                       feedback_cb=progress_callback,
                                       done_cb=done_callback)
        except rospy.ROSInterruptException:
            rospy.logerr("mission server is not available; exception detected")
            return False
        return True

    def waitForGoalCompletion(self, actionlib_client, timeout=rospy.Duration()):
        """Waits for goal to be executed.

        This function blocks until the goal has been executed fully.

        Parameters
        ----------
        actionlib_client : actionlib.SimpleActionClient reference
            Reference to an instance of actionlib.SimpleActionClient use to make calls to
            the appropriate actionlib server

        timeout : rospy.Duration, optional
            Amount of time to wait for the goal to complete (default is to wait forever)

        Returns
        -------
        bool
            True on successful goal completion, else False
        """

        try:
            # Waits until the action server has started up and started
            # listening for goals, then sends the goal
            if not actionlib_client.wait_for_server(timeout=rospy.Duration()):
                return False

            # Wait for the server to finish performing the action
            actionlib_client.wait_for_result(timeout)

            # Return the result of executing the action (result is a bool)
            return actionlib_client.get_result()
        except rospy.ROSInterruptException:
            rospy.logerr("mission server is not available; exception detected")
            return False

    def sendGoalAndWait(self, actionlib_client):
        """Sends the goal to be executed and waits until completed.

        This function blocks until the goal has been executed fully.

        Parameters
        ----------
        actionlib_client : actionlib.SimpleActionClient reference
            Reference to an instance of actionlib.SimpleActionClient use to make calls to
            the appropriate actionlib server

        Returns
        -------
        bool
            True on successful goal completion, else False
        """

        if not self.sendGoal(actionlib_client):
            return False
        if not self.waitForGoalCompletion(actionlib_client):
            return False

    def cancelGoal(self, actionlib_client):
        """Cancels the goal that we are currently executing

        Parameters
        ----------
        actionlib_client : actionlib.SimpleActionClient reference
            Reference to an instance of actionlib.SimpleActionClient use to make calls to
            the appropriate actionlib server
        """

        actionlib_client.cancel_goal()

    def toYaml(self):
        """Converts the goal to YAML format, for writing to disk.

        Returns
        -------
        dict
            A YAML-compatible dictionary representation of the object.
        """

        name = self.getName()
        goal_msg = self.getGoalMsg()
        viapoints_yaml = []
        for viapoint in goal_msg.mission.viapoints:
            viapoints_yaml.append({
                'x': float(viapoint.x),
                'y': float(viapoint.y)
            })
        tasks_yaml = []
        for task in goal_msg.mission.tasks:
            tasks_yaml.append({
                'task_name': task.task_name,
                'action_server_name': task.action_server_name,
                'version': task.version,
                'floats': task.floats,
                'strings': task.strings
            })
        goal_msg_yaml = {
            'goalpoint': {
                'x': float(goal_msg.mission.goalpoint.x),
                'y': float(goal_msg.mission.goalpoint.y)
            },
            'enable_final_heading': goal_msg.mission.enable_final_heading,
            'goalpoint_heading': goal_msg.mission.goalpoint_heading,
            'enable_goal_tolerance': goal_msg.mission.enable_goal_tolerance,
            'position_tolerance': goal_msg.mission.position_tolerance,
            'yaw_tolerance': goal_msg.mission.yaw_tolerance,
            'viapoints': viapoints_yaml,
            'tasks': tasks_yaml
        }
        goal_yaml = {
          'name': name,
          'msg': goal_msg_yaml
        }
        return goal_yaml

    @staticmethod
    def fromYaml(goal_yaml):
        """Creates a Goal object from a YAML-compatible dictionary representation.

        Parameters
        ----------
        goal_yaml : dict
            The YAML-compatible dictionary representation of a Goal object
        """

        try:
            name = goal_yaml['name']
            goal_msg_yaml = goal_yaml['msg']
            goal_msg = clearpath_navigation_msgs.msg.MissionGoal()

            goal_msg.mission.goalpoint.x = goal_msg_yaml['goalpoint']['x']
            goal_msg.mission.goalpoint.y = goal_msg_yaml['goalpoint']['y']

            goal_msg.mission.enable_final_heading = goal_msg_yaml['enable_final_heading']
            goal_msg.mission.goalpoint_heading = goal_msg_yaml['goalpoint_heading']
            goal_msg.mission.enable_goal_tolerance = goal_msg_yaml['enable_goal_tolerance']
            goal_msg.mission.position_tolerance = goal_msg_yaml['position_tolerance']
            goal_msg.mission.yaw_tolerance = goal_msg_yaml['yaw_tolerance']

            viapoints_yaml = goal_msg_yaml['viapoints']
            goal_msg.mission.viapoints = []
            for viapoint_yaml in viapoints_yaml:
                viapoint = clearpath_navigation_msgs.msg.Waypoint()
                viapoint.x = viapoint_yaml['x']
                viapoint.y = viapoint_yaml['y']
                goal_msg.mission.viapoints.append(viapoint)

            tasks_yaml = goal_msg_yaml['tasks']
            goal_msg.mission.tasks = []
            for task_yaml in tasks_yaml:
                task = clearpath_navigation_msgs.msg.Task()
                task.task_name = task_yaml['task_name']
                task.action_server_name = task_yaml['action_server_name']
                task.version = task_yaml['version']
                task.floats = task_yaml['floats']
                task.strings = task_yaml['strings']
                goal_msg.mission.tasks.append(task)

            goal = Goal(name, goal_msg)
            return goal
        except KeyError as e:
            rospy.logerr('Unable to parse yaml for goal: %s' % e)
        return None
