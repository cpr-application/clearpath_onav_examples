import rospy
import actionlib
import clearpath_navigation_msgs.msg
from clearpath_onav_api_examples_lib.task import Task
from clearpath_onav_api_examples_lib.waypoint import Waypoint


# The name of the action server to which missions are sent.
# This needs to match the CPR OutdoorNav API.
MISSION_ACTION_NAME = 'mission'


class Mission:
    """A sequence of waypoints with optional tasks."""

    def __init__(self, name, uuid, waypoints, on_start_tasks, on_stop_tasks,
                 from_start, start_waypoint_num, onav_config="",
                 progress_callback=None, done_callback=None):
        """Builds up the data structure containing the details of the mission.

        Parameters
        ----------
        name : str
            The name of the mission

        uuid : str
            UUID string used to uniquely identify this mission

        waypoints : Waypoint[]
            A list of Waypoint objects, to be executed in sequence

        on_start : Task[]
            An array of Tasks to execute on Mission start

        on_stop : Task[]
            An array of Tasks to execute on Mission stop. These 
            Tasks will execute regardless of mission success or failure

        from_start: bool
            Whether or not the mission should start from the first Waypoint

        start_waypoint_num: int
            The index representing the Waypoint in the list of Waypoint, from which the
            mission should start.

        onav_config: str
            Configuration parameters for the mission

        progress_callback : fn(feedback_str), optional
            A callback function that is called at regular intervals while the mission
            is being executed to provide progress updates. The function should take
            the "feedback_str" string parameter. If set to None, no feedback will be provided
            and the mission will execute to completion.

        done_callback : fn(bool), optional
            A callback function that is called when the mission is complete. The function
            should take one parameter: a bool to indicate if the mission was completed
            successfully.
        """

        self._name = name
        self._uuid = uuid
        self._waypoints = waypoints
        self._on_start_tasks = on_start_tasks
        self._on_stop_tasks = on_stop_tasks
        self._from_start = from_start
        self._start_waypoint_num = start_waypoint_num
        self._onav_config = onav_config
        self._mission_progress_callback = progress_callback
        self._mission_done_callback = done_callback
        self._mission_success = False
        self._mission_complete = True
        self._client = None

    def _toMissionMsg(self):
        """Gets the mission message.

        Returns
        -------
        clearpath_navigation_msgs.msg.Mission
            the mission message
        """

        mission_msg = clearpath_navigation_msgs.msg.Mission()
        mission_msg.name = self._name
        mission_msg.uuid = self._uuid
        mission_msg.onav_config = self._onav_config
        waypoint_msgs = []
        for waypoint in self._waypoints:
            waypoint_msgs.append(waypoint.getWaypointMsg())
        mission_msg.waypoints = waypoint_msgs
        on_start_msgs = []
        for task in self._on_start_tasks:
            on_start_msgs.append(task.getTaskMsg())
        mission_msg.on_start = on_start_msgs
        on_stop_msgs = []
        for task in self._on_stop_tasks:
            on_stop_msgs.append(task.getTaskMsg())
        mission_msg.on_stop = on_stop_msgs
        return mission_msg

    def _missionFeedbackCallback(self, feedback):
        """Executes when 'feedback' is received from the mission execution.

        Forwards the callback to the user-provided mission feedback callback.
        """

        rospy.logdebug("Mission %s: %s" % (self._name, feedback.state))
        if self._mission_progress_callback is not None:
            self._mission_progress_callback(feedback)

    def _missionDoneCallback(self, mission_state, result):
        """Executes when 'done' is received from the mission execution.

        Forwards the callback to the user-provided mission done callback.
        """

        # Due to a bug, the result field is not getting filled in properly at
        # present. Once the bug is fixed, should check "result.success" rather
        # than just "result".
        self._mission_complete = True
        if mission_state != actionlib.GoalStatus.SUCCEEDED or not result:
            mission_result_str = "Failed"
            self._mission_success = False
            rospy.logdebug("Mission complete, but not successful.")
        else:
            mission_result_str = "Succeeded"
            self._mission_success = True
        mission_feedback = "Mission %s: %s" % (self._name, mission_result_str)
        rospy.logdebug(mission_feedback)
        if self._mission_done_callback is not None:
            self._mission_done_callback(self._mission_success)

    def _sendMission(self):
        """Sends the mission to the action server.

        Returns
        -------
        bool
            True on successful dispatch, else False
        """

        if self._client is None:
            rospy.logerr('Action client not started. Cannot send mission.')
            return False

        self._mission_complete = False
        mission_goal_msg = clearpath_navigation_msgs.msg.MissionGoal()
        mission_goal_msg.mission = self._toMissionMsg()
        mission_goal_msg.from_start = self._from_start
        if not mission_goal_msg.from_start:
            mission_goal_msg.start_waypoint = self._waypoints[self._start_waypoint_num]
        try:
            # Waits until the action server has started up and started
            # listening for goals, then sends the goal
            print('Trying to connect to mission server...')
            if not self._client.wait_for_server(timeout=rospy.Duration()):
                rospy.logerr("Failed to connect to server")
                return False
            print('Connected to mission server')
            self._client.send_goal(mission_goal_msg,
                                   feedback_cb=self._missionFeedbackCallback,
                                   done_cb=self._missionDoneCallback)
            print('Mission executing...')
        except rospy.ROSInterruptException:
            rospy.logerr("mission server is not available; exception detected")
            return False
        return True

    def getName(self):
        """Gets the name of the mission

        Returns
        -------
        str
            The name of the mission
        """

        return self._name

    def setCallbacks(self, progress_callback=None, done_callback=None):
        """Set the callbacks for the mission.

        Provides a way to set the callbacks if not done during object creation,
        which is the case when creating the object from a YAML file.

        Parameters
        ----------
        progress_callback : fn(feedback_str), optional
            A callback function that is called at regular intervals while the mission
            is being executed to provide progress updates. The function should take
            the "feedback_str" string parameter. If set to None, no feedback will be provided
            and the mission will execute to completion.

        done_callback : fn(bool), optional
            A callback function that is called when the mission is complete. The function
            should take one parameters: a bool to indicate if the mission was completed
            successfully.
        """

        self._mission_progress_callback = progress_callback
        self._mission_done_callback = done_callback

    def getMissionSuccess(self):
        """Gets the current status of the mission.

        If the mission is still in progress, reports the current status. If the
        mission is complete, reports the overall mission status.

        See also isMissionComplete().

        Returns
        -------
        bool
            True if the mission is running successfully so far, else False
        """

        return self._mission_success

    def isMissionComplete(self):
        """Determines if the mission is complete.

        See also getMissionSuccess().

        Returns
        -------
        bool
            True if the mission is complete, else False if mission is still running
        """

        return self._mission_complete

    def startMission(self):
        """Starts the execution of the mission.

        This function returns as soon as the mission is started. See isMissionComplete()
        and getMissionSuccess() to determine mission status. If there is already an
        active mission, a new one cannot be started until the active mission completes
        or has been stopped.

        See also stopMission().

        Returns
        -------
        bool
            True on successful mission start, else False
        """

        # Connect to the action server, if it is not already started
        if self._client is None:
            print('Creating mission client')
            self._client = actionlib.SimpleActionClient(MISSION_ACTION_NAME,
                                                        clearpath_navigation_msgs.msg.MissionAction)
        # Do not start a new mission if one is already active
        if not self.isMissionComplete():
            rospy.logwarn("Existing mission is still active. Cannot start new mission.")
            return False

        success = self._sendMission()
        if not success:
            rospy.logerr("Cannot send mission. Failed to start mission.")
        # Initialize the mission success, for final reporting
        self._mission_success = success
        return success

    def stopMission(self):
        """Stops the mission that is currently executing.

        Returns
        -------
        bool
            True if the mission was stopped successfully, else False"""

        if self._client is not None:
            self._client.cancel_all_goals()
            rospy.logdebug("Stopping mission. All goals cancelled. Setting success to False.")
        self._mission_success = False
        self._mission_complete = True

    def toYaml(self):
        """Converts the mission to YAML format, for writing to disk.

        Returns
        -------
        dict
            A YAML-compatible dictionary representation of the object.
        """

        waypoints = self._waypoints
        waypoints_yaml = []
        for waypoint in waypoints:
            waypoint_yaml = waypoint.toYaml()
            waypoints_yaml.append(waypoint_yaml)

        on_start_tasks = self._on_start_tasks
        on_start_tasks_yaml = []
        for task in on_start_tasks:
            on_start_task_yaml = task.toYaml()
            on_start_tasks_yaml.append(on_start_task_yaml)

        on_stop_tasks = self._on_stop_tasks
        on_stop_tasks_yaml = []
        for task in on_stop_tasks:
            on_stop_task_yaml = task.toYaml()
            on_stop_tasks_yaml.append(on_stop_task_yaml)

        mission_yaml = {
            'name': self._name,
            'uuid': self._uuid,
            'waypoints': waypoints_yaml,
            'on_start_tasks': on_start_tasks_yaml,
            'on_stop_tasks': on_stop_tasks_yaml,
            'onav_config': self._onav_config
        }
        return mission_yaml

    @staticmethod
    def fromYaml(mission_yaml):
        """Creates a Mission object from a YAML-compatible dictionary representation.

        Parameters
        ----------
        mission_yaml : dict
            The YAML-compatible dictionary representation of a Mission object
        """

        if 'name' not in mission_yaml:
            rospy.logerr("yamlFileToMission: missing 'name' field")
            return None
        if 'uuid' not in mission_yaml:
            rospy.logerr("yamlFileToMission: missing 'uuid' field")
            return None
        if 'waypoints' not in mission_yaml:
            rospy.logerr("yamlFileToMission: missing 'waypoints' field")
            return None
        if 'on_start_tasks' not in mission_yaml:
            rospy.logerr("yamlFileToMission: missing 'on_start_tasks' field")
            return None
        if 'on_stop_tasks' not in mission_yaml:
            rospy.logerr("yamlFileToMission: missing 'on_stop_tasks' field")
            return None
        if 'onav_config' not in mission_yaml:
            rospy.logerr("yamlFileToMission: missing 'onav_config' field")
            return None
        name = mission_yaml['name']
        uuid = mission_yaml['uuid']
        onav_config = mission_yaml['onav_config']
        waypoints = []
        waypoints_yaml = mission_yaml['waypoints']
        for waypoint_yaml in waypoints_yaml:
            waypoint = Waypoint.fromYaml(waypoint_yaml)
            if waypoint is None:
                rospy.logerr("yamlFileToMission: could not parse 'waypoints'")
                return None
            waypoints.append(waypoint)

        on_start_tasks = []
        on_start_tasks_yaml = mission_yaml['on_start_tasks']
        for on_start_task_yaml in on_start_tasks_yaml:
            task = Task.fromYaml(on_start_task_yaml)
            if task is None:
                rospy.logerr("yamlFileToMission: could not parse 'on_start_tasks'")
                return None
            on_start_tasks.append(task)

        on_stop_tasks = []
        on_stop_tasks_yaml = mission_yaml['on_stop_tasks']
        for on_stop_task_yaml in on_stop_tasks_yaml:
            task = Task.fromYaml(on_stop_task_yaml)
            if task is None:
                rospy.logerr("yamlFileToMission: could not parse 'on_stop_tasks'")
                return None
            on_stop_tasks.append(task)

        return Mission(name, uuid, waypoints, on_start_tasks, on_stop_tasks, onav_config)
