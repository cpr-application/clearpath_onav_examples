import rospy
from threading import Lock
from clearpath_navigation_msgs.msg import DistanceToGoal
from clearpath_navigation_msgs.msg import MotionState
from geometry_msgs.msg import PoseArray
from clearpath_navigation_msgs.msg import Progress
from clearpath_navigation_msgs.msg import NavigationState
from geometry_msgs.msg import Twist


# The name of the topic for the distance to goal. This needs to match the CPR OutdoorNav API.
DISTANCE_TO_GOAL_TOPIC_NAME = "/navigation/distance_to_goal"
# The name of the topic for the motion state. This needs to match the CPR OutdoorNav API.
MOTION_STATE_TOPIC_NAME = "/navigation/motion_state"
# The name of the topic for the navigation path. This needs to match the CPR OutdoorNav API.
NAV_PATH_TOPIC_NAME = "/navigation/path"
# The name of the topic for the navigation progress. This needs to match the CPR OutdoorNav API.
NAV_PROGRESS_TOPIC_NAME = "/navigation/progress"
# The name of the topic for the navigation commanded velocity. This needs to match the CPR OutdoorNav API.
NAV_CMD_VEL_TOPIC_NAME = "/navigation/cmd_vel"
# The name of the topic for the navigation commanded velocity. This needs to match the CPR OutdoorNav API.
PLATFORM_VEL_TOPIC_NAME = "/platform/cmd_vel"


class NavigationStatusMonitor:
    """Create ROS subscribers for navigation status topics and save the results."""

    def __init__(self, store_data=False, msg_warn_period=10.0):
        """Subscribes for updates to the various navigation topics"""

        self._store_data = store_data
        self._status_lock = Lock()

        self._last_platform_vel = Twist()
        self._platform_vel_sub = rospy.Subscriber(
                PLATFORM_VEL_TOPIC_NAME,
                Twist, self._platformVelCallback)

        self._last_nav_cmd_vel = Twist()
        self._nav_vel_sub = rospy.Subscriber(
                NAV_CMD_VEL_TOPIC_NAME,
                Twist, self._navVelCallback)

        self._distance_to_goal = DistanceToGoal()
        self._distance_to_goal_sub = rospy.Subscriber(
                DISTANCE_TO_GOAL_TOPIC_NAME,
                DistanceToGoal,
                self._distanceToGoalCallback)

        self._motion_state = MotionState()
        self._motion_state_sub = rospy.Subscriber(
                MOTION_STATE_TOPIC_NAME,
                MotionState,
                self._motionStateCallback)

        self._new_path_received = False
        self._planned_path = PoseArray()
        self._planned_path_sub = rospy.Subscriber(
                NAV_PATH_TOPIC_NAME,
                PoseArray,
                self._plannedPathCallback)

        self._progress = Progress()
        self._progress_sub = rospy.Subscriber(
                NAV_PROGRESS_TOPIC_NAME,
                Progress,
                self._progressCallback)

        self._no_new_msg_period = msg_warn_period
        self._last_platform_vel_msg_time = rospy.get_time()
        self._last_nav_cmd_vel_msg_time = rospy.get_time()
        self._last_distance_msg_time = rospy.get_time()
        self._last_motion_state_msg_time = rospy.get_time()
        self._last_progress_msg_time = rospy.get_time()
        self._timer = rospy.Timer(period=rospy.Duration(0.2), callback=self._msgChecker, oneshot=False)

        self._time = []
        self._platform_vel = {
            "lin_x": [],
            "ang_z": [],
        }
        self._nav_cmd_vel = {
            "lin_x": [],
            "ang_z": [],
        }
        self._nav_path = {
            "x": [],
            "y": [],
        }

    def _platformVelCallback(self, msg):
        """Updates the platform velocity.

        Parameters
        ----------
        msg : geometry_msgs.msg.Twist
          The updated platform velocity
        """

        self._last_platform_vel_msg_time = rospy.get_time()
        with self._status_lock:
            self._last_platform_vel = msg

    def _navVelCallback(self, msg):
        """Updates the navigation commanded velocity.

        Parameters
        ----------
        msg : geometry_msgs.msg.Twist
          The updated navigation commanded velocity
        """

        self._last_nav_cmd_vel_msg_time = rospy.get_time()
        with self._status_lock:
            self._last_nav_cmd_vel = msg
            if self._store_data:
                self._time.append(rospy.get_time())
                self._platform_vel["lin_x"].append(self._last_platform_vel.linear.x)
                self._platform_vel["ang_z"].append(self._last_platform_vel.angular.z)
                self._nav_cmd_vel["lin_x"].append(self._last_nav_cmd_vel.linear.x)
                self._nav_cmd_vel["ang_z"].append(self._last_nav_cmd_vel.angular.z)

    def _distanceToGoalCallback(self, msg):
        """Updates the distance to goal state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.DistanceToGoal
          The updated distance to goal state
        """

        self._last_distance_msg_time = rospy.get_time()
        with self._status_lock:
            self._distance_to_goal = msg

    def _motionStateCallback(self, msg):
        """Updates the motion state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.MotionState
          The updated motion state
        """

        self._last_motion_state_msg_time = rospy.get_time()
        with self._status_lock:
            self._motion_state = msg

    def _plannedPathCallback(self, msg):
        """Updates the planned path state.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseArray
          The updated planned path state
        """

        self._new_path_received = True
        with self._status_lock:
            self._planned_path = msg
            if self._store_data:
                for i in range(len(self._planned_path.poses)):
                    self._nav_path["x"].append(self._planned_path.poses[i].position.x)
                    self._nav_path["y"].append(self._planned_path.poses[i].position.y)

    def _progressCallback(self, msg):
        """Updates the progress state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.Progress
          The updated progress state
        """

        self._last_progress_msg_time = rospy.get_time()
        with self._status_lock:
            self._progress = msg

    def _msgChecker(self, event):
        """Check to see if messages have not been received within a certain amount of time"""

        now = rospy.get_time()
        if now - self._last_platform_vel_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new platform velocity message received in the last %0.1f seconds", self._no_new_msg_period)
        if now - self._last_nav_cmd_vel_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new navigation command velocity message received in the last %0.1f seconds", self._no_new_msg_period)
        if now - self._last_distance_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new distance to goal message received in the last %0.1f seconds", self._no_new_msg_period)
        if now - self._last_motion_state_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new motion state received in the last %0.1f seconds", self._no_new_msg_period)
        if now - self._last_progress_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new progress message received in the last %0.1f seconds", self._no_new_msg_period)

    def report(self):
        """Logs all localization information."""

        with self._status_lock:
            rospy.loginfo("  Navigation:")
            rospy.loginfo("    Distance to Goal: Euclidean: %f, Path: %f" % (
                    self._distance_to_goal.euclidean,
                    self._distance_to_goal.path))
            rospy.loginfo("    Motion State: Motion: %d, Indicator: %d, Direction: %d" % (
                    self._motion_state.motion,
                    self._motion_state.indicator,
                    self._motion_state.direction))
            if self._new_path_received:
                rospy.loginfo("    Planned Path:")
                for pose in self._planned_path.poses:
                    rospy.loginfo("      (%f, %f)" % (
                        pose.position.x, pose.position.y))
                self._new_path_received = False
            rospy.loginfo("    Progress: Path: %f, Goal: %f, Mission: %f" % (
                    self._progress.path_progress,
                    self._progress.goal_progress,
                    self._progress.mission_progress))
