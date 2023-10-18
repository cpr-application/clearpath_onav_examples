import rospy
from threading import Lock
from clearpath_navigation_msgs.msg import CurrentGoalInfo
from clearpath_navigation_msgs.msg import DistanceToGoal
from clearpath_navigation_msgs.msg import MotionState
from geometry_msgs.msg import PoseArray
from clearpath_navigation_msgs.msg import Progress
from clearpath_navigation_msgs.msg import NavigationState


# The name of the topic for the current goal info. This needs to match the CPR OutdoorNav API.
CURRENT_GOAL_INFO_TOPIC_NAME = "/navigation/current_goal_info"
# The name of the topic for the distance to goal. This needs to match the CPR OutdoorNav API.
DISTANCE_TO_GOAL_TOPIC_NAME = "/navigation/distance_to_goal"
# The name of the topic for the motion state. This needs to match the CPR OutdoorNav API.
MOTION_STATE_TOPIC_NAME = "/navigation/motion_state"
# The name of the topic for the navigation path. This needs to match the CPR OutdoorNav API.
NAV_PATH_TOPIC_NAME = "/navigation/path"
# The name of the topic for the navigation progress. This needs to match the CPR OutdoorNav API.
NAV_PROGRESS_TOPIC_NAME = "/navigation/progress"
# The name of the topic for the navigation state. This needs to match the CPR OutdoorNav API.
NAV_STATE_TOPIC_NAME = "/navigation/state"


class NavigationStatusMonitor:
    """Create ROS subscribers for navigation status topics and save the results."""

    def __init__(self, num_bms=0):
        """Subscribes for updates to the various navigation topics"""

        self._status_lock = Lock()

        self._current_goal_info = CurrentGoalInfo()
        self._current_goal_info_sub = rospy.Subscriber(
                CURRENT_GOAL_INFO_TOPIC_NAME,
                CurrentGoalInfo,
                self._currentGoalInfoCallback)

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

        self._navigation_state = NavigationState()
        self._navigation_state_sub = rospy.Subscriber(
                NAV_STATE_TOPIC_NAME,
                NavigationState,
                self._navigationStateCallback)

    def _currentGoalInfoCallback(self, msg):
        """Updates the current goal info state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.CurrentGoalInfo
          The updated current goal info state
        """

        with self._status_lock:
            self._current_goal_info = msg

    def _distanceToGoalCallback(self, msg):
        """Updates the distance to goal state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.DistanceToGoal
          The updated distance to goal state
        """

        with self._status_lock:
            self._distance_to_goal = msg

    def _motionStateCallback(self, msg):
        """Updates the motion state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.MotionState
          The updated motion state
        """

        with self._status_lock:
            self._motion_state = msg

    def _plannedPathCallback(self, msg):
        """Updates the planned path state.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseArray
          The updated planned path state
        """

        with self._status_lock:
            self._planned_path = msg

    def _progressCallback(self, msg):
        """Updates the progress state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.Progress
          The updated progress state
        """

        with self._status_lock:
            self._progress = msg

    def _navigationStateCallback(self, msg):
        """Updates the navigation state.

        Parameters
        ----------
        msg : clearpath_navigation_msgs.msg.NavigationState
          The updated navigation state
        """

        with self._status_lock:
            self._navigation_state = msg

    def report(self):
        """Logs all localization information."""

        with self._status_lock:
            rospy.loginfo("  Navigation:")
            rospy.loginfo("    Current Goal: ID: %s, Endpoint (%f, %f)" % (
                    self._current_goal_info.goal_id,
                    self._current_goal_info.goal.latitude,
                    self._current_goal_info.goal.longitude))
            rospy.loginfo("    Distance to Goal: Euclidean: %f, Path: %f" % (
                    self._distance_to_goal.euclidean,
                    self._distance_to_goal.path))
            rospy.loginfo("    Motion State: Motion: %d, Indicator: %d, Direction: %d" % (
                    self._motion_state.motion,
                    self._motion_state.indicator,
                    self._motion_state.direction))
            rospy.loginfo("    Planned Path:")
            for pose in self._planned_path.poses:
                rospy.loginfo("      (%f, %f, %f)" % (
                    pose.position.x, pose.position.y, pose.position.z))
            rospy.loginfo("    Progress: Path: %f, Goal: %f, Mission: %f" % (
                    self._progress.path_progress,
                    self._progress.goal_progress,
                    self._progress.mission_progress))
            rospy.loginfo("    State:")
            for state in self._navigation_state.states:
                rospy.loginfo("      %d" % state)
