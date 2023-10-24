import rospy
from threading import Lock
from clearpath_control_msgs.msg import ControlMode
from clearpath_control_msgs.msg import ControlSelectionState


# The name of the topic for current state. This needs to match the CPR OutdoorNav API.
CURRENT_STATE_TOPIC_NAME = "/control_selection/control_state"
# The name of the topic for current mode. This needs to match the CPR OutdoorNav API.
CURRENT_MODE_TOPIC_NAME = "/control_selection/current_mode"


class ControlStatusMonitor:
    """Create ROS subscribers for control status topics and save the results."""

    def __init__(self, msg_warn_period=10.0):
        """Subscribes for updates to the various control topics"""

        self._status_lock = Lock()
        self._control_state = ControlSelectionState()
        self._control_state_sub = rospy.Subscriber(
                CURRENT_STATE_TOPIC_NAME,
                ControlSelectionState,
                self._controlStateCallback)

        self._current_mode = ControlMode()
        self._current_mode_sub = rospy.Subscriber(
                CURRENT_MODE_TOPIC_NAME,
                ControlMode,
                self._controlModeCallback)

        self._no_new_msg_period = msg_warn_period
        self._last_state_msg_time = rospy.get_time()
        self._last_mode_msg_time = rospy.get_time()
        self._timer = rospy.Timer(period=rospy.Duration(0.2), callback=self._msgChecker, oneshot=False)

    def _controlStateCallback(self, msg):
        """Updates the control state.

        Parameters
        ----------
        msg : clearpath_control_msgs.msg.ControlSelectionState
          The updated control state
        """

        self._last_state_msg_time = rospy.get_time()
        with self._status_lock:
            self._control_state = msg

    def _controlModeCallback(self, msg):
        """Updates the current mode.

        Parameters
        ----------
        msg : clearpath_control_msgs.msg.ControlMode
          The updated control mode
        """

        self._last_mode_msg_time = rospy.get_time()
        with self._status_lock:
            self._current_mode = msg

    def getAutonomyEnabled(self):
        """Get value of the autonomy enabled state.

        Returns
        -------
        enabled : bool
            True if autonomy is enabled, else False
        """

        with self._status_lock:
            return self._control_state.autonomy.enabled

    def getAutonomyPaused(self):
        """Get value of the autonomy paused state.

        Returns
        -------
        enabled : bool
            True if autonomy is paused, else False
        """

        with self._status_lock:
            return self._control_state.autonomy.paused

    def getCurrentMode(self):
        """Get the current control mode.

        Returns
        -------
        mode : int8 (enum)
            The current control mode (NEUTRAL, MANUAL, or AUTONOMY)
        """

        with self._status_lock:
            return self._current_mode.mode

    def _msgChecker(self, event):
        """Check to see if messages have not been received within a certain amount of time"""

        now = rospy.get_time()
        if now - self._last_state_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new control state message received in the last %0.1f seconds", self._no_new_msg_period)
        if now - self._last_mode_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new current mode message received in the last %0.1f seconds", self._no_new_msg_period)

    def report(self):
        """Logs all localization information."""

        with self._status_lock:
            rospy.loginfo("  Control:")
            rospy.loginfo("    State: Enabled: %s, Paused: %s " % (
                    self._control_state.autonomy.enabled,
                    self._control_state.autonomy.paused))
            rospy.loginfo("    Mode: %d" % self._current_mode.mode)
