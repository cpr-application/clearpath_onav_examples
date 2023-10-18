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

    def __init__(self, num_bms=0):
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

    def _controlStateCallback(self, msg):
        """Updates the control state.

        Parameters
        ----------
        msg : clearpath_control_msgs.msg.ControlSelectionState
          The updated control state
        """

        with self._status_lock:
            self._control_state = msg

    def _controlModeCallback(self, msg):
        """Updates the current mode.

        Parameters
        ----------
        msg : clearpath_control_msgs.msg.ControlMode
          The updated control mode
        """

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

    def report(self):
        """Logs all localization information."""

        with self._status_lock:
            rospy.loginfo("  Control:")
            rospy.loginfo("    State: Enabled: %s, Paused: %s " % (
                    self._control_state.autonomy.enabled,
                    self._control_state.autonomy.paused))
            rospy.loginfo("    Mode: %d" % self._current_mode.mode)
