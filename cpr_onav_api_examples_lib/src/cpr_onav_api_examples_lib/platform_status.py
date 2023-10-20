import rospy
from threading import Lock
from wireless_msgs.msg import Connection as WirelessConnection
from sensor_msgs.msg import BatteryState


# The name of the topic for the wireless connection. This needs to match the CPR OutdoorNav API.
WIRELESS_CONNECTION_TOPIC_NAME = "/onboard_systems/wireless/connection"
# The root name of the topic for the BMS. This needs to match the CPR OutdoorNav API.
BMS_ROOT_TOPIC_NAME = "/onboard_systems/bms"


class PlatformStatusMonitor:
    """Create ROS subscribers for Platform API topics and save the results."""

    def __init__(self, num_bms=0, msg_warn_period=10.0):
        """Subscribes for updates to the various platform topics

        Parameters
        ----------
        num_bms : int, optional
          Number of BMS in the system (default is 0)
        """

        self._status_lock = Lock()

        self._wireless_connection = WirelessConnection()
        self._wireless_connection_sub = rospy.Subscriber(
                WIRELESS_CONNECTION_TOPIC_NAME,
                WirelessConnection,
                self._wirelessConnectionCallback)

        if num_bms > 0:
            self._bms_state = [BatteryState()] * num_bms
            self._bms_state_sub = [None] * num_bms
            for i in range(num_bms):
                topic = BMS_ROOT_TOPIC_NAME + "/[%d]/state" % i
                self._bms_state_sub[i] = rospy.Subscriber(
                        topic, BatteryState, self._bmsStateCallback, i)
        else:
            self._bms_state = []

        self._no_new_msg_period = msg_warn_period
        self._last_wireless_msg_time = rospy.get_time()
        self._last_bms_msg_time = rospy.get_time()
        self._timer = rospy.Timer(period=rospy.Duration(0.2), callback=self._msgChecker)

    def _wirelessConnectionCallback(self, msg):
        """Updates the wireless connection state.

        Parameters
        ----------
        msg : wireless_msgs.msg.Connection
          The updated wireless connection state
        """

        self._last_wireless_msg_time = rospy.get_time()
        with self._status_lock:
            self._wireless_connection = msg

    def _bmsStateCallback(self, msg, index):
        """Updates the BMS state.

        Parameters
        ----------
        msg : sensor_msgs.msg.BatteryState
          The updated battery state

        index : int
          The index of the BMS for the msg (in the case of multi-BMS systems)
        """

        self._last_bms_msg_time = rospy.get_time()
        with self._status_lock:
            self._bms_state[index] = msg


    def _msgChecker(self, event):
        """Check to see if messages have not been received within a certain amount of time"""

        now = rospy.get_time()
        if now - self._last_wireless_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new wireless message received in the last %0.1f seconds", self._no_new_msg_period)
        if now - self._last_bms_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new bms messages received in the last %0.1f seconds", self._no_new_msg_period)

    def report(self):
        """Logs all status information."""

        with self._status_lock:
            rospy.loginfo("  Platform:")
            rospy.loginfo("    Wireless connection: Bitrate %f Mbps; Link quality: %f" % (
                    self._wireless_connection.bitrate, self._wireless_connection.link_quality))
            for i in range(len(self._bms_state)):
                rospy.loginfo("    BMS[%d]: %f Volts" % (i, self._bms_state[i].voltage))
