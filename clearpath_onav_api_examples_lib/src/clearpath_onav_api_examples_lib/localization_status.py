import rospy
from threading import Lock
from nav_msgs.msg import Odometry
from clearpath_localization_msgs.msg import LocalizationStatus


# The name of the topic for odometry. This needs to match the CPR OutdoorNav API.
ODOM_TOPIC_NAME = "/localization/odom"
# The name of the topic for status. This needs to match the CPR OutdoorNav API.
STATUS_TOPIC_NAME = "/localization/status"


class LocalizationStatusMonitor:
    """Create ROS subscribers for localization status topics and save the results."""

    def __init__(self, store_data=False, msg_warn_period=10.0):
        """Subscribes for updates to the various localization topics"""

        self._store_data = store_data
        self._status_lock = Lock()

        self._odom = Odometry()
        self._odom_sub = rospy.Subscriber(
                ODOM_TOPIC_NAME,
                Odometry,
                self._odomCallback)

        self._status = LocalizationStatus()
        self._status_sub = rospy.Subscriber(
                STATUS_TOPIC_NAME,
                LocalizationStatus,
                self._statusCallback)

        self._no_new_msg_period = msg_warn_period
        self._last_odom_msg_time = rospy.get_time()
        self._last_status_msg_time = rospy.get_time()
        self._timer = rospy.Timer(period=rospy.Duration(0.2), callback=self._msgChecker, oneshot=False)

        self._odom_path = {
            "x": [],
            "y": [],
        }

    def _odomCallback(self, msg):
        """Updates the odom state.

        Parameters
        ----------
        msg : nav_msgs.msg.Odometry
          The updated odom state
        """

        self._last_odom_msg_time = rospy.get_time()
        with self._status_lock:
            self._odom = msg
            if self._store_data:
                self._odom_path["x"].append(self._odom.pose.pose.position.x)
                self._odom_path["y"].append(self._odom.pose.pose.position.y)

    def _statusCallback(self, msg):
        """Updates the GPS status state.

        Parameters
        ----------
        msg : clearpath_localization_msgs.msg.LocalizationStatus
          The updated GPS status state
        """

        self._last_status_msg_time = rospy.get_time()
        with self._status_lock:
            self._status = msg

    def _msgChecker(self, event):
        """Check to see if messages have not been received within a certain amount of time"""

        now = rospy.get_time()
        if now - self._last_odom_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new odometry message received in the last %0.1f seconds", self._no_new_msg_period)
        if now - self._last_status_msg_time > self._no_new_msg_period:
            rospy.logwarn_throttle(self._no_new_msg_period, "No new status message received in the last %0.1f seconds", self._no_new_msg_period)

    def report(self):
        """Logs all localization information."""

        with self._status_lock:
            rospy.loginfo("  Localization:")
            rospy.loginfo("    Odom: Pose (%f, %f, %f) Orientation (%f, %f, %f, %f) " % (
                    self._odom.pose.pose.position.x,
                    self._odom.pose.pose.position.y,
                    self._odom.pose.pose.position.z,
                    self._odom.pose.pose.orientation.x,
                    self._odom.pose.pose.orientation.y,
                    self._odom.pose.pose.orientation.z,
                    self._odom.pose.pose.orientation.w))
            rospy.loginfo("    Status:")
            rospy.loginfo("      Accuracy: %f" % self._status.accuracy)
            rospy.loginfo("      GPS Position Num Connected Sats: %d" %
                          self._status.position_gnss.num_connected_sats)
            rospy.loginfo("      GPS Heading Num Connected Sats: %d" %
                          self._status.heading_gnss.num_connected_sats)
            rospy.loginfo("      GPS Base Station Num Connected Sats: %d" %
                          self._status.base_station_gnss.num_connected_sats)
            rospy.loginfo("      GPS Dead Reckoning: %d" %
                          self._status.dead_reckoning)
