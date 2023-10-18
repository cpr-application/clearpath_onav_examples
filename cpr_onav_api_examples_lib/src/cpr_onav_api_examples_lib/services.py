import rospy
from clearpath_control_msgs.srv import SetControlMode
from clearpath_control_msgs.msg import ControlMode
from std_srvs.srv import SetBool
from clearpath_localization_msgs.srv import SetDatum


# The name of the service to set the operating mode
SET_MODE_SERVICE = 'control_selection/set_mode'
# The name of the service to 'pause' missions
PAUSE_SERVICE = 'control_selection/autonomy_pause'
# The name of the service to 'resume' missions
RESUME_SERVICE = 'control_selection/autonomy_resume'
# The name of the service to set the datum
SET_DATUM_SERVICE = 'localization/set_datum'
# The name of the service to set obstacle avoidance
SET_OBSTACLE_AVOIDANCE_SERVICE = 'navigation/set_obstacle_avoidance'


class Services:
    """Encapsulates the services provided by the Autonomy API."""

    @staticmethod
    def setMode(mode):
        """Sets the control mode (NEUTRAL, MANUAL, or AUTONOMY).

        Parameters
        ----------
        mode : int8 (enum)
            The current control mode (NEUTRAL, MANUAL, or AUTONOMY)

        Returns
        -------
        bool
            True if the mode was set successfully, else False
        """

        control_mode = ControlMode()
        control_mode.mode = mode
        rospy.wait_for_service(SET_MODE_SERVICE, timeout=2.0)
        try:
            set_mode_service = rospy.ServiceProxy(SET_MODE_SERVICE, SetControlMode)
            set_mode_service(control_mode)    # does not return a value; assume success
            return True
        except rospy.ServiceException as e:
            rospy.logerr("setMode Service call failed: %s" % e)
            return False

    @staticmethod
    def pauseMission():
        """Pauses the mission that is currently executing.

        To resume the mission, see resumeMission().
        To stop the mission, see stopMission().

        Returns
        -------
        bool
            True if the mission was paused successfully, else False
        """

        rospy.wait_for_service(PAUSE_SERVICE, timeout=2.0)
        try:
            pause_service = rospy.ServiceProxy(PAUSE_SERVICE, SetBool)
            resp = pause_service(True)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("pauseMission Service call failed: %s" % e)
            return False

    @staticmethod
    def resumeMission():
        """Resumes the mission that was previously paused with pauseMission().

        Returns
        -------
        bool
            True if the mission was resumed successfully, else False
        """

        rospy.wait_for_service(RESUME_SERVICE, timeout=2.0)
        try:
            resume_service = rospy.ServiceProxy(RESUME_SERVICE, SetBool)
            resp = resume_service(True)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("resumeMission Service call failed: %s" % e)
            return False

    @staticmethod
    def setDatum(coordinate_lat_lon):
        """Sets the datum position.

        Parameters
        ----------
        coordinate_lat_lon : CoordinateLatLon
            The coordinate of the datum

        Returns
        -------
        bool
            True if the datum was set successfully, else False
        """

        rospy.wait_for_service(SET_DATUM_SERVICE, timeout=2.0)
        try:
            set_datum_service = rospy.ServiceProxy(SET_DATUM_SERVICE, SetDatum)
            resp = set_datum_service(
                    float(coordinate_lat_lon.getLat()), float(coordinate_lat_lon.getLon()))
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("setDatum Service call failed: %s" % e)
            return False

    @staticmethod
    def setObstacleAviodance(enable_avoidance):
        """Enables/disables obstacle avoidance.

        Parameters
        ----------
        enable_avoidance : bool
            If True, enable obstacle avoidance; if False, disable obstacle avoidance

        Returns
        -------
        bool
            True if obstacle avoidance was updated successfully, else False
        """

        rospy.wait_for_service(SET_OBSTACLE_AVOIDANCE_SERVICE, timeout=2.0)
        try:
            set_avoidance_service = rospy.ServiceProxy(SET_OBSTACLE_AVOIDANCE_SERVICE, SetBool)
            resp = set_avoidance_service(enable_avoidance)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("setObstacleAviodance Service call failed: %s" % e)
            return False
