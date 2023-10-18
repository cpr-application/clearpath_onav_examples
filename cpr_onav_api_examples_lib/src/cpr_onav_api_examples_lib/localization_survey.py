import rospy
import actionlib
import clearpath_localization_msgs.msg


# The name of the action server for surveying.
# This needs to match the CPR OutdoorNav API.
SURVEY_ACTION_NAME = 'localization/survey'


class LocalizationSurvey:
    """Performs surveying, prior to running missions.

    This is typically used to initialize the position of the Base Station.
    """

    def __init__(self, progress_callback=None, done_callback=None):
        """Builds up the data structure containing the details of the surveying.

        Parameters
        ----------
        progress_callback : fn(float32), optional
            A callback function that is called at regular intervals while the surveying
            is being executed to provide progress updates. The function should take
            float32 parameter that is the percent complete. If set to None, no feedback
            will be provided and the surveying will execute to completion.

        done_callback : fn(enum, bool), optional
            A callback function that is called when the surveying is complete. The function
            should take two parameters: the terminal state (as an integer from
            actionlib_msgs/GoalStatus) and the result.
        """

        self._survey_complete = False
        self._survey_success = False
        self._survey_progress_callback = progress_callback
        self._survey_done_callback = done_callback
        self._client = actionlib.SimpleActionClient(
                SURVEY_ACTION_NAME, clearpath_localization_msgs.msg.SurveyBaseStationAction)

    def _surveyFeedbackCallback(self, feedback):
        """Executes when 'feedback' is received from the surveying.

        Forwards the callback to the survey feedback callback (if provided).
        """

        rospy.logdebug("Surveying is %f percent complete" % (feedback.percent_complete))
        if self._survey_progress_callback is not None:
            self._survey_progress_callback(feedback)

    def _surveyDoneCallback(self, goal_state, result):
        """Executes when 'done' is received from the surveying.

        Forwards the callback to the survey feedback callback (if provided).
        """

        if goal_state == actionlib.GoalStatus.SUCCEEDED and result.success:
            self._survey_success = True
        else:
            self._survey_success = False
        self._survey_complete = True
        rospy.logdebug("Surveying is complete. Success: %s" % (self._survey_success))
        if self._survey_done_callback is not None:
            self._survey_done_callback(goal_state, result)

    def startSurvey(self, number_of_desired_fixes=2000):
        """Begin the survey.

        Using the default value, this process will take approximately 10 minutes.

        Parameters
        ----------
        number_of_desired_fixes : int, optional
            The number of sample to take during the surveying process (default is 2000)
        """

        self._survey_success = False
        self._survey_complete = False
        try:
            # Waits until the action server has started up and started
            # listening for goals, then sends the goal
            if not self._client.wait_for_server(timeout=rospy.Duration(2.0)):
                rospy.logerr("Failed to connect to server")
                return False
            goal = {number_of_desired_fixes: number_of_desired_fixes}
            self._client.send_goal(goal,
                                   feedback_cb=self._surveyFeedbackCallback,
                                   done_cb=self._surveyDoneCallback)
        except rospy.ROSInterruptException:
            rospy.logerr("localization/survey server is not available; exception detected")
            return False
        return True

    def cancelSurvey(self):
        """Cancels the active survey."""

        self._client.cancel_goal()

    def getSurveySuccess(self):
        """Determines if the survey was successful.

        If the survey is still in progress, reports False.

        See also isSurveyComplete().

        Returns
        -------
        bool
            True if the survey completed successfully, else False
        """

        return self._survey_success

    def isSurveyComplete(self):
        """Determines if the survey is complete.

        See also getSurveySuccess().

        Returns
        -------
        bool
            True if the survey is complete, else False if survey is still running
        """

        return self._survey_complete
