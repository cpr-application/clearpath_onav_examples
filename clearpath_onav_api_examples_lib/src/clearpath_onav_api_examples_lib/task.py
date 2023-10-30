import rospy
import clearpath_navigation_msgs.msg


class Task:
    """A single Task (part of a Waypoint).

    Contains a name, uuid, action_server_name, version, arrays of floats
    and strings related to the specific Task.
    """

    def __init__(self, name, uuid, action_server_name, version="", floats=[], strings=[]):
        """Creates a Waypoint object from the given parameters.

        Parameters
        ----------
        name : str
            The name or ID of the task, used for tracking/reporting

        uuid : str
            Unique UUID string for the task

        action_server_name : str
            The ROS action that this task executes

        floats : floats[], optional
            Numerical/boolean data to be passed to the action_server_name
            The exact meaning of these values is dependent on the underlying service

        strings : str[], optional
            String data to be passed to the action_server_name
            The exact meaning of these values is dependent on the underlying service
        """

        self._task_msg = clearpath_navigation_msgs.msg.Task()
        self._task_msg.name = name
        self._task_msg.uuid = uuid
        self._task_msg.action_server_name = action_server_name
        self._task_msg.version = version
        self._task_msg.floats = floats
        self._task_msg.strings = strings

    def getName(self):
        """Gets the name of the task.

        Returns
        -------
        str
            the name of the task
        """

        return self._task_msg.name

    def getTaskMsg(self):
        """Gets the task message.

        Returns
        -------
        clearpath_navigation_msgs.msg.Task
            the task message
        """

        return self._task_msg

    def toYaml(self):
        """Converts the task to YAML format, for writing to disk.

        Returns
        -------
        dict
            A YAML-compatible dictionary representation of the object.
        """

        tasks_yaml = []
        for task in self._task_msg.tasks:
            tasks_yaml.append({
                'name': task.name,
                'uuid': task.uuid,
                'action_server_name': task.action_server_name,
                'version': task.version,
                'floats': task.floats,
                'strings': task.strings
            })
        return tasks_yaml

    @staticmethod
    def fromYaml(task_yaml):
        """Creates a Task object from a YAML-compatible dictionary representation.

        Parameters
        ----------
        task_yaml : dict
            The YAML-compatible dictionary representation of a Task object
        """

        try:
            name = task_yaml['name']
            uuid = task_yaml['uuid']
            action_server_name = task_yaml['action_server_name']
            version = task_yaml['version']
            floats = task_yaml['floats']
            strings = task_yaml['strings']
            return Task(name, uuid, action_server_name, version, floats, strings)

        except KeyError as e:
            rospy.logerr('Unable to parse yaml for tasks: %s' % e)
        return None
