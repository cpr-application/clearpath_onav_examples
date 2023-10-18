import rospy


class RosNode:
    """Initializes the ROS node"""

    def __init__(self, node_name, log_level=rospy.INFO):
        """Initializes the ROS node.

        Parameters
        ----------
        node_name : str
            The name of the ROS node to be initialized
        """

        self._node_name = node_name
        rospy.init_node(node_name, log_level=log_level)
