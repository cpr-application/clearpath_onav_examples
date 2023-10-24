import yaml
import rospy
from clearpath_onav_api_examples_lib.mission import Mission


class YamlConfig:
    """Converts between Mission and yaml files."""

    @staticmethod
    def missionToYamlFile(filename, mission):
        """Converts a Mission object to a yaml file.

        Parameters
        ----------
        filename : str
            Full pathname for the output yaml file

        mission : Mission
            The mission to be saved as a yaml file
        """

        mission_yaml = {'mission': mission.toYaml()}
        with open(filename, 'w') as file:
            yaml.dump(mission_yaml, file)

    @staticmethod
    def yamlFileToMission(filename):
        """Creates a Mission object from a yaml file.

        Parameters
        ----------
        filename : str
            Full pathname of the yaml file to be read

        Returns
        -------
        Mission
            The created Mission object on success; None on any error
        """

        mission_yaml = None
        with open(filename, 'r') as file:
            try:
                mission_yaml = yaml.safe_load(file)
                if 'mission' not in mission_yaml:
                    rospy.logerr("yamlFileToMission: missing 'mission' field")
                    return None
                return Mission.fromYaml(mission_yaml['mission'])
            except yaml.YAMLError as e:
                rospy.logerr("Error while reading file: %s" % e)
        return None
