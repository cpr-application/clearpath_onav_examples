class CoordinateUtm:
    """A coordinate in UTM format"""

    def __init__(self, north, east, zone_number, zone_letter):
        """Initialize the coordinate from the parameters

        Parameters
        ----------
        north : float
            North offset, in meters, within the zone

        east : float
            East offset, in meters, within the zone

        zone_number : int
            The UTM zone number (1 to 60)

        zone_letter : string
            The UTM zone letter (C to X)
        """

        self.north = north
        self.east = east
        self.zone_number = zone_number
        self.zone_letter = zone_letter

    def getNorth(self):
        """Get the North offset

        Returns
        -------
        float
          North offset within the zone, in meters
        """

        return self.north

    def getEast(self):
        """Get the East offset

        Returns
        -------
        float
          East offset within the zone, in meters
        """

        return self.east

    def getZoneNumber(self):
        """Get the zone number

        Returns
        -------
        int
          UTM zone number (1 to 60)
        """

        return self.zone_number

    def getZoneLetter(self):
        """Get the zone letter

        Returns
        -------
        string
          UTM zone letter (C to X)
        """

        return self.zone_letter
