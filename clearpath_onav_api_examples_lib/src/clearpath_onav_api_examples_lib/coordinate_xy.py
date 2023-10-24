class CoordinateXY:
    """A coordinate in XY format, relative to a fixed origin"""

    def __init__(self, x, y):
        """Initialize the coordinate from the parameters

        Parameters
        ----------
        x : float
            X offset, in meters, from the origin

        east : float
            Y offset, in meters, from the origin
        """

        self.x = x
        self.y = y

    def getX(self):
        """Get the X offset

        Returns
        -------
        float
          X offset from the origin, in meters
        """

        return self.x

    def getY(self):
        """Get the Y offset

        Returns
        -------
        float
          Y offset from the origin, in meters
        """

        return self.y
