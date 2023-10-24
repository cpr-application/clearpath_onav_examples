import rospy
from pyproj import Proj
import math
from clearpath_onav_api_examples_lib.coordinate_utm import CoordinateUtm
from clearpath_onav_api_examples_lib.coordinate_xy import CoordinateXY


class CoordinateLatLon:
    """A coordinate in Latitude/Longitude format"""

    def __init__(self, lat, lon):
        """Initialize the coordinate from the parameters

        Parameters
        ----------
        lat : float
            Latitude value (-90.0 to +90.0)

        lon : float
            Longitude value (-180.0 to + 180.0)
        """

        self.lat = lat
        self.lon = lon

    def __repr__(self):
        """String representation of the class."""

        return "Latitude: " + str(self.lat) + " Longitude: " + str(self.lon)

    def getLat(self):
        """Get the latitude value of the coordinate

        Returns
        -------
        float
            latitude
        """
        return self.lat

    def getLon(self):
        """Get the longitude value of the coordinate

        Returns
        -------
        float
            longitude
        """
        return self.lon

    def toUtm(self):
        """Gets the coordinate in UTM format

        Returns
        -------
        CoordinateUtm
            coordinate in UTM format
        """

        utm_zone_number, utm_zone_letter = self.getUtmZone()
        projector = Proj(proj='utm', zone=utm_zone_number, ellps='WGS84')
        easting,northing = projector(self.lon, self.lat)
        return CoordinateUtm(northing, easting, utm_zone_number, utm_zone_letter)

    @staticmethod
    def fromUtm(coordinate_utm):
        """Converts from UTM to latitude/longitude

        Returns
        -------
        CoordinateLatLon
            A coordinate in latitude/longitude format.
        """

        projector = Proj(proj='utm', zone=coordinate_utm.getZoneNumber(), ellps='WGS84')
        coord_latlon = projector(coordinate_utm.getEast(), coordinate_utm.getNorth(), inverse=True)
        return CoordinateLatLon(coord_latlon[1], coord_latlon[0])

    def toXY(self, origin):
        """Get the X and Y offsets of the coordinate from the specified origin

        Parameters
        ----------
        origin : CoordinateLatLon
            origin for the computed X,Y points

        Returns
        -------
        CoordinateXY
            coordinate in XY format
        """

        coord_utm = self.toUtm()
        origin_utm = origin.toUtm()
        # If the zones don't match, return None
        if (
            coord_utm.getZoneNumber() != origin_utm.getZoneNumber() or
            coord_utm.getZoneletter() != origin_utm.getZoneletter()
        ):
            print(coord_utm.getZoneNumber())
            print(origin_utm.getZoneNumber())
            print(origin_utm)
            return None
        x = coord_utm.getEast() - origin_utm.getEast()
        y = coord_utm.getNorth() - origin_utm.getNorth()
        return CoordinateXY(x, y)

    @staticmethod
    def fromXY(coordinate_xy, origin):
        """Convert the XY coordinate into a latitude/longitude one.

        Parameters
        ----------
        coordinate_xy: Coordinate_XY
            coordinate to be converted from XY to latitude/longitude
        origin : CoordinateLatLon
            origin for coordinate_xy
        """

        origin_utm = origin.toUtm()
        east = origin_utm.getEast() + coordinate_xy.getX()
        north = origin_utm.getNorth() + coordinate_xy.getY()
        coord_utm = CoordinateUtm(north, east, origin_utm.getZoneNumber(),
                                  origin_utm.getZoneLetter())
        return CoordinateLatLon.fromUtm(coord_utm)

    def toYaml(self):
        """Converts the object to YAML format, for writing to disk.

        Returns
        -------
        dict
            A YAML-compatible dictionary representation of the object.
        """

        coordinate_yaml = {
            'latitude': self.lat,
            'longitude': self.lon
        }
        return coordinate_yaml

    @staticmethod
    def fromYaml(coordinate_yaml):
        """Creates an object from a YAML-compatible dictionary representation.

        Returns
        -------
        CoordinateLatLon
            A coordinate in latitude/longitude format.
        """

        try:
            latitude = coordinate_yaml['latitude']
            longitude = coordinate_yaml['longitude']
            return CoordinateLatLon(latitude, longitude)
        except KeyError as e:
            rospy.logerr('Unable to parse yaml for coordinate: %s' % e)
        return None

    def getUtmZone(self):
        zone = math.floor((self.lon/6)+31)
        if (self.lat < -72):
            letter='C'
        elif (self.lat < -64):
            letter='D'
        elif (self.lat < -56):
            letter='E'
        elif (self.lat < -48):
            letter='F'
        elif (self.lat < -40):
            letter='G'
        elif (self.lat < -32):
            letter='H'
        elif (self.lat < -24):
            letter='J'
        elif (self.lat < -16):
            letter='K'
        elif (self.lat < -8):
            letter='L'
        elif (self.lat < 0):
            letter='M'
        elif (self.lat < 8):
            letter='N'
        elif (self.lat < 16):
            letter='P'
        elif (self.lat < 24):
            letter='Q'
        elif (self.lat < 32):
            letter='R'
        elif (self.lat < 40):
            letter='S'
        elif (self.lat < 48):
            letter='T'
        elif (self.lat < 56):
            letter='U'
        elif (self.lat < 64):
            letter='V'
        elif (self.lat < 72):
            letter='W'
        else:
            letter='X'
        return zone, letter