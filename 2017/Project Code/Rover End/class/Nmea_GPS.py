#Minimalistic NMEA-0183 message parser, based on micropyGPS
#\
#Created: 01/17
# -Peter Affolter
#Modified: 03/28/17
# -Joseph Fuentes
# -jafuentes3594@yahoo.com
import utime

class NmeaParser(object):
    """NMEA Sentence Parser. Creates object that stores all relevant GPS data and statistics.
    Parses sentences using update(). """
    
    def __init__(self):
        """Setup GPS Object Status Flags, Internal Data Registers, etc"""

        #####################
        # Data From Sentences
        # Time
        self.utc = (0)
        
        # Object Status Flags
        self.fix_time = 0
        self.valid_sentence = False

        # Position/Motion
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

        # GPS Info
        self.satellites_in_use = 0
        self.hdop = 0.0
        self.fix_stat = 0
        
        #raw data segments
        self.nmea_segments = []

    def update(self,  sentence):
        self.valid_sentence = False
        self.nmea_segments = str(sentence).split(',')
       
        #Parse GPGGA NMEA 2.0  
        #GGA is fix data for GPS
        if (self.nmea_segments[0] == "b'$GPGGA"):
            self.valid_sentence = True
            try:
                # UTC Timestamp
                 utc_string = self.nmea_segments[1]
                 
                # Skip timestamp if receiver doesn't have one yet
                 if utc_string:
                    hours = int(utc_string[0:2])
                    minutes = int(utc_string[2:4])
                    seconds = float(utc_string[4:])
                 else:
                    hours = 0
                    minutes = 0
                    seconds = 0.0
                    
                 # Number of Satellites in Use
                 satellites_in_use = int(self.nmea_segments[7])

                 # Horizontal Dilution of Positiom
                 hdop = float(self.nmea_segments[8])

                 # Get Fix Status
                 fix_stat = int(self.nmea_segments[6])
            except ValueError:
                return False
            
         # Process Location and Speed Data if Fix is GOOD
            if fix_stat==1:
                # Longitude / Latitude
                try:
                    # Latitude
                    lat_string = self.nmea_segments[2]
                    lat_degs = float(lat_string[0:2])
                    lat_mins = float(lat_string[2:])

                    # Longitude
                    lon_string = self.nmea_segments[4]
                    lon_degs = float(lon_string[0:3])
                    lon_mins = float(lon_string[3:])
                except ValueError:
                    return False
        
                # Altitude / Height Above Geoid
                try:
                    #Height above mean sea level
                    altitude = float(self.nmea_segments[9])
                    #Height of mean sea level
                    geoid_height = float(self.nmea_segments[11])
                except ValueError:
                    return False
                    
                # Update Object Data
                self.latitude = lat_degs + (lat_mins/60)
                self.longitude = lon_degs + (lon_mins/60)
                self.altitude = altitude
                self.geoid_height = geoid_height
                
            # Update Object Data
            self.timestamp = (hours, minutes, seconds)
            self.satellites_in_use = satellites_in_use
            self.hdop = hdop
            self.fix_stat = fix_stat
        
            # If Fix is GOOD, update fix timestamp
            if fix_stat==1:
                self.fix_time = utime.time()

            return True
