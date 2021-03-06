###############################################################################
# pyboard_RazorIMU.py
#
# Script demonstrating using the serial communication on the pyboard
# to receive data and set settings on the SparkFun Razor IMU
# https://www.sparkfun.com/products/10736
#
# This code assumes that the razor is running the firmware found at:
# https://github.com/ptrbrtz/razor-9dof-ahrs/blob/master/Arduino/Razor_AHRS/Razor_AHRS.ino
#
#
# Created: 09/14/16
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   *
#
# TODO:
#   * 09/14/16 - Implement binary parsing, only text parsing so far
#   * 09/14/16 - Implement parsing of data in calibrated and raw modes
#   * 09/14/16 - Improve parsing of data in angle mode
###############################################################################

import pyb # Is it bad practice to import the entire pyboard module?
import re  # Regular expressions for parsing IMU data

class Razor(object):
    """ Class of convenience methods for the RAZOR IMU """

    def __init__(self, port, baudrate, bits=8, parity=None, stop=1, read_buf_len = 512):
        # Set up the UART communications
        self.port = port
        self.baudrate = baudrate
        self.uart = pyb.UART(port, baudrate)

        # Start with streaming off
        self.streaming = False
        self.uart.write('#o0') # UART command to disable streaming

        # Set the default mode to return angles in text format
        self.mode = 'angles'
        self.uart.write('#ot')

        self.x_regex = re.compile('x=\W+\d+')
        self.y_regex = re.compile('y=\W+\d+')
        self.z_regex = re.compile('z=\W+\d+')

    def stop_streaming(self):
        """
        method to stop streaming data from the sensor. Good for use with the
        get_one_frame() method below for infrequent readings from the sensor
        The status LED will be off.
        """
        self.uart.write('#o0')
        self.streaming = False

    def start_streaming(self):
        """
        Method to start streaming data from the sensor
        The status LED will be on if streaming output is enabled.
        """
        self.uart.write('#o1')
        self.streaming = True

    def set_angle_output(self):
        """
        Output angles in TEXT format (Output frames have form like
        "#YPR=-142.28,-5.38,33.52", followed by carriage return and line feed [\r\n]).
        """
        self.uart.write('#ot')
        self.mode = 'angles'

    def set_all_calibrated_output(self):
        """
        Set output to all CALIBRATED SENSOR data of all 9 axes in TEXT format.
        One frame consist of three lines - one for each sensor: acc, mag, gyr.
        """
        self.uart.write('#osct')
        self.mode = 'calibrated'

    def set_all_raw_output(self):
        """
        Output RAW SENSOR data of all 9 axes in TEXT format.
        One frame consist of three lines - one for each sensor: acc, mag, gyr.
        """
        self.uart.write('#osrt')
        self.mode = 'raw'

    def get_latest_angle_data(self):
        """
        Function to get the latest data for an IMU set up in streaming modes

        Returns:
            x_angle, y_angle, z_angle data from the IMU, None is returned if match is not found
        """

        data = self.uart.readline()
        print('\033[2J\033[;H')
        print('data = {}'.format(data))

        x_angle_matches = self.x_regex.search(data)
        y_angle_matches = self.y_regex.search(data)
        z_angle_matches = self.z_regex.search(data)

        if x_angle_matches is not None:
            x_angle = int(x_angle_matches.group(0)[3:])
        else:
            x_angle = None

        if y_angle_matches is not None:
            y_angle = int(y_angle_matches.group(0)[3:])
        else:
            y_angle = None

        if z_angle_matches is not None:
            z_angle = int(z_angle_matches.group(0)[3:])
        else:
            z_angle = None

        #print('\033[2J\033[;H')
        print(x_angle)
        print('x= {} y= {} z= {}'.format(x_angle,y_angle,z_angle))


        return (x_angle, y_angle, z_angle)



    def get_one_frame(self):
        """
        Request one output frame - useful when continuous output is disabled and updates are
        required in larger intervals only. Though #f only requests one reply, replies are still
        bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
        """
        self.uart.write('#f') # JEV - 09/12/18 - I don't think we need to ask for a frame if streaming


        if not self.streaming:
            data = self.uart.readline()

        if data:
            if self.mode == 'calibrated':
                pass
                # TODO: Finish regular expression based parsing of this
                # We need regular expressions because we're not guaranteed the
                # order that accel, gyro, and magnometer data is returned in
            elif self.mode == 'raw':
                pass
                # TODO: Finish regular expression based parsing of this
                # We need regular expressions because we're not guaranteed the
                # order that accel, gyro, and magnometer data is returned in
            elif self.mode == 'angles':
                # TODO: Finish regular expression based parsing of this
                # This removing the first set of characters then using split is
                # not very robust

                # The data returned in this mode matches the form
                #   '#YPR=-35.87,26.25,0.26\r\n'
                # So, we'll ignore the first 5 characters, then split at the commas
                # Afterwards, we can convert the values to floats

                yaw, pitch, roll = data[4:-1].decode('utf-8').split(',')


                # Now convert the values from strings to floats
                yaw = float(yaw)
                pitch = float(pitch)
                roll = float(roll)

                return yaw, pitch, roll
        else:
            return None, None, None



## TODO: Implement binary output settings and reading
# "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
#         One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
# "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
#         One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
# "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
#         One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
