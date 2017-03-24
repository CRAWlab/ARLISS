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
from machine import UART
import time
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

    def get_one_frame(self):
        """
        Request one output frame - useful when continuous output is disabled and updates are
        required in larger intervals only. Though #f only requests one reply, replies are still
        bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
        """
        self.uart.write('#f')

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
                yaw, pitch, roll = data[5:-1].decode('utf-8').split(',')

                # Now convert the values from strings to floats
                yaw = float(yaw)
                pitch = float(pitch)
                roll = float(roll)

                return yaw, pitch, roll
        else:
            return None, None, None

    def gyro_read(self):
        self.uart.write('#osct')
        try:
            data = Razor.readline(0xD1)
            yaw_raw ,  pitch_raw,  roll_raw = data.decode('utf-8').split(',')
            yaw = int(yaw_raw.replace('x=', ''))
            pitch = int(pitch_raw.replace('y=', ''))
            roll = int(roll_raw.replace('z=', ''))
            return yaw,  pitch, roll
        except ValueError:
            pass
        except AttributeError:
            pass
            
    def accel_read(self):
        self.uart.write('#osct')
        try:
            data = Razor.readline(0xA7)
            x_raw ,  y_raw,  z_raw = data.decode('utf-8').split(',')
            x = int(x_raw.replace('x=', ''))
            y = int(y_raw.replace('y=', ''))
            z = int(z_raw.replace('z=', ''))
            return x, y, z
        except ValueError:
            pass
        except AttributeError:
            pass
    
## TODO: Implement binary output settings and reading
# "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
#         One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
# "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
#         One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
# "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
#         One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
