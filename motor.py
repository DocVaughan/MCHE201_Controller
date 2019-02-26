###############################################################################
# motor.py
#
# Script for using a PCA9685 to control DC motors. This is configured for use
# on the MCHE201 breakout board to control several motors. On the MCHE201 
# controller board, the motors are driven with DRV8871 motor drivers. These
# drivers have two inputs. One input should be controlled via PWM to set the 
# speed and the other pulled low. To switch directions, we do the opposite.
#
# This code is modified from that created by Adafruit, which was MIT licensed. 
# Per the MIT license, the original License is provided in this repository.
#
# Created: 11/02/18
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 02/25/19 - JEV
#       - updated i2c address to match v2 of MCHE201 board
#
# TODO:
#   * 
###############################################################################

import pca9685


_DC_MOTORS = ((2, 3), (4, 5))


class DCMotors:
    def __init__(self, i2c, address=0x60, freq=1600):
        self.pca9685 = pca9685.PCA9685(i2c, address)
        self.pca9685.freq(freq)

    def _pin(self, pin, value=None):
        if value is None:
            return bool(self.pca9685.pwm(pin)[0])
        if value:
            self.pca9685.pwm(pin, 4096, 0)
        else:
            self.pca9685.pwm(pin, 0, 0)

    def set_speed(self, motor_number, speed=None):
        """
        Function to set the speed of the motor
        
        Arguments:
            motor_number : the number of the motor to control, either 1 or 2
            speed : the speed +/-100% to run the motor
        
        Returns:
            N/A
        """
        
        # Get the pin numbers for the selected motor
        # index-1 because list of pins is 0 indexed
        in1, in2 = _DC_MOTORS[motor_number-1] 
        
        # Concert the % speed to 0-4095
        duty_cycle_value = int(speed/100 * 4095)
        
        if duty_cycle_value > 0:
            # Forward
            self.pca9685.duty(in1, abs(duty_cycle_value))
            self._pin(in2, False)
        
        elif duty_cycle_value < 0:
            # Backward
            self._pin(in1, False)
            self.pca9685.duty(in2, abs(duty_cycle_value))
        
        else:
            # Release
            self._pin(in1, False)
            self._pin(in2, False)

    def brake(self, index):
        in1, in2 = _DC_MOTORS[motor_number-1]
        
        self._pin(in1, True)
        self._pin(in2, True)
