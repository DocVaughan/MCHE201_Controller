###############################################################################
# pca9685.py
#
# Script for using a PCA9685 to control the DC motor of a linear actuator.
# This is configured for use on the MCHE201 breakout board. On the MCHE201 
# controller board, the linear actuator is driven with a DRV8871 motor driver,
# which has two inputs. One input should be controlled via PWM to set the 
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

class LinearActuator:
    def __init__(self, i2c, address=0x60, freq=1600):
        self.pca9685 = pca9685.PCA9685(i2c, address)
        self.pca9685.freq(freq)
        
        self.speed = 0
        
        # Set the pin numbers for the selected motor
        # These are hard coded here, since here is only 1 linear actuator
        self.in1 = 6
        self.in2 = 7

    def _pin(self, pin, value=None):
        if value is None:
            return bool(self.pca9685.pwm(pin)[0])
        if value:
            self.pca9685.pwm(pin, 4096, 0)
        else:
            self.pca9685.pwm(pin, 0, 0)

    def set_speed(self, speed=0):
        """
        Function to set the speed of the linear actuator's motor
        
        Arguments:
            speed : the speed +/-100% to run the actuator
        
        Returns:
            N/A
        """
        
        self.speed = speed
        
        if speed > 0:             # Forward
            # Correct for the dead-zone from +/-50% duty cycle
            speed = 0.5 * speed + 50
        
            # Concert the % speed to 0-4095
            duty_cycle_value = int(speed/100 * 4095)    
            
            self.pca9685.duty(self.in1, abs(duty_cycle_value))
            self._pin(self.in2, False)
        
        elif speed < 0:            # Backward
            # Correct for the dead-zone from +/-50% duty cycle
            speed = 0.5 * speed - 50
        
            # Concert the % speed to 0-4095
            duty_cycle_value = int(speed/100 * 4095)
            
            self._pin(self.in1, False)
            self.pca9685.duty(self.in2, abs(duty_cycle_value))
        
        else:
            # Release
            self._pin(self.in1, False)
            self._pin(self.in2, False)
        

    def brake(self, index):
        self._pin(self.in1, True)
        self._pin(self.in2, True)
