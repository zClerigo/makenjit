#!/usr/bin/env python3
"""
DC Motor Control for Jetson Orin Nano using IRF510N MOSFETs
- Motor 1: GPIO pin 15 (PWM)
- Motor 2: GPIO pin 33 (PWM)
"""

import Jetson.GPIO as GPIO
import time

class MotorController:
    def __init__(self):
        # Define GPIO pins
        self.MOTOR1_PIN = 15  # PWM pin for motor 1
        self.MOTOR2_PIN = 33  # PWM pin for motor 2
        
        # PWM frequencies and initial duty cycles
        self.PWM_FREQ = 1000  # 1 kHz
        self.initial_duty_cycle = 0  # Start with motors off
        
        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)  # Use board pin numbering
        GPIO.setup(self.MOTOR1_PIN, GPIO.OUT)
        GPIO.setup(self.MOTOR2_PIN, GPIO.OUT)
        
        # Initialize PWM
        self.motor1_pwm = GPIO.PWM(self.MOTOR1_PIN, self.PWM_FREQ)
        self.motor2_pwm = GPIO.PWM(self.MOTOR2_PIN, self.PWM_FREQ)
        
        # Start PWM with motors off
        self.motor1_pwm.start(self.initial_duty_cycle)
        self.motor2_pwm.start(self.initial_duty_cycle)
        
        print("Motor controller initialized")
    
    def set_motor1_speed(self, speed_percent):
        """
        Set the speed of motor 1
        
        Args:
            speed_percent (float): Speed percentage (0-100)
        """
        # Ensure speed is within valid range
        speed_percent = max(0, min(100, speed_percent))
        self.motor1_pwm.ChangeDutyCycle(speed_percent)
        print(f"Motor 1 speed set to {speed_percent}%")
    
    def set_motor2_speed(self, speed_percent):
        """
        Set the speed of motor 2
        
        Args:
            speed_percent (float): Speed percentage (0-100)
        """
        # Ensure speed is within valid range
        speed_percent = max(0, min(100, speed_percent))
        self.motor2_pwm.ChangeDutyCycle(speed_percent)
        print(f"Motor 2 speed set to {speed_percent}%")
    
    def set_both_motors_speed(self, speed_percent):
        """
        Set the speed of both motors to the same value
        
        Args:
            speed_percent (float): Speed percentage (0-100)
        """
        self.set_motor1_speed(speed_percent)
        self.set_motor2_speed(speed_percent)
    
    def stop_motors(self):
        """Stop both motors"""
        self.set_both_motors_speed(0)
        print("Motors stopped")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_motors()
        self.motor1_pwm.stop()
        self.motor2_pwm.stop()
        GPIO.cleanup()
        print("GPIO resources cleaned up")


# Example usage
if __name__ == "__main__":
    try:
        # Create motor controller
        controller = MotorController()
        
        print("Running motor test sequence...")
        
        # Test motor 1
        print("Testing Motor 1...")
        controller.set_motor1_speed(50)  # 50% speed
        time.sleep(2)
        controller.set_motor1_speed(0)   # Stop
        time.sleep(1)
        
        # Test motor 2
        print("Testing Motor 2...")
        controller.set_motor2_speed(50)  # 50% speed
        time.sleep(2)
        controller.set_motor2_speed(0)   # Stop
        time.sleep(1)
        
        # Test both motors
        print("Testing both motors...")
        controller.set_both_motors_speed(75)  # 75% speed
        time.sleep(3)
        
        # Ramp up and down
        print("Ramping motors up and down...")
        for speed in range(0, 101, 10):
            controller.set_both_motors_speed(speed)
            time.sleep(0.5)
        for speed in range(100, -1, -10):
            controller.set_both_motors_speed(speed)
            time.sleep(0.5)
        
        # Stop motors and clean up
        controller.stop_motors()
        
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        if 'controller' in locals():
            controller.cleanup() 