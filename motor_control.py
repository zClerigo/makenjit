#!/usr/bin/env python3
"""
DC Motor Control for Nvidia Jetson Orin Nano
Controls two DC motors using PWM on GPIO pins 33 and 15
"""

import Jetson.GPIO as GPIO
import time
import signal
import sys

# Define GPIO pins
MOTOR1_PIN = 33  # PWM pin for motor 1
MOTOR2_PIN = 15  # PWM pin for motor 2

# PWM frequency (Hz)
PWM_FREQ = 1000

# Initialize GPIO
def setup():
    # Set GPIO mode to BOARD (pin numbers)
    GPIO.setmode(GPIO.BOARD)
    
    # Set up the GPIO pins as outputs
    GPIO.setup(MOTOR1_PIN, GPIO.OUT)
    GPIO.setup(MOTOR2_PIN, GPIO.OUT)
    
    # Create PWM objects
    motor1_pwm = GPIO.PWM(MOTOR1_PIN, PWM_FREQ)
    motor2_pwm = GPIO.PWM(MOTOR2_PIN, PWM_FREQ)
    
    # Start PWM with 0% duty cycle (motors off)
    motor1_pwm.start(0)
    motor2_pwm.start(0)
    
    return motor1_pwm, motor2_pwm

# Set motor speed (0-100%)
def set_motor_speed(motor_pwm, speed):
    # Ensure speed is within valid range
    if speed < 0:
        speed = 0
    elif speed > 100:
        speed = 100
    
    # Set PWM duty cycle
    motor_pwm.ChangeDutyCycle(speed)

# Clean up GPIO on exit
def cleanup(*args):
    print("Cleaning up GPIO...")
    GPIO.cleanup()
    sys.exit(0)

# Main function
def main():
    # Register signal handler for clean exit
    signal.signal(signal.SIGINT, cleanup)
    
    # Setup GPIO and get PWM objects
    motor1_pwm, motor2_pwm = setup()
    
    try:
        print("DC Motor Control - Press Ctrl+C to exit")
        
        while True:
            # Example: Gradually increase speed of motor 1
            print("Accelerating motor 1...")
            for speed in range(0, 101, 5):
                set_motor_speed(motor1_pwm, speed)
                time.sleep(0.1)
            
            time.sleep(1)
            
            # Example: Gradually decrease speed of motor 1
            print("Decelerating motor 1...")
            for speed in range(100, -1, -5):
                set_motor_speed(motor1_pwm, speed)
                time.sleep(0.1)
            
            time.sleep(1)
            
            # Example: Gradually increase speed of motor 2
            print("Accelerating motor 2...")
            for speed in range(0, 101, 5):
                set_motor_speed(motor2_pwm, speed)
                time.sleep(0.1)
            
            time.sleep(1)
            
            # Example: Gradually decrease speed of motor 2
            print("Decelerating motor 2...")
            for speed in range(100, -1, -5):
                set_motor_speed(motor2_pwm, speed)
                time.sleep(0.1)
            
            time.sleep(1)
            
            # Example: Run both motors at different speeds
            print("Running both motors...")
            set_motor_speed(motor1_pwm, 75)
            set_motor_speed(motor2_pwm, 50)
            time.sleep(3)
            
            # Stop both motors
            print("Stopping motors...")
            set_motor_speed(motor1_pwm, 0)
            set_motor_speed(motor2_pwm, 0)
            time.sleep(2)
            
    except KeyboardInterrupt:
        # This will be caught by the signal handler
        pass
    finally:
        # Ensure cleanup happens
        cleanup()

if __name__ == "__main__":
    main() 