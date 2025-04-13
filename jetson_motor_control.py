#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time

class MotorController:
    def __init__(self, motor1_pin, motor2_pin):
        """
        Initialize the motor controller with GPIO pins for two motors.
        
        Args:
            motor1_pin: GPIO pin connected to MOSFET controlling motor 1
            motor2_pin: GPIO pin connected to MOSFET controlling motor 2
        """
        self.motor1_pin = motor1_pin
        self.motor2_pin = motor2_pin
        
        # Set up GPIO
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setup(self.motor1_pin, GPIO.OUT)
        GPIO.setup(self.motor2_pin, GPIO.OUT)
        
        # Create PWM objects for speed control (frequency: 100Hz)
        self.motor1_pwm = GPIO.PWM(self.motor1_pin, 100)
        self.motor2_pwm = GPIO.PWM(self.motor2_pin, 100)
        
        # Start PWM with 0% duty cycle (motors off)
        self.motor1_pwm.start(0)
        self.motor2_pwm.start(0)
        
    def set_motor_speed(self, motor_num, speed):
        """
        Set the speed of a specific motor.
        
        Args:
            motor_num: Motor number (1 or 2)
            speed: Speed value (0-100)
        """
        # Ensure speed is within valid range
        speed = max(0, min(100, speed))
        
        if motor_num == 1:
            self.motor1_pwm.ChangeDutyCycle(speed)
        elif motor_num == 2:
            self.motor2_pwm.ChangeDutyCycle(speed)
        else:
            print(f"Invalid motor number: {motor_num}")
    
    def move_forward(self, speed=50):
        """Move both motors forward at the specified speed."""
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)
        
    def move_backward(self, speed=50):
        """
        Move both motors backward.
        
        Note: For simple DC motors with MOSFET control, backward motion
        would typically require an H-bridge. This implementation assumes
        you have the appropriate hardware to reverse motor direction.
        """
        # If using an H-bridge or motor driver that supports direction control,
        # you would implement the direction control logic here
        print("Warning: Backward motion requires H-bridge or motor driver with direction control")
        
    def turn_left(self, speed=50):
        """Turn left by running right motor faster than left motor."""
        self.set_motor_speed(1, speed/2)  # Left motor slower
        self.set_motor_speed(2, speed)    # Right motor faster
        
    def turn_right(self, speed=50):
        """Turn right by running left motor faster than right motor."""
        self.set_motor_speed(1, speed)    # Left motor faster
        self.set_motor_speed(2, speed/2)  # Right motor slower
        
    def stop(self):
        """Stop both motors."""
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)
        
    def cleanup(self):
        """Clean up GPIO resources."""
        self.motor1_pwm.stop()
        self.motor2_pwm.stop()
        GPIO.cleanup()


def main():
    # Define GPIO pins connected to MOSFETs
    # Note: Use the appropriate pin numbers for your connections
    MOTOR1_PIN = 32  # Example pin, change to your actual pin
    MOTOR2_PIN = 33  # Example pin, change to your actual pin
    
    try:
        # Initialize motor controller
        motors = MotorController(MOTOR1_PIN, MOTOR2_PIN)
        
        print("Motor control demo starting...")
        
        # Move forward for 2 seconds
        print("Moving forward...")
        motors.move_forward(70)
        time.sleep(2)
        
        # Stop for 1 second
        print("Stopping...")
        motors.stop()
        time.sleep(1)
        
        # Turn left for 1 second
        print("Turning left...")
        motors.turn_left(70)
        time.sleep(1)
        
        # Turn right for 1 second
        print("Turning right...")
        motors.turn_right(70)
        time.sleep(1)
        
        # Move forward at different speeds
        print("Accelerating...")
        for speed in range(20, 101, 20):
            print(f"Speed: {speed}%")
            motors.move_forward(speed)
            time.sleep(1)
        
        # Stop motors
        print("Stopping...")
        motors.stop()
        
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        # Clean up GPIO resources
        if 'motors' in locals():
            motors.cleanup()
        print("GPIO cleaned up")


if __name__ == "__main__":
    main() 