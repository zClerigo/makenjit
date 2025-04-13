#!/usr/bin/env python3
import serial
import time
import sys

class MotorController:
    def __init__(self, port='/dev/ttyTHS0', baudrate=115200):
        """
        Initialize the motor controller using UART communication with ESP32.
        Jetson Orin uses pins 8 (TX) and 10 (RX) for UART.
        """
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"Connected to ESP32 via {port}")
            time.sleep(2)  # Allow time for ESP32 to reset
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            sys.exit(1)

    def send_command(self, command):
        """Send a command to the ESP32."""
        try:
            self.serial.write(f"{command}\n".encode())
            time.sleep(0.1)  # Small delay to ensure command is processed
            response = self.serial.readline().decode().strip()
            print(f"ESP32 response: {response}")
            return response
        except Exception as e:
            print(f"Error sending command: {e}")
            return None

    def test_left_motor(self, speed=100, duration=2):
        """Test the left motor at specified speed for a duration."""
        print(f"Testing left motor at speed {speed} for {duration} seconds")
        self.send_command(f"L:{speed}")
        time.sleep(duration)
        self.send_command("L:0")  # Stop the motor
        
    def test_motor_sequence(self):
        """Run a sequence of motor tests."""
        print("Starting motor test sequence...")
        
        # Forward at different speeds
        for speed in [50, 100, 150, 200, 255]:
            self.test_left_motor(speed, 1)
            time.sleep(0.5)
        
        # Reverse at different speeds
        for speed in [-50, -100, -150, -200, -255]:
            self.test_left_motor(speed, 1)
            time.sleep(0.5)
            
        print("Motor test sequence completed")

    def close(self):
        """Close the serial connection."""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            print("Serial connection closed")

if __name__ == "__main__":
    controller = MotorController()
    
    try:
        # Simple test
        controller.send_command("PING")
        
        # Run motor tests
        controller.test_motor_sequence()
        
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        controller.close() 