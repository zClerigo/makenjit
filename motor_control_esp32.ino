/*
 * ESP32 Motor Control
 * 
 * This program receives commands from Jetson Orin via UART
 * and controls motors accordingly.
 * 
 * UART Connections:
 * - ESP32 Pin 16 (RX) connects to Jetson Orin Pin 8 (TX)
 * - ESP32 Pin 17 (TX) connects to Jetson Orin Pin 10 (RX)
 * 
 * Motor Connections:
 * - Left Motor: ESP32 Pins 18 and 27
 */

// Define motor control pins
#define LEFT_MOTOR_PIN1 18
#define LEFT_MOTOR_PIN2 27

// PWM properties
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
#define PWM_MAX 255

// PWM channels
#define PWM_CHANNEL_LEFT_1 0
#define PWM_CHANNEL_LEFT_2 1

// Buffer for incoming data
#define MAX_BUFFER_SIZE 64
char buffer[MAX_BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  // Initialize UART communication
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  
  // Configure motor control pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  
  // Configure PWM for motor control
  ledcSetup(PWM_CHANNEL_LEFT_1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT_2, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Attach PWM channels to GPIO pins
  ledcAttachPin(LEFT_MOTOR_PIN1, PWM_CHANNEL_LEFT_1);
  ledcAttachPin(LEFT_MOTOR_PIN2, PWM_CHANNEL_LEFT_2);
  
  // Stop motors initially
  stopLeftMotor();
  
  Serial.println("ESP32 Motor Controller initialized");
  Serial2.println("ESP32 Motor Controller initialized");
}

void loop() {
  // Check for commands from Jetson Orin
  if (Serial2.available()) {
    char c = Serial2.read();
    
    // Process complete commands (terminated by newline)
    if (c == '\n') {
      buffer[bufferIndex] = '\0';  // Null-terminate the string
      processCommand(buffer);
      bufferIndex = 0;  // Reset buffer for next command
    } else if (bufferIndex < MAX_BUFFER_SIZE - 1) {
      buffer[bufferIndex++] = c;  // Add character to buffer
    }
  }
}

void processCommand(const char* cmd) {
  Serial.print("Received command: ");
  Serial.println(cmd);
  
  // Check for ping command
  if (strcmp(cmd, "PING") == 0) {
    Serial2.println("PONG");
    return;
  }
  
  // Process left motor command (format: "L:speed")
  if (cmd[0] == 'L' && cmd[1] == ':') {
    int speed = atoi(&cmd[2]);
    setLeftMotorSpeed(speed);
    Serial2.print("Left motor set to speed: ");
    Serial2.println(speed);
    return;
  }
  
  // Unknown command
  Serial2.println("Unknown command");
}

void setLeftMotorSpeed(int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  
  if (speed > 0) {
    // Forward
    ledcWrite(PWM_CHANNEL_LEFT_1, speed);
    ledcWrite(PWM_CHANNEL_LEFT_2, 0);
  } else if (speed < 0) {
    // Backward
    ledcWrite(PWM_CHANNEL_LEFT_1, 0);
    ledcWrite(PWM_CHANNEL_LEFT_2, -speed);
  } else {
    // Stop
    stopLeftMotor();
  }
}

void stopLeftMotor() {
  ledcWrite(PWM_CHANNEL_LEFT_1, 0);
  ledcWrite(PWM_CHANNEL_LEFT_2, 0);
} 