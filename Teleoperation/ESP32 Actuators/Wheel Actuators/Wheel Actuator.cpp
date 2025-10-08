#include <Arduino.h>
#include "driver/twai.h" // The native ESP-IDF TWAI/CAN driver
#include <SPI.h>

// --- Pin Definitions ---
const int CAN_STBY_PIN = 47;
const gpio_num_t CAN_TX_PIN = GPIO_NUM_48;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_34;

const int M1_FORWARD_PIN = 40; 
const int M1_REVERSE_PIN = 39;
const int M2_FORWARD_PIN = 37;
const int M2_REVERSE_PIN = 38; 

const int SPI_CS_PIN = 4;
const int SPI_SCK_PIN = 12;
const int SPI_MISO_PIN = 13;
const int SPI_MOSI_PIN = 14;

// --- PWM Configuration ---
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;
const int M1_FORWARD_CHANNEL = 0;
const int M1_REVERSE_CHANNEL = 1;
const int M2_FORWARD_CHANNEL = 2;
const int M2_REVERSE_CHANNEL = 3;

// --- CAN Configuration ---
const int DRIVE_SYSTEM_CAN_ID = 0x200;

// Drive command byte positions from Pi: ['f', 'b', 'l', 'r', 'u', 'd', 'l2', 'r2']
const int POS_F = 0;
const int POS_B = 1;
const int POS_L = 2;
const int POS_R = 3;

// --- Global Motor Speed Variables ---
float g_motor1_speed = 0.0;
float g_motor2_speed = 0.0;

// --- Timing for Periodic Tasks ---
const unsigned long PERIODIC_TASK_INTERVAL = 50; // ms
unsigned long g_previous_task_millis = 0;

// --- AS5047P Sensor Configuration ---
const uint16_t REG_ANGLEUNC = 0x3FFE;
const uint16_t REG_NOP = 0x0000;
const uint16_t CMD_READ_FLAG = 0x4000;
SPISettings settings(1000000, MSBFIRST, SPI_MODE1);

// --- Function Prototypes ---
void setupCAN();
void receiveCAN();
void processDriveCommand(const twai_message_t &message);
void setMotorSpeed(int forward_channel, int reverse_channel, float speed);
uint16_t readAngle();


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("--- CAN Bus Actuator Controller (Listening for ID: 0x%03X) ---\n", DRIVE_SYSTEM_CAN_ID);

  setupCAN();

  ledcSetup(M1_FORWARD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M1_REVERSE_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2_FORWARD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2_REVERSE_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  
  ledcAttachPin(M1_FORWARD_PIN, M1_FORWARD_CHANNEL);
  ledcAttachPin(M1_REVERSE_PIN, M1_REVERSE_CHANNEL);
  ledcAttachPin(M2_FORWARD_PIN, M2_FORWARD_CHANNEL);
  ledcAttachPin(M2_REVERSE_PIN, M2_REVERSE_CHANNEL);
  Serial.println("Motor PWM outputs configured.");

  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Serial.println("AS5047P Sensor initialized.");
}

void loop() {
  receiveCAN();
  setMotorSpeed(M1_FORWARD_CHANNEL, M1_REVERSE_CHANNEL, g_motor1_speed);
  setMotorSpeed(M2_FORWARD_CHANNEL, M2_REVERSE_CHANNEL, g_motor2_speed);

  unsigned long current_millis = millis();
  if (current_millis - g_previous_task_millis >= PERIODIC_TASK_INTERVAL) {
    g_previous_task_millis = current_millis;
    uint16_t rawAngle = readAngle();
    float degrees = rawAngle * (360.0 / 16384.0);
    Serial.printf("Sensor Angle: %.2f | M1 Speed: %.2f | M2 Speed: %.2f\n", degrees, g_motor1_speed, g_motor2_speed);
  }
}

void processDriveCommand(const twai_message_t &message) {
  if (message.data_length_code < 8) {
    Serial.printf("ERROR: Expected 8-byte payload, got %d bytes\n", message.data_length_code);
    return;
  }
  
  uint8_t f_value = message.data[POS_F];
  uint8_t b_value = message.data[POS_B];
  uint8_t l_value = message.data[POS_L];
  uint8_t r_value = message.data[POS_R];
  
  // --- *** CORRECTED SCALING LOGIC *** ---
  // The incoming values are 0-100, so we divide by 100.0 to get a 0.0-1.0 scale.

  // Motor 1 is controlled by F (forward) and B (backward)
  float motor1_forward_scaled = (float)f_value / 100.0;
  float motor1_backward_scaled = (float)b_value / 100.0;
  g_motor1_speed = motor1_forward_scaled - motor1_backward_scaled;

  // Motor 2 is controlled by R (forward) and L (backward)
  float motor2_forward_scaled = (float)r_value / 100.0;
  float motor2_backward_scaled = (float)l_value / 100.0;
  g_motor2_speed = motor2_forward_scaled - motor2_backward_scaled;

  // Debugging print to confirm logic
  Serial.printf("-> Drive Cmd Parsed: M1_Speed=%.2f (f-b), M2_Speed=%.2f (r-l)\n", g_motor1_speed, g_motor2_speed);
}

void receiveCAN() {
  twai_message_t message;
  if (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier == DRIVE_SYSTEM_CAN_ID) {
      processDriveCommand(message);
    }
  }
}

// ---UNCHANGED FUNCTIONS BELOW---

void setupCAN() {
  pinMode(CAN_STBY_PIN, OUTPUT);
  digitalWrite(CAN_STBY_PIN, LOW);
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver");
    return;
  }
  Serial.println("CAN Driver started successfully!");
}

void setMotorSpeed(int forward_channel, int reverse_channel, float speed) {
  speed = constrain(speed, -1.0, 1.0);
  if (speed >= 0.0) {
    ledcWrite(forward_channel, (uint32_t)(speed * 255.0));
    ledcWrite(reverse_channel, 0);
  } else {
    ledcWrite(forward_channel, 0);
    ledcWrite(reverse_channel, (uint32_t)(-speed * 255.0));
  }
}

uint16_t readAngle() {
  uint16_t command = CMD_READ_FLAG | REG_ANGLEUNC;
  
  SPI.beginTransaction(settings);
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer16(command);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();

  delayMicroseconds(1);

  command = CMD_READ_FLAG | REG_NOP;
  
  SPI.beginTransaction(settings);
  digitalWrite(SPI_CS_PIN, LOW);
  uint16_t receivedData = SPI.transfer16(command);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();

  return receivedData & 0x3FFF;
}