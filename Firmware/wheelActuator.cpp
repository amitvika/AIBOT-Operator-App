#include <Arduino.h>
#include "driver/twai.h" // The native ESP-IDF TWAI/CAN driver
#include <SPI.h>
#include <Preferences.h>

// --- Pin Definitions ---
const int CAN_STBY_PIN = 47;
const gpio_num_t CAN_TX_PIN = GPIO_NUM_48;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_34;

const int M1_FORWARD_PIN = 37; 
const int M1_REVERSE_PIN = 38;
const int M2_FORWARD_PIN = 40;
const int M2_REVERSE_PIN = 39; 

const int SPI_CS_PIN = 4;
const int SPI_SCK_PIN = 12;
const int SPI_MISO_PIN = 13;
const int SPI_MOSI_PIN = 14;

// --- PWM Configuration ---
const int PWM_FREQUENCY = 20000; 
const int PWM_RESOLUTION = 8;
const int M1_FORWARD_CHANNEL = 0;
const int M1_REVERSE_CHANNEL = 1;
const int M2_FORWARD_CHANNEL = 2;
const int M2_REVERSE_CHANNEL = 3;

// --- CAN Configuration ---
const int DRIVE_SYSTEM_CAN_ID = 0x200;

// --- Tuning Command CAN IDs ---
const int TUNING_BASE_ID = 0x300;
const int CMD_SET_ADDRESS = TUNING_BASE_ID + 0;      // 0x300
const int CMD_SET_SETPOINT = TUNING_BASE_ID + 1;     // 0x301
const int CMD_MOTOR_DIR = TUNING_BASE_ID + 2;        // 0x302
const int CMD_SENSOR_DIR = TUNING_BASE_ID + 3;       // 0x303
const int CMD_UPDATE_P = TUNING_BASE_ID + 4;         // 0x304
const int CMD_MAX_PWM = TUNING_BASE_ID + 5;          // 0x305
const int CMD_MIN_PWM = TUNING_BASE_ID + 6;          // 0x306
const int CMD_SAVE_PARAMS = TUNING_BASE_ID + 7;      // 0x307
const int CMD_REQUEST_TELEMETRY = TUNING_BASE_ID + 8; // 0x308
const int CMD_TELEMETRY_RESPONSE_1 = TUNING_BASE_ID + 9; // 0x309
const int CMD_TELEMETRY_RESPONSE_2 = TUNING_BASE_ID + 10; // 0x30A

// Drive command byte positions from Pi: ['f', 'b', 'l', 'r', 'u', 'd', 'l2', 'r2']
const int POS_F = 0;
const int POS_B = 1;
const int POS_L = 2;
const int POS_R = 3;

// --- Global Motor Speed Variables ---
float g_motor1_speed = 0.0;
float g_motor2_speed = 0.0;

// --- Position Control Variables for Motor2 (Actuator) ---
float g_target_position_degrees = 0.0;  // Target position in degrees
float g_current_position_degrees = 0.0; // Current position in degrees
float g_position_error = 0.0;           // Position error in degrees
float g_motor2_output = 0.0;            // PID output for Motor2

// PID Controller Parameters
float KP = 5.0;  // Proportional gain (now tunable)
const float KI = 0;  // Integral gain  
const float KD = 0;  // Derivative gain
float g_integral_error = 0.0;    // Integral of error
float g_previous_error = 0.0;    // Previous error for derivative
unsigned long g_last_pid_time = 0; // Last PID calculation time

// --- Tunable Parameters ---
int g_can_address = DRIVE_SYSTEM_CAN_ID;  // Device CAN address (tunable)
float g_actuator_setpoint = 0.0;           // Actuator zero position offset (degrees)
bool g_motor_direction_inverted = false;   // Motor direction flag
bool g_sensor_direction_inverted = false;  // Sensor direction flag
int g_max_pwm = 255;                       // Maximum PWM value (0-255)
int g_min_pwm = 30;                        // Minimum PWM value (deadzone)

// --- EEPROM/Preferences Storage ---
Preferences preferences;

// --- Timing for Periodic Tasks ---
const unsigned long PERIODIC_TASK_INTERVAL = 50; // ms
unsigned long g_previous_task_millis = 0;

// --- CAN Bus Health Monitoring ---
const unsigned long CAN_HEALTH_CHECK_INTERVAL = 1000; // Check every 1 second
unsigned long g_last_can_health_check = 0;

// --- AS5047P Sensor Configuration ---
const uint16_t REG_ANGLEUNC = 0x3FFE;
const uint16_t REG_NOP = 0x0000;
const uint16_t CMD_READ_FLAG = 0x4000;
SPISettings settings(1000000, MSBFIRST, SPI_MODE1);

// --- Function Prototypes ---
void setupCAN();
void receiveCAN();
void checkCANHealth();
void processDriveCommand(const twai_message_t &message);
void processTuningCommand(const twai_message_t &message);
void sendTelemetry();
void loadParametersFromEEPROM();
void saveParametersToEEPROM();
void setMotorSpeed(int forward_channel, int reverse_channel, float speed);
uint16_t readAngle();
void updatePositionControl();
float calculatePID(float error);
void setTargetPosition(uint8_t l_value, uint8_t r_value);


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("--- CAN Bus Actuator Controller (Listening for ID: 0x%03X) ---\n", DRIVE_SYSTEM_CAN_ID);

  // Load saved parameters from EEPROM
  loadParametersFromEEPROM();

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
  // Check CAN bus health periodically
  checkCANHealth();
  
  receiveCAN();
  
  // Update position control for Motor2 (actuator)
  updatePositionControl();
  
  // Apply motor speeds
  setMotorSpeed(M1_FORWARD_CHANNEL, M1_REVERSE_CHANNEL, g_motor1_speed);
  setMotorSpeed(M2_FORWARD_CHANNEL, M2_REVERSE_CHANNEL, g_motor2_output); // Use PID output for Motor2

  unsigned long current_millis = millis();
  if (current_millis - g_previous_task_millis >= PERIODIC_TASK_INTERVAL) {
    g_previous_task_millis = current_millis;
    uint16_t rawAngle = readAngle();
    float degrees = rawAngle * (360.0 / 16384.0);
    Serial.printf("Sensor: %.2f° | Target: %.2f° | Error: %.2f° | M1: %.2f | M2: %.2f\n", 
                  degrees, g_target_position_degrees, g_position_error, g_motor1_speed, g_motor2_output);
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

  // Motor 2 is now position-controlled using L and R values
  setTargetPosition(l_value, r_value);

  // Debugging print to confirm logic
  Serial.printf("-> Drive Cmd Parsed: M1_Speed=%.2f (f-b), M2_Target=%.2f° (L:%d R:%d)\n", 
                g_motor1_speed, g_target_position_degrees, l_value, r_value);
}

void receiveCAN() {
  twai_message_t message;
  if (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier == g_can_address) {
      processDriveCommand(message);
    }
    else if (message.identifier >= TUNING_BASE_ID && message.identifier < TUNING_BASE_ID + 10) {
      processTuningCommand(message);
    }
  }
}

/**
 * @brief Monitors CAN bus health and recovers from error states
 * This prevents the need to manually reset when the transmitter starts late
 */
void checkCANHealth() {
  unsigned long current_millis = millis();
  
  // Only check periodically to avoid excessive overhead
  if (current_millis - g_last_can_health_check < CAN_HEALTH_CHECK_INTERVAL) {
    return;
  }
  g_last_can_health_check = current_millis;
  
  // Get CAN bus status
  twai_status_info_t status_info;
  if (twai_get_status_info(&status_info) == ESP_OK) {
    // Check if CAN controller is in error states
    if (status_info.state == TWAI_STATE_BUS_OFF) {
      Serial.println("CAN: Bus-off detected! Initiating recovery...");
      twai_initiate_recovery();
      delay(100);
      Serial.println("CAN: Recovery initiated");
    } 
    else if (status_info.state == TWAI_STATE_RECOVERING) {
      Serial.println("CAN: Bus recovering...");
    }
    else if (status_info.state == TWAI_STATE_STOPPED) {
      Serial.println("CAN: Bus stopped! Restarting...");
      twai_start();
    }
    
    // Optional: Log warnings if TX/RX error counters are high
    if (status_info.tx_error_counter > 96 || status_info.rx_error_counter > 96) {
      Serial.printf("CAN Warning: TX errors=%d, RX errors=%d\n", 
                    status_info.tx_error_counter, status_info.rx_error_counter);
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
  
  // Apply motor direction inversion if enabled
  if (g_motor_direction_inverted) {
    speed = -speed;
  }
  
  // Calculate PWM value with min/max limits
  int pwm_value;
  if (speed >= 0.0) {
    pwm_value = (int)(speed * g_max_pwm);
    if (pwm_value > 0 && pwm_value < g_min_pwm) {
      pwm_value = g_min_pwm; // Apply deadzone
    }
    ledcWrite(forward_channel, pwm_value);
    ledcWrite(reverse_channel, 0);
  } else {
    pwm_value = (int)(-speed * g_max_pwm);
    if (pwm_value > 0 && pwm_value < g_min_pwm) {
      pwm_value = g_min_pwm; // Apply deadzone
    }
    ledcWrite(forward_channel, 0);
    ledcWrite(reverse_channel, pwm_value);
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

  uint16_t angle = receivedData & 0x3FFF;
  
  // Apply sensor direction inversion if enabled
  if (g_sensor_direction_inverted) {
    angle = 16384 - angle; // Invert the angle reading
  }
  
  return angle;
}

/**
 * @brief Sets the target position based on L and R values
 * L value (0-100) maps to 0° to -90° (left rotation)
 * R value (0-100) maps to 0° to +90° (right rotation)
 */
void setTargetPosition(uint8_t l_value, uint8_t r_value) {
  // Calculate target position: L moves left (negative), R moves right (positive)
  float left_degrees = -(float)l_value * 0.9;  // L=100 -> -90°
  float right_degrees = (float)r_value * 0.9;  // R=100 -> +90°
  
  // Combine L and R values (they should be mutually exclusive in practice)
  g_target_position_degrees = right_degrees + left_degrees;
  
  // Clamp to reasonable range
  g_target_position_degrees = constrain(g_target_position_degrees, -90.0, 90.0);
}

/**
 * @brief Updates the position control system
 * Reads current position, calculates error, runs PID, and updates motor output
 */
void updatePositionControl() {
  // Read current position from sensor
  uint16_t raw_angle = readAngle();
  g_current_position_degrees = raw_angle * (360.0 / 16384.0);
  
  // Apply actuator setpoint offset
  g_current_position_degrees += g_actuator_setpoint;
  
  // Calculate position error
  g_position_error = g_target_position_degrees - g_current_position_degrees;
  
  // Handle angle wrapping (shortest path)
  if (g_position_error > 180.0) {
    g_position_error -= 360.0;
  } else if (g_position_error < -180.0) {
    g_position_error += 360.0;
  }
  
  // Calculate PID output
  g_motor2_output = calculatePID(g_position_error);
}

/**
 * @brief PID controller for position control
 * @param error Position error in degrees
 * @return Motor output (-1.0 to 1.0)
 */
float calculatePID(float error) {
  unsigned long current_time = millis();
  float dt = (current_time - g_last_pid_time) / 1000.0; // Convert to seconds
  
  if (dt <= 0) dt = 0.01; // Prevent division by zero
  g_last_pid_time = current_time;
  
  // Proportional term
  float p_term = KP * error;
  
  // Integral term
  g_integral_error += error * dt;
  float i_term = KI * g_integral_error;
  
  // Derivative term
  float d_term = KD * (error - g_previous_error) / dt;
  g_previous_error = error;
  
  // Calculate PID output
  float output = p_term + i_term + d_term;
  
  // Clamp output to motor range
  output = constrain(output, -1.0, 1.0);
  
  // Deadband to prevent jitter when close to target
  if (abs(error) < 1.0) {
    output *= 0.3; // Reduce output when close to target
  }
  
  return output;
}

/**
 * @brief Processes tuning commands received via CAN
 * @param message The CAN message containing the tuning command
 */
void processTuningCommand(const twai_message_t &message) {
  uint32_t cmd_id = message.identifier;
  
  Serial.printf("Tuning command received: 0x%03X\n", cmd_id);
  
  switch (cmd_id) {
    case CMD_SET_ADDRESS:
      if (message.data_length_code >= 2) {
        uint16_t new_address = (message.data[0] << 8) | message.data[1];
        g_can_address = new_address;
        Serial.printf("-> CAN Address set to: 0x%03X\n", g_can_address);
      }
      break;
      
    case CMD_SET_SETPOINT:
      if (message.data_length_code >= 4) {
        float setpoint;
        memcpy(&setpoint, message.data, sizeof(float));
        g_actuator_setpoint = setpoint;
        Serial.printf("-> Actuator setpoint set to: %.2f degrees\n", g_actuator_setpoint);
      }
      break;
      
    case CMD_MOTOR_DIR:
      if (message.data_length_code >= 1) {
        g_motor_direction_inverted = (message.data[0] != 0);
        Serial.printf("-> Motor direction inverted: %s\n", g_motor_direction_inverted ? "YES" : "NO");
      }
      break;
      
    case CMD_SENSOR_DIR:
      if (message.data_length_code >= 1) {
        g_sensor_direction_inverted = (message.data[0] != 0);
        Serial.printf("-> Sensor direction inverted: %s\n", g_sensor_direction_inverted ? "YES" : "NO");
      }
      break;
      
    case CMD_UPDATE_P:
      if (message.data_length_code >= 4) {
        float new_kp;
        memcpy(&new_kp, message.data, sizeof(float));
        KP = new_kp;
        Serial.printf("-> P value updated to: %.3f\n", KP);
      }
      break;
      
    case CMD_MAX_PWM:
      if (message.data_length_code >= 1) {
        g_max_pwm = message.data[0];
        g_max_pwm = constrain(g_max_pwm, 0, 255);
        Serial.printf("-> Max PWM set to: %d\n", g_max_pwm);
      }
      break;
      
    case CMD_MIN_PWM:
      if (message.data_length_code >= 1) {
        g_min_pwm = message.data[0];
        g_min_pwm = constrain(g_min_pwm, 0, 255);
        Serial.printf("-> Min PWM (deadzone) set to: %d\n", g_min_pwm);
      }
      break;
      
    case CMD_SAVE_PARAMS:
      Serial.println("-> Save parameters command received");
      saveParametersToEEPROM();
      Serial.println("-> Parameters saved to EEPROM");
      break;
      
    case CMD_REQUEST_TELEMETRY:
      if (message.data_length_code >= 2) {
        // Check if the request is for this device
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          Serial.printf("-> Telemetry requested for address 0x%03X\n", g_can_address);
          sendTelemetry();
        }
      }
      break;
      
    default:
      Serial.printf("-> Unknown tuning command: 0x%03X\n", cmd_id);
      break;
  }
}

/**
 * @brief Sends current parameter values via CAN telemetry messages
 */
void sendTelemetry() {
  twai_message_t telemetry_msg;
  telemetry_msg.extd = 0;
  telemetry_msg.rtr = 0;
  telemetry_msg.ss = 0;
  telemetry_msg.self = 0;
  telemetry_msg.dlc_non_comp = 0;
  
  // Telemetry Response 1: address(2), motor_dir(1), sensor_dir(1), max_pwm(1), min_pwm(1)
  telemetry_msg.identifier = CMD_TELEMETRY_RESPONSE_1;
  telemetry_msg.data_length_code = 6;
  
  // Pack address (big-endian uint16)
  telemetry_msg.data[0] = (g_can_address >> 8) & 0xFF;
  telemetry_msg.data[1] = g_can_address & 0xFF;
  
  // Pack boolean flags
  telemetry_msg.data[2] = g_motor_direction_inverted ? 1 : 0;
  telemetry_msg.data[3] = g_sensor_direction_inverted ? 1 : 0;
  
  // Pack PWM values
  telemetry_msg.data[4] = g_max_pwm;
  telemetry_msg.data[5] = g_min_pwm;
  
  // Send first telemetry message
  if (twai_transmit(&telemetry_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("-> Telemetry part 1 sent");
  } else {
    Serial.println("-> Failed to send telemetry part 1");
  }
  
  delay(10); // Small delay between messages
  
  // Telemetry Response 2: setpoint(4), p_gain(4)
  telemetry_msg.identifier = CMD_TELEMETRY_RESPONSE_2;
  telemetry_msg.data_length_code = 8;
  
  // Pack setpoint (little-endian float)
  memcpy(telemetry_msg.data, &g_actuator_setpoint, sizeof(float));
  
  // Pack P gain (little-endian float)
  memcpy(telemetry_msg.data + 4, &KP, sizeof(float));
  
  // Send second telemetry message
  if (twai_transmit(&telemetry_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("-> Telemetry part 2 sent");
  } else {
    Serial.println("-> Failed to send telemetry part 2");
  }
  
  Serial.printf("-> Telemetry: Addr=0x%03X, MotorDir=%d, SensorDir=%d, MaxPWM=%d, MinPWM=%d, Setpoint=%.2f, P=%.3f\n",
                g_can_address, g_motor_direction_inverted, g_sensor_direction_inverted, 
                g_max_pwm, g_min_pwm, g_actuator_setpoint, KP);
}

/**
 * @brief Loads tuning parameters from EEPROM/Preferences
 */
void loadParametersFromEEPROM() {
  preferences.begin("wheel_actuator", false); // false = read-write mode
  
  // Check if parameters have been saved before
  bool initialized = preferences.getBool("initialized", false);
  
  if (initialized) {
    // Load saved parameters
    g_can_address = preferences.getInt("can_addr", DRIVE_SYSTEM_CAN_ID);
    g_actuator_setpoint = preferences.getFloat("setpoint", 0.0);
    g_motor_direction_inverted = preferences.getBool("motor_inv", false);
    g_sensor_direction_inverted = preferences.getBool("sensor_inv", false);
    KP = preferences.getFloat("kp", 5.0);
    g_max_pwm = preferences.getInt("max_pwm", 255);
    g_min_pwm = preferences.getInt("min_pwm", 30);
    
    Serial.println("=== Loaded Parameters from EEPROM ===");
    Serial.printf("  CAN Address: 0x%03X (%d)\n", g_can_address, g_can_address);
    Serial.printf("  Setpoint: %.2f degrees\n", g_actuator_setpoint);
    Serial.printf("  Motor Direction: %s\n", g_motor_direction_inverted ? "INVERTED" : "NORMAL");
    Serial.printf("  Sensor Direction: %s\n", g_sensor_direction_inverted ? "INVERTED" : "NORMAL");
    Serial.printf("  P Gain: %.3f\n", KP);
    Serial.printf("  Max PWM: %d\n", g_max_pwm);
    Serial.printf("  Min PWM: %d\n", g_min_pwm);
    Serial.println("=====================================");
  } else {
    Serial.println("=== No Saved Parameters Found ===");
    Serial.println("Using default values. Use 'Save Parameters' to persist settings.");
    Serial.println("=================================");
  }
  
  preferences.end();
}

/**
 * @brief Saves current tuning parameters to EEPROM/Preferences
 */
void saveParametersToEEPROM() {
  preferences.begin("wheel_actuator", false); // false = read-write mode
  
  // Save all tunable parameters
  preferences.putInt("can_addr", g_can_address);
  preferences.putFloat("setpoint", g_actuator_setpoint);
  preferences.putBool("motor_inv", g_motor_direction_inverted);
  preferences.putBool("sensor_inv", g_sensor_direction_inverted);
  preferences.putFloat("kp", KP);
  preferences.putInt("max_pwm", g_max_pwm);
  preferences.putInt("min_pwm", g_min_pwm);
  preferences.putBool("initialized", true); // Mark as initialized
  
  preferences.end();
  
  Serial.println("=== Saved Parameters to EEPROM ===");
  Serial.printf("  CAN Address: 0x%03X (%d)\n", g_can_address, g_can_address);
  Serial.printf("  Setpoint: %.2f degrees\n", g_actuator_setpoint);
  Serial.printf("  Motor Direction: %s\n", g_motor_direction_inverted ? "INVERTED" : "NORMAL");
  Serial.printf("  Sensor Direction: %s\n", g_sensor_direction_inverted ? "INVERTED" : "NORMAL");
  Serial.printf("  P Gain: %.3f\n", KP);
  Serial.printf("  Max PWM: %d\n", g_max_pwm);
  Serial.printf("  Min PWM: %d\n", g_min_pwm);
  Serial.println("===================================");
}