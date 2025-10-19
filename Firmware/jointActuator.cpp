#include <Arduino.h>
#include "driver/twai.h" // The native ESP-IDF TWAI/CAN driver
#include <SPI.h>
#include <Preferences.h>

// --- Pin Definitions ---
const int CAN_STBY_PIN = 47;
const gpio_num_t CAN_TX_PIN = GPIO_NUM_48;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_34;

const int MOTOR_FORWARD_PIN = 37; 
const int MOTOR_REVERSE_PIN = 38;

const int SPI_CS_PIN = 4;
const int SPI_SCK_PIN = 12;
const int SPI_MISO_PIN = 13;
const int SPI_MOSI_PIN = 14;

// --- PWM Configuration ---
const int PWM_FREQUENCY = 20000; 
const int PWM_RESOLUTION = 8;
const int MOTOR_FORWARD_CHANNEL = 0;
const int MOTOR_REVERSE_CHANNEL = 1;

// --- CAN Configuration ---
// Joint actuators use CAN IDs 0x600-0x60B (12 actuators)
// Default to first joint actuator - MUST BE CONFIGURED for each actuator
const int JOINT_ACTUATOR_CAN_ID = 0x600;

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

// --- Global Motor Variables ---
float g_motor_output = 0.0;  // PID output for motor

// --- Position Control Variables ---
float g_target_position_degrees = 0.0;  // Target position in degrees
float g_current_position_degrees = 0.0; // Current position in degrees
float g_position_error = 0.0;           // Position error in degrees

// PID Controller Parameters
float KP = 5.0;   // Proportional gain (tunable)
const float KI = 0.0;   // Integral gain  
const float KD = 0.0;   // Derivative gain
float g_integral_error = 0.0;    // Integral of error
float g_previous_error = 0.0;    // Previous error for derivative
unsigned long g_last_pid_time = 0; // Last PID calculation time

// --- Tunable Parameters ---
int g_can_address = JOINT_ACTUATOR_CAN_ID;  // Device CAN address (tunable)
float g_actuator_setpoint = 0.0;            // Actuator zero position offset (degrees)
bool g_motor_direction_inverted = false;    // Motor direction flag
bool g_sensor_direction_inverted = false;   // Sensor direction flag
int g_max_pwm = 255;                        // Maximum PWM value (0-255)
int g_min_pwm = 30;                         // Minimum PWM value (deadzone)

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
void processPositionCommand(const twai_message_t &message);
void processTuningCommand(const twai_message_t &message);
void sendTelemetry();
void loadParametersFromEEPROM();
void saveParametersToEEPROM();
void setMotorSpeed(float speed);
uint16_t readAngle();
void updatePositionControl();
float calculatePID(float error);


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("--- Joint Actuator Controller (CAN ID: 0x%03X) ---\n", JOINT_ACTUATOR_CAN_ID);

  // Load saved parameters from EEPROM
  loadParametersFromEEPROM();

  setupCAN();

  // Setup PWM channels
  ledcSetup(MOTOR_FORWARD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR_REVERSE_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  
  ledcAttachPin(MOTOR_FORWARD_PIN, MOTOR_FORWARD_CHANNEL);
  ledcAttachPin(MOTOR_REVERSE_PIN, MOTOR_REVERSE_CHANNEL);
  Serial.println("Motor PWM outputs configured.");

  // Initialize SPI for AS5047P sensor
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Serial.println("AS5047P Position Sensor initialized.");
}

void loop() {
  // Check CAN bus health periodically
  checkCANHealth();
  
  // Receive and process CAN messages
  receiveCAN();
  
  // Update position control
  updatePositionControl();
  
  // Apply motor output
  setMotorSpeed(g_motor_output);

  // Periodic status reporting
  unsigned long current_millis = millis();
  if (current_millis - g_previous_task_millis >= PERIODIC_TASK_INTERVAL) {
    g_previous_task_millis = current_millis;
    uint16_t rawAngle = readAngle();
    float degrees = rawAngle * (360.0 / 16384.0);
    Serial.printf("Pos: %.2f° | Target: %.2f° | Error: %.2f° | Output: %.2f\n", 
                  degrees, g_target_position_degrees, g_position_error, g_motor_output);
  }
}

/**
 * @brief Process incoming position command
 * The position command is sent as a 4-byte float in the CAN payload
 */
void processPositionCommand(const twai_message_t &message) {
  if (message.data_length_code < 4) {
    Serial.printf("ERROR: Expected at least 4-byte payload, got %d bytes\n", message.data_length_code);
    return;
  }
  
  // Extract target position as float (little-endian)
  float target_position;
  memcpy(&target_position, message.data, sizeof(float));
  
  g_target_position_degrees = target_position;
  
  Serial.printf("-> Position Cmd: Target=%.2f°\n", g_target_position_degrees);
}

/**
 * @brief Receive and route CAN messages
 */
void receiveCAN() {
  twai_message_t message;
  if (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier == g_can_address) {
      processPositionCommand(message);
    }
    else if (message.identifier >= TUNING_BASE_ID && message.identifier <= TUNING_BASE_ID + 10) {
      processTuningCommand(message);
    }
  }
}

/**
 * @brief Monitors CAN bus health and recovers from error states
 */
void checkCANHealth() {
  unsigned long current_millis = millis();
  
  if (current_millis - g_last_can_health_check < CAN_HEALTH_CHECK_INTERVAL) {
    return;
  }
  g_last_can_health_check = current_millis;
  
  twai_status_info_t status_info;
  if (twai_get_status_info(&status_info) == ESP_OK) {
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
    
    if (status_info.tx_error_counter > 96 || status_info.rx_error_counter > 96) {
      Serial.printf("CAN Warning: TX errors=%d, RX errors=%d\n", 
                    status_info.tx_error_counter, status_info.rx_error_counter);
    }
  }
}

/**
 * @brief Initialize CAN bus
 */
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
  Serial.println("CAN Bus initialized successfully.");
}

/**
 * @brief Set motor speed with direction control and PWM limits
 * @param speed Motor speed from -1.0 (full reverse) to +1.0 (full forward)
 */
void setMotorSpeed(float speed) {
  // Apply motor direction inversion if configured
  if (g_motor_direction_inverted) {
    speed = -speed;
  }
  
  // Clamp speed to valid range
  if (speed > 1.0) speed = 1.0;
  if (speed < -1.0) speed = -1.0;
  
  int pwm_value = 0;
  
  if (speed > 0) {
    // Forward motion
    pwm_value = (int)(speed * (g_max_pwm - g_min_pwm) + g_min_pwm);
    if (pwm_value > g_max_pwm) pwm_value = g_max_pwm;
    
    ledcWrite(MOTOR_FORWARD_CHANNEL, pwm_value);
    ledcWrite(MOTOR_REVERSE_CHANNEL, 0);
  } 
  else if (speed < 0) {
    // Reverse motion
    pwm_value = (int)((-speed) * (g_max_pwm - g_min_pwm) + g_min_pwm);
    if (pwm_value > g_max_pwm) pwm_value = g_max_pwm;
    
    ledcWrite(MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(MOTOR_REVERSE_CHANNEL, pwm_value);
  } 
  else {
    // Stop
    ledcWrite(MOTOR_FORWARD_CHANNEL, 0);
    ledcWrite(MOTOR_REVERSE_CHANNEL, 0);
  }
}

/**
 * @brief Read angle from AS5047P sensor via SPI
 * @return Raw 14-bit angle value (0-16383)
 */
uint16_t readAngle() {
  SPI.beginTransaction(settings);
  digitalWrite(SPI_CS_PIN, LOW);
  delayMicroseconds(1);
  
  // Send read command for angle register
  uint16_t command = CMD_READ_FLAG | REG_ANGLEUNC;
  SPI.transfer16(command);
  
  digitalWrite(SPI_CS_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CS_PIN, LOW);
  delayMicroseconds(1);
  
  // Send NOP to read the result
  uint16_t response = SPI.transfer16(CMD_READ_FLAG | REG_NOP);
  
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  
  // Extract 14-bit angle value
  uint16_t angle = response & 0x3FFF;
  
  // Apply sensor direction inversion if configured
  if (g_sensor_direction_inverted) {
    angle = 16383 - angle;
  }
  
  return angle;
}

/**
 * @brief Update position control using PID controller
 */
void updatePositionControl() {
  // Read current position
  uint16_t rawAngle = readAngle();
  g_current_position_degrees = rawAngle * (360.0 / 16384.0);
  
  // Apply setpoint offset
  float adjusted_current = g_current_position_degrees - g_actuator_setpoint;
  
  // Calculate error
  g_position_error = g_target_position_degrees - adjusted_current;
  
  // Calculate PID output
  g_motor_output = calculatePID(g_position_error);
}

/**
 * @brief Calculate PID controller output
 * @param error Position error in degrees
 * @return Motor output (-1.0 to +1.0)
 */
float calculatePID(float error) {
  unsigned long current_time = millis();
  float dt = (current_time - g_last_pid_time) / 1000.0;  // Convert to seconds
  
  if (dt <= 0) dt = 0.001;  // Prevent division by zero
  
  // Proportional term
  float P = KP * error;
  
  // Integral term
  g_integral_error += error * dt;
  float I = KI * g_integral_error;
  
  // Derivative term
  float D = 0;
  if (dt > 0) {
    D = KD * (error - g_previous_error) / dt;
  }
  
  // Calculate output
  float output = P + I + D;
  
  // Clamp output
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;
  
  // Update for next iteration
  g_previous_error = error;
  g_last_pid_time = current_time;
  
  return output;
}

/**
 * @brief Process tuning commands received via CAN
 */
void processTuningCommand(const twai_message_t &message) {
  uint32_t cmd_id = message.identifier;
  
  Serial.printf("-> Tuning Cmd ID: 0x%03X, DLC: %d\n", cmd_id, message.data_length_code);
  
  switch (cmd_id) {
    case CMD_SET_ADDRESS:
      if (message.data_length_code >= 2) {
        uint16_t new_address = (message.data[0] << 8) | message.data[1];
        // Check if this command is for us (broadcast or our address)
        if (new_address == g_can_address || message.data[0] == 0xFF) {
          g_can_address = new_address;
          Serial.printf("-> CAN Address set to: 0x%03X\n", g_can_address);
        }
      }
      break;
      
    case CMD_SET_SETPOINT:
      if (message.data_length_code >= 6) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          float new_setpoint;
          memcpy(&new_setpoint, &message.data[2], sizeof(float));
          g_actuator_setpoint = new_setpoint;
          Serial.printf("-> Setpoint set to: %.2f degrees\n", g_actuator_setpoint);
        }
      }
      break;
      
    case CMD_MOTOR_DIR:
      if (message.data_length_code >= 3) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          g_motor_direction_inverted = (message.data[2] != 0);
          Serial.printf("-> Motor direction inverted: %s\n", g_motor_direction_inverted ? "YES" : "NO");
        }
      }
      break;
      
    case CMD_SENSOR_DIR:
      if (message.data_length_code >= 3) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          g_sensor_direction_inverted = (message.data[2] != 0);
          Serial.printf("-> Sensor direction inverted: %s\n", g_sensor_direction_inverted ? "YES" : "NO");
        }
      }
      break;
      
    case CMD_UPDATE_P:
      if (message.data_length_code >= 6) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          float new_p;
          memcpy(&new_p, &message.data[2], sizeof(float));
          KP = new_p;
          Serial.printf("-> P gain set to: %.3f\n", KP);
        }
      }
      break;
      
    case CMD_MAX_PWM:
      if (message.data_length_code >= 3) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          g_max_pwm = message.data[2];
          if (g_max_pwm > 255) g_max_pwm = 255;
          Serial.printf("-> Max PWM set to: %d\n", g_max_pwm);
        }
      }
      break;
      
    case CMD_MIN_PWM:
      if (message.data_length_code >= 3) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          g_min_pwm = message.data[2];
          if (g_min_pwm > 255) g_min_pwm = 255;
          Serial.printf("-> Min PWM set to: %d\n", g_min_pwm);
        }
      }
      break;
      
    case CMD_SAVE_PARAMS:
      if (message.data_length_code >= 2) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          Serial.println("-> Saving parameters to EEPROM...");
          saveParametersToEEPROM();
        }
      }
      break;
      
    case CMD_REQUEST_TELEMETRY:
      if (message.data_length_code >= 2) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          Serial.printf("-> Telemetry requested for address 0x%03X\n", g_can_address);
          sendTelemetry();
        }
      }
      break;
      
    default:
      Serial.printf("-> Unknown tuning command ID: 0x%03X\n", cmd_id);
      break;
  }
}

/**
 * @brief Send telemetry data via CAN (current parameter values)
 * Sends two CAN messages with complete actuator state
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
  telemetry_msg.data[0] = (g_can_address >> 8) & 0xFF;
  telemetry_msg.data[1] = g_can_address & 0xFF;
  telemetry_msg.data[2] = g_motor_direction_inverted ? 1 : 0;
  telemetry_msg.data[3] = g_sensor_direction_inverted ? 1 : 0;
  telemetry_msg.data[4] = g_max_pwm;
  telemetry_msg.data[5] = g_min_pwm;
  
  if (twai_transmit(&telemetry_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("-> Telemetry part 1 sent");
  } else {
    Serial.println("-> Failed to send telemetry part 1");
  }
  
  delay(10); // Small delay between messages
  
  // Telemetry Response 2: setpoint(4), p_gain(4)
  telemetry_msg.identifier = CMD_TELEMETRY_RESPONSE_2;
  telemetry_msg.data_length_code = 8;
  memcpy(telemetry_msg.data, &g_actuator_setpoint, sizeof(float));
  memcpy(telemetry_msg.data + 4, &KP, sizeof(float));
  
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
 * @brief Load saved parameters from EEPROM
 */
void loadParametersFromEEPROM() {
  preferences.begin("joint-actuator", false);
  
  g_can_address = preferences.getUInt("can_addr", JOINT_ACTUATOR_CAN_ID);
  g_actuator_setpoint = preferences.getFloat("setpoint", 0.0);
  g_motor_direction_inverted = preferences.getBool("motor_inv", false);
  g_sensor_direction_inverted = preferences.getBool("sensor_inv", false);
  KP = preferences.getFloat("kp", 5.0);
  g_max_pwm = preferences.getUInt("max_pwm", 255);
  g_min_pwm = preferences.getUInt("min_pwm", 30);
  
  preferences.end();
  
  Serial.println("=== Loaded Parameters from EEPROM ===");
  Serial.printf("  CAN Address: 0x%03X\n", g_can_address);
  Serial.printf("  Setpoint: %.2f degrees\n", g_actuator_setpoint);
  Serial.printf("  Motor Inverted: %s\n", g_motor_direction_inverted ? "YES" : "NO");
  Serial.printf("  Sensor Inverted: %s\n", g_sensor_direction_inverted ? "YES" : "NO");
  Serial.printf("  P Gain: %.3f\n", KP);
  Serial.printf("  Max PWM: %d\n", g_max_pwm);
  Serial.printf("  Min PWM: %d\n", g_min_pwm);
  Serial.println("======================================");
}

/**
 * @brief Save current parameters to EEPROM
 */
void saveParametersToEEPROM() {
  preferences.begin("joint-actuator", false);
  
  preferences.putUInt("can_addr", g_can_address);
  preferences.putFloat("setpoint", g_actuator_setpoint);
  preferences.putBool("motor_inv", g_motor_direction_inverted);
  preferences.putBool("sensor_inv", g_sensor_direction_inverted);
  preferences.putFloat("kp", KP);
  preferences.putUInt("max_pwm", g_max_pwm);
  preferences.putUInt("min_pwm", g_min_pwm);
  
  preferences.end();
  
  Serial.println("=== Saved Parameters to EEPROM ===");
  Serial.printf("  CAN Address: 0x%03X\n", g_can_address);
  Serial.printf("  Setpoint: %.2f degrees\n", g_actuator_setpoint);
  Serial.printf("  Motor Inverted: %s\n", g_motor_direction_inverted ? "YES" : "NO");
  Serial.printf("  Sensor Inverted: %s\n", g_sensor_direction_inverted ? "YES" : "NO");
  Serial.printf("  P Gain: %.3f\n", KP);
  Serial.printf("  Max PWM: %d\n", g_max_pwm);
  Serial.printf("  Min PWM: %d\n", g_min_pwm);
  Serial.println("====================================");
}

