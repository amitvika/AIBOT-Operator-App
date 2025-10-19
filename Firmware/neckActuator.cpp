#include <Arduino.h>
#include "driver/twai.h" // The native ESP-IDF TWAI/CAN driver
#include <Preferences.h>

// --- Pin Definitions ---
const int CAN_STBY_PIN = 47;
const gpio_num_t CAN_TX_PIN = GPIO_NUM_48;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_34;

const int MOTOR_FORWARD_PIN = 37;  // Pin for upward motion
const int MOTOR_REVERSE_PIN = 38;  // Pin for downward motion

// --- PWM Configuration ---
const int PWM_FREQUENCY = 20000; 
const int PWM_RESOLUTION = 8;  // 8-bit (0-255)
const int MOTOR_FORWARD_CHANNEL = 0;
const int MOTOR_REVERSE_CHANNEL = 1;

// --- CAN Configuration ---
const int NECK_ACTUATOR_CAN_ID = 0x500;  // Default CAN ID for neck actuator

// --- Tuning Command CAN IDs ---
const int TUNING_BASE_ID = 0x300;
const int CMD_SET_ADDRESS = TUNING_BASE_ID + 0;      // 0x300
const int CMD_MOTOR_DIR = TUNING_BASE_ID + 2;        // 0x302
const int CMD_MAX_PWM = TUNING_BASE_ID + 5;          // 0x305
const int CMD_MIN_PWM = TUNING_BASE_ID + 6;          // 0x306
const int CMD_SAVE_PARAMS = TUNING_BASE_ID + 7;      // 0x307
const int CMD_REQUEST_TELEMETRY = TUNING_BASE_ID + 8; // 0x308
const int CMD_TELEMETRY_RESPONSE_1 = TUNING_BASE_ID + 9; // 0x309
const int CMD_TELEMETRY_RESPONSE_2 = TUNING_BASE_ID + 10; // 0x30A

// Drive command byte positions from Pi: ['f', 'b', 'l', 'r', 'u', 'd', 'l2', 'r2']
const int POS_U = 4;  // Up command position
const int POS_D = 5;  // Down command position

// --- Global Motor Speed Variables ---
float g_motor_speed = 0.0;  // Current motor speed (-1.0 to +1.0, positive = up)

// --- Tunable Parameters ---
int g_can_address = NECK_ACTUATOR_CAN_ID;  // Device CAN address (tunable)
bool g_motor_direction_inverted = false;   // Motor direction flag
int g_max_pwm = 255;                       // Maximum PWM value (0-255)
int g_min_pwm = 30;                        // Minimum PWM value (deadzone)

// --- EEPROM/Preferences Storage ---
Preferences preferences;

// --- Timing for Periodic Tasks ---
const unsigned long PERIODIC_TASK_INTERVAL = 100; // ms
unsigned long g_previous_task_millis = 0;

// --- CAN Bus Health Monitoring ---
const unsigned long CAN_HEALTH_CHECK_INTERVAL = 1000; // Check every 1 second
unsigned long g_last_can_health_check = 0;

// --- Function Prototypes ---
void setupCAN();
void receiveCAN();
void checkCANHealth();
void processDriveCommand(const twai_message_t &message);
void processTuningCommand(const twai_message_t &message);
void sendTelemetry();
void loadParametersFromEEPROM();
void saveParametersToEEPROM();
void setMotorSpeed(float speed);


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("--- Neck Actuator CAN Controller (Listening for ID: 0x%03X) ---\n", NECK_ACTUATOR_CAN_ID);

  // Load saved parameters from EEPROM
  loadParametersFromEEPROM();

  setupCAN();

  // Setup PWM channels
  ledcSetup(MOTOR_FORWARD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR_REVERSE_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  
  ledcAttachPin(MOTOR_FORWARD_PIN, MOTOR_FORWARD_CHANNEL);
  ledcAttachPin(MOTOR_REVERSE_PIN, MOTOR_REVERSE_CHANNEL);
  Serial.println("Motor PWM outputs configured.");
}

void loop() {
  // Check CAN bus health periodically
  checkCANHealth();
  
  // Receive and process CAN messages
  receiveCAN();
  
  // Apply motor speed
  setMotorSpeed(g_motor_speed);

  // Periodic status reporting
  unsigned long current_millis = millis();
  if (current_millis - g_previous_task_millis >= PERIODIC_TASK_INTERVAL) {
    g_previous_task_millis = current_millis;
    Serial.printf("Motor Speed: %.2f (%.0f%%)\n", g_motor_speed, g_motor_speed * 100.0);
  }
}

/**
 * @brief Process incoming drive commands (U/D values)
 */
void processDriveCommand(const twai_message_t &message) {
  if (message.data_length_code < 8) {
    Serial.printf("ERROR: Expected 8-byte payload, got %d bytes\n", message.data_length_code);
    return;
  }
  
  uint8_t u_value = message.data[POS_U];  // Up command (0-100)
  uint8_t d_value = message.data[POS_D];  // Down command (0-100)
  
  // Scale values from 0-100 to 0.0-1.0
  float up_scaled = (float)u_value / 100.0;
  float down_scaled = (float)d_value / 100.0;
  
  // Calculate motor speed: positive = up, negative = down
  g_motor_speed = up_scaled - down_scaled;
  
  Serial.printf("-> Drive Cmd: U=%d, D=%d -> Speed=%.2f\n", u_value, d_value, g_motor_speed);
}

/**
 * @brief Receive and route CAN messages
 */
void receiveCAN() {
  twai_message_t message;
  if (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier == g_can_address) {
      processDriveCommand(message);
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
 * @param speed Motor speed from -1.0 (full down) to +1.0 (full up)
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
    // Upward motion
    pwm_value = (int)(speed * (g_max_pwm - g_min_pwm) + g_min_pwm);
    if (pwm_value > g_max_pwm) pwm_value = g_max_pwm;
    
    ledcWrite(MOTOR_FORWARD_CHANNEL, pwm_value);
    ledcWrite(MOTOR_REVERSE_CHANNEL, 0);
  } 
  else if (speed < 0) {
    // Downward motion
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
      
    case CMD_MOTOR_DIR:
      if (message.data_length_code >= 3) {
        uint16_t target_address = (message.data[0] << 8) | message.data[1];
        if (target_address == g_can_address || target_address == 0xFFFF) {
          g_motor_direction_inverted = (message.data[2] != 0);
          Serial.printf("-> Motor direction inverted: %s\n", g_motor_direction_inverted ? "YES" : "NO");
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
 * Note: Neck actuator only sends one telemetry message since it has fewer parameters
 */
void sendTelemetry() {
  twai_message_t telemetry_msg;
  telemetry_msg.extd = 0;
  telemetry_msg.rtr = 0;
  telemetry_msg.ss = 0;
  telemetry_msg.self = 0;
  telemetry_msg.dlc_non_comp = 0;
  
  // Telemetry Response: address(2), motor_dir(1), max_pwm(1), min_pwm(1), reserved(3)
  telemetry_msg.identifier = CMD_TELEMETRY_RESPONSE_1;
  telemetry_msg.data_length_code = 8;
  telemetry_msg.data[0] = (g_can_address >> 8) & 0xFF;
  telemetry_msg.data[1] = g_can_address & 0xFF;
  telemetry_msg.data[2] = g_motor_direction_inverted ? 1 : 0;
  telemetry_msg.data[3] = 0;  // No sensor direction for neck actuator
  telemetry_msg.data[4] = g_max_pwm;
  telemetry_msg.data[5] = g_min_pwm;
  telemetry_msg.data[6] = 0;  // Reserved
  telemetry_msg.data[7] = 0;  // Reserved
  
  if (twai_transmit(&telemetry_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("-> Telemetry sent");
  } else {
    Serial.println("-> Failed to send telemetry");
  }
  
  Serial.printf("-> Telemetry: Addr=0x%03X, MotorDir=%d, MaxPWM=%d, MinPWM=%d\n",
                g_can_address, g_motor_direction_inverted, g_max_pwm, g_min_pwm);
}

/**
 * @brief Load saved parameters from EEPROM
 */
void loadParametersFromEEPROM() {
  preferences.begin("neck-actuator", false);
  
  g_can_address = preferences.getUInt("can_addr", NECK_ACTUATOR_CAN_ID);
  g_motor_direction_inverted = preferences.getBool("motor_inv", false);
  g_max_pwm = preferences.getUInt("max_pwm", 255);
  g_min_pwm = preferences.getUInt("min_pwm", 30);
  
  preferences.end();
  
  Serial.println("=== Loaded Parameters from EEPROM ===");
  Serial.printf("  CAN Address: 0x%03X\n", g_can_address);
  Serial.printf("  Motor Inverted: %s\n", g_motor_direction_inverted ? "YES" : "NO");
  Serial.printf("  Max PWM: %d\n", g_max_pwm);
  Serial.printf("  Min PWM: %d\n", g_min_pwm);
  Serial.println("======================================");
}

/**
 * @brief Save current parameters to EEPROM
 */
void saveParametersToEEPROM() {
  preferences.begin("neck-actuator", false);
  
  preferences.putUInt("can_addr", g_can_address);
  preferences.putBool("motor_inv", g_motor_direction_inverted);
  preferences.putUInt("max_pwm", g_max_pwm);
  preferences.putUInt("min_pwm", g_min_pwm);
  
  preferences.end();
  
  Serial.println("=== Saved Parameters to EEPROM ===");
  Serial.printf("  CAN Address: 0x%03X\n", g_can_address);
  Serial.printf("  Motor Inverted: %s\n", g_motor_direction_inverted ? "YES" : "NO");
  Serial.printf("  Max PWM: %d\n", g_max_pwm);
  Serial.printf("  Min PWM: %d\n", g_min_pwm);
  Serial.println("====================================");
}

