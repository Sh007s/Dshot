#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DShotRMT.h"
#include "esp_log.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define MIN_THROTTLE 48
#define FULL_THROTTLE 2047
#define MOTOR_COUNT 1
#define MOTOR_POLES 14

static const char *TAG = "dshot-bidirectional-example";

DShotRMT bldc[4];
gpio_num_t motor_gpio[4] = { GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_19 };
rmt_channel_t rmt_channel[4] = { RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3 };

// New function to send bidirectional throttle with direction
static void sendBidirectionalThrottle(uint8_t motor, int throttle, bool reverse) {
  // In bidirectional mode, throttle values are modified
  // Normal direction: 1048-2047
  // Reverse direction: 0-1047
  uint16_t adjusted_throttle;
  
  if (reverse) {
    // Reverse direction (0-1047)
    adjusted_throttle = throttle > 1047 ? 1047 : throttle;
  } else {
    // Forward direction (1048-2047)
    adjusted_throttle = throttle < 1048 ? 1048 : throttle;
  }
  
  bldc[motor].sendThrottle(adjusted_throttle);
}

static void rampThrottle(uint8_t motor, int start, int stop, int step, bool reverse) {
  if (step == 0)
    return;
  
  for (int i = start; step > 0 ? i < stop : i > stop; i += step) {
    sendBidirectionalThrottle(motor, i, reverse);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  sendBidirectionalThrottle(motor, stop, reverse);
}

TaskHandle_t Task1;
void task1(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    // Read data from Serial2
    if (Serial2.available() >= 10) {
      char data[10];
      Serial2.readBytes(data, 10);
      uint16_t erpm = (data[8] << 8) | data[9];
      int rpm = (erpm * 100) / (MOTOR_POLES / 2);
      Serial.printf("Telemetry: %d RPM\n", rpm);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  ESP_LOGI(TAG, "Initializing Bidirectional DShot RMT");
  
  // Initialize motors and DShotRMT instances
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (bldc[i].install(motor_gpio[i], rmt_channel[i]) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to install DShotRMT for motor %d", i);
      while (1);  // Halt execution
    }
    if (bldc[i].init() != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize DShotRMT for motor %d", i);
      while (1);  // Halt execution
    }
  }
  
  // Create FreeRTOS task for telemetry reading
  xTaskCreate(task1, "Task1", 2048, NULL, 1, &Task1);
  
  // Hold stop for 3 seconds
  const TickType_t holdStop = xTaskGetTickCount() + (3000 / portTICK_PERIOD_MS);
  while (xTaskGetTickCount() < holdStop) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      sendBidirectionalThrottle(i, MIN_THROTTLE, false);  // Forward direction
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // Forward direction ramp up
  ESP_LOGI(TAG, "Ramping throttle forward");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rampThrottle(i, MIN_THROTTLE, FULL_THROTTLE, 20, false);
  }
  
  // Hold full throttle for 3 seconds
  const TickType_t holdStop = xTaskGetTickCount() + (3000 / portTICK_PERIOD_MS);
  while (xTaskGetTickCount() < holdStop) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      sendBidirectionalThrottle(i, FULL_THROTTLE, false);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  
  // Ramp down
  Serial.println("Ramping down");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rampThrottle(i, FULL_THROTTLE, MIN_THROTTLE, -20, false);
  }
  
  // Reverse direction ramp up
  ESP_LOGI(TAG, "Ramping throttle reverse");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rampThrottle(i, MIN_THROTTLE, FULL_THROTTLE, 20, true);
  }
  
  // Hold full reverse throttle for 3 seconds
  const TickType_t holdReverseStop = xTaskGetTickCount() + (3000 / portTICK_PERIOD_MS);
  while (xTaskGetTickCount() < holdReverseStop) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      sendBidirectionalThrottle(i, FULL_THROTTLE, true);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  
  // Ramp down in reverse
  Serial.println("Ramping down reverse");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rampThrottle(i, FULL_THROTTLE, MIN_THROTTLE, -20, true);
  }
}