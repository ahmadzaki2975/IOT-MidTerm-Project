#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "dht11.h"
#include "ssd1306.h"
#include <ultrasonic.h>
#include <esp_err.h>
#include "driver/uart.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_log.h"

// ? Definisi Debug Flag
#define DEBUG_FLAG false
#define DEBUG_JSON true

// ? Konfigurasi UART
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16

// ? Konfigurasi OLED
#define I2C_MASTER_SCL_IO 22      // ? GPIO untuk I2C master clock
#define I2C_MASTER_SDA_IO 21      // ? GPIO untuk I2C master data
#define I2C_MASTER_NUM I2C_NUM_1  // ? I2C port untuk master dev
#define I2C_MASTER_FREQ_HZ 100000 // ? I2C master clock frequency
static ssd1306_handle_t ssd1306_dev = NULL;

// ? Konfigurasi Ultrasonic Sensor
#define MAX_DISTANCE_CM 500 // ? max 500cm
#define TRIGGER_GPIO 5
#define ECHO_GPIO 18

// ? Konfigurasi Servo
#define SERVO_GPIO GPIO_NUM_12
#define SERVO_MIN_DUTY 900        // micro seconds (uS), for 0
#define SERVO_MAX_DUTY 3800       // micro seconds (uS),for 180
#define SERVO_TRANSITION_TIME 500 // in ms
#define ACTIVE_ANGLE 180
#define REST_ANGLE 0

int servo_duty = SERVO_MIN_DUTY;
int servo_delta = SERVO_MAX_DUTY - SERVO_MIN_DUTY;

void configureServo()
{
  ledc_timer_config_t timer_conf;
  timer_conf.duty_resolution = LEDC_TIMER_15_BIT;
  timer_conf.freq_hz = 50;
  timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer_conf.timer_num = LEDC_TIMER_2;
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t ledc_conf;
  ledc_conf.channel = LEDC_CHANNEL_2;
  ledc_conf.duty = servo_duty;
  ledc_conf.gpio_num = SERVO_GPIO;
  ledc_conf.intr_type = LEDC_INTR_DISABLE;
  ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_conf.timer_sel = LEDC_TIMER_2;
  ledc_channel_config(&ledc_conf);
  ledc_fade_func_install(0);
}

void setServoAngle(int target_angle)
{
  ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, (uint16_t)(servo_duty + (servo_delta * (target_angle / 180.0))), SERVO_TRANSITION_TIME);
  ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, LEDC_FADE_WAIT_DONE);
}

// ? Konfigurasi Push Button
#define BUTTON_GPIO GPIO_NUM_23
bool button_state = false;
SemaphoreHandle_t button_mutex;

// ? Definisi Queue
QueueHandle_t dhtQueue;
QueueHandle_t ultrasonicQueue;
QueueHandle_t jsonQueue;

// ? Definisi Struct
typedef struct
{
  int temperature;
  int humidity;
} DHTdata;

typedef struct
{
  int temperature;
  int humidity;
  uint32_t distance;
} JSONdata;

// ? Task untuk Membaca Sensor DHT11
void DHT_Task()
{
  DHTdata data;
  while (1)
  {
    // ? Read data dari sensor DHT
    data.temperature = DHT11_read().temperature;
    data.humidity = DHT11_read().humidity;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // ? Mengirim queue berisi data
    xQueueSend(dhtQueue, &data, 0);
  }
}

// ? Task untuk Membaca Sensor Ultrasonic
void Ultrasonic_Task()
{
  ultrasonic_sensor_t sensor = {
      .trigger_pin = TRIGGER_GPIO,
      .echo_pin = ECHO_GPIO};
  ultrasonic_init(&sensor);

  while (1)
  {
    uint32_t distance;
    esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
    if (res != ESP_OK)
    {
#if DEBUG_FLAG
      printf("Error: ");
#endif
      switch (res)
      {
      case ESP_ERR_ULTRASONIC_PING:
#if DEBUG_FLAG
        printf("Cannot ping (device is in invalid state)\n");
#endif
        break;
      case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
#if DEBUG_FLAG
        printf("Ping timeout (no device found)\n");
#endif
        break;
      case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
#if DEBUG_FLAG
        printf("Echo timeout (i.e. distance too big)\n");
#endif
        break;
      default:
#if DEBUG_FLAG
        printf("%d\n", res);
#endif
      }
    }
    else
    {
#if DEBUG_FLAG
      printf("Distance: %ld cm\n", distance);
#endif
      // ? Mengirim data dari sensor ultrasonic ke queue
      xQueueSend(ultrasonicQueue, &distance, 0);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ? Task untuk Menampilkan Data Sensor ke OLED dan UART
void Display_Task()
{
  uint32_t distance;
  DHTdata data_received;
  char json_str[100];
  while (1)
  {
    // ? Update display jika menerima data baru dari queue
    if (
        xQueueReceive(dhtQueue, &data_received, pdMS_TO_TICKS(100)) == pdPASS ||
        xQueueReceive(ultrasonicQueue, &distance, pdMS_TO_TICKS(100)) == pdPASS)
    {
      ssd1306_clear_screen(ssd1306_dev, 0x00);
      char temp_str[16];
      char hum_str[16];
      char dist_str[16];

      sprintf(temp_str, "%i C", data_received.temperature);
      sprintf(hum_str, "%i %%", data_received.humidity);
      sprintf(dist_str, "%ld cm", distance);

      ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)dist_str, 16, 1);
      ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)temp_str, 16, 1);
      ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)hum_str, 16, 1);

      ssd1306_refresh_gram(ssd1306_dev);
      sprintf(json_str,
              "{ \"temperature\": %i, \"humidity\": %i, \"distance\": %ld }",
              data_received.temperature,
              data_received.humidity,
              distance);
#if DEBUG_JSON
      printf("%s\n", json_str);
#endif
      // ? Mengirim data ke UART
      uart_write_bytes(UART_NUM_2, json_str, sizeof(json_str));
    }
  }
}

// ? Task untuk Mengirim data ke UART
void UART_Task()
{
  JSONdata data_received;
  while (1)
  {
    char json_str[100];
    // ? Menerima data dari queue
    if (xQueueReceive(jsonQueue, &data_received, pdMS_TO_TICKS(100)) == pdPASS)
    {
      sprintf(json_str,
              "{ \"temperature:\" %i, \"humidity:\" %i, \"distance:\" %ld }",
              data_received.temperature,
              data_received.humidity,
              data_received.distance);
#if DEBUG_JSON
      printf("%s\n", json_str);
#endif
      // ? Mengirim data ke UART
      uart_write_bytes(TX_PIN, json_str, sizeof(json_str));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ? Task untuk Menggerakkan Servo
void Servo_Task()
{
  configureServo();
  while (1)
  {
    if(xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE)
    {
      if (button_state)
      {
        setServoAngle(ACTIVE_ANGLE);
      }
      else
      {
        setServoAngle(REST_ANGLE);
      }
      xSemaphoreGive(button_mutex);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ? Task untuk Membaca Push Button
void PushButton_Task()
{
  TickType_t last_button_press_time = 0; // Initialize the time of the last button press
  while (1)
  {
    if (gpio_get_level(BUTTON_GPIO) == 0)
    {
      // Calculate the time since the last button press
      TickType_t current_time = xTaskGetTickCount();
      TickType_t time_since_last_press = current_time - last_button_press_time;
      // Check if it's been more than a debouncing delay (e.g., 200ms)
      if (time_since_last_press >= pdMS_TO_TICKS(500))
      {
        if (xSemaphoreTake(button_mutex, 0) == pdTRUE)
        {
          // Button is pressed, toggle the button state
          button_state = !button_state;
          xSemaphoreGive(button_mutex);
          #if DEBUG_FLAG
          printf("Button state: %d\n", button_state);
          #endif
          // Update the time of the last button press
          last_button_press_time = current_time;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ? Main program
void app_main(void)
{
  // ? Inisialisasi OLED
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
  ssd1306_refresh_gram(ssd1306_dev);
  ssd1306_clear_screen(ssd1306_dev, 0x00);
  char data_str[10] = {0};
  sprintf(data_str, "ZAKI");
  ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)data_str, 16, 1);
  ssd1306_refresh_gram(ssd1306_dev);

  // ? Inisialisasi Queue
  dhtQueue = xQueueCreate(10, sizeof(char[10]));
  ultrasonicQueue = xQueueCreate(10, sizeof(uint32_t));
  jsonQueue = xQueueCreate(10, sizeof(char[100]));

  // ? Inisialisasi Mutex
  button_mutex = xSemaphoreCreateMutex();

  // ? Inisialisasi pin GPIO 32 untuk sensor DHT11
  DHT11_init(GPIO_NUM_32);

  // ? Inisialisasi UART
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(UART_NUM_2, &uart_config);
  uart_set_pin(UART_NUM_2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);

  // ? Inisialisasi pin GPIO 23 untuk push button
  gpio_config_t button_config = {
      .pin_bit_mask = (1ULL << BUTTON_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE, // Enable the pull-up resistor
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE // You can enable interrupts if needed
  };
  gpio_config(&button_config);

  // ! Delay menunggu sensor stabil
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  // ? Membuat task untuk membaca sensor DHT11
  xTaskCreate(&DHT_Task, "DHT_Task", 2048, NULL, 5, NULL);
  // ? Membuat task untuk display OLED
  xTaskCreate(&Display_Task, "Display_Task", 2048, NULL, 5, NULL);
  // ? Membuat task untuk membaca sensor Ultrasonic
  xTaskCreate(&Ultrasonic_Task, "Ultrasonic_Task", 2048, NULL, 5, NULL);
  // ? Membuat task untuk menggerakkan servo
  xTaskCreate(&Servo_Task, "Servo_Task", 2048, NULL, 5, NULL);
  // ? Membuat task untuk membaca push button
  xTaskCreate(&PushButton_Task, "PushButton_Task", 2048, NULL, 5, NULL);
}