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

  // ? Membuat task untuk membaca sensor DHT11
  xTaskCreate(&DHT_Task, "DHT_Task", 2048, NULL, 5, NULL);
  // ? Membuat task untuk display OLED
  xTaskCreate(&Display_Task, "Display_Task", 2048, NULL, 5, NULL);
  // ? Membuat task untuk membaca sensor Ultrasonic
  xTaskCreate(&Ultrasonic_Task, "Ultrasonic_Task", 2048, NULL, 5, NULL);
}