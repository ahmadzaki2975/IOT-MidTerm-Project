#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "dht11.h"

// ? Task untuk membaca sensor DHT11
void DHT_Task() {
  while(1) {
    double temp = DHT11_read().temperature;
    double hum = DHT11_read().humidity;
    printf("Temperature: %f\n", temp);
    printf("Humidity: %f\n", hum);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  // ? Inisialisasi pin GPIO 32 untuk sensor DHT11
  DHT11_init(GPIO_NUM_32);

  // ? Membuat task untuk membaca sensor DHT11
  xTaskCreate(&DHT_Task, "DHT_Task", 2048, NULL, 5, NULL);
}