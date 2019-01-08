/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include <si_7006.h>

void app_main() { 
    Status_t ret;
    Failure_t failure;
    float humidity;
    float temperature;

    ret = Si7006_init(&failure);
    while (1) {
        if (ret != SUCCESS) {
            printf("Error during initialization!\n\r");
        }
        else {
            Si7006_readHumidity(&failure, &humidity, NO_HOLD);
            printf("Humidity: %f\n\r", humidity);
            Si7006_readTemperature(&failure, &temperature, NO_HOLD);
            printf("Temperature: %f°C\n\r", temperature);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
