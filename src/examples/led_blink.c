#include "FreeRTOS.h"      // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"          // FreeRTOS task functions (e.g., vTaskDelay)
#include "led.h"           // LED functions (e.g., ledSet)

// Main application loop
void appMain(void *param)
{
    // Infinite loop (runs continuously while the quadcopter is powered on)
    while (true)
    {
        // Turn on left green led
        ledSet(LED_GREEN_L, true);
        // Wait for 100 milliseconds (2 Hz loop)
        vTaskDelay(pdMS_TO_TICKS(100));
        // Turn off left green led
        ledSet(LED_GREEN_L, false);
        // Wait for 100 milliseconds (2 Hz loop)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}