#include "FreeRTOS.h"      // FreeRTOS core definitions (needed for task handling and timing)
#include "task.h"          // FreeRTOS task functions (e.g., vTaskDelay)
#include "debug.h"         // Debug printing functions (e.g., DEBUG_PRINT)

// Main application loop
void appMain(void *param)
{
    // Infinite 
    // // loop (runs continuously while the quadcopter is powered on)
    while (true)
    {
        // Print message to console
        DEBUG_PRINT("Hello world!\n");
        // Wait for 100 milliseconds (2 Hz loop)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}