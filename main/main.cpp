#include <esp_log.h>

// local includes
#include "antbms/antbms.h"
#include "espnow.h"

extern "C" void app_main()
{
    esp_log_level_set("*", ESP_LOG_DEBUG);

    antbms::AntBms antbms;

    espnow::wifi_init();

    espnow::init();

    while (true)
    {
        antbms.update();

        espnow::handle();

        vPortYield();

        vTaskDelay(50/portTICK_PERIOD_MS); // 50ms
    }
}
