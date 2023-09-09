#include "espnow.h"

// system includes
#include <cstring>
#include <string_view>
#include <deque>

// esp-idf includes
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>

// 3rd party includes
#include <fmt/core.h>
#include <espchrono.h>

constexpr const char * const TAG = "espnow";

namespace espnow {

std::deque<espnow_recv_param_t> message_queue;

void wifi_init()
{
    if (auto err = nvs_flash_init(); err != ESP_OK)
    {
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            if (auto erase_err = nvs_flash_erase(); erase_err != ESP_OK)
            {
                ESP_LOGE(TAG, "nvs_flash_erase failed: %s", esp_err_to_name(erase_err));
                return;
            }

            if (auto reinit_err = nvs_flash_init(); reinit_err != ESP_OK)
            {
                ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(reinit_err));
                return;
            }
        }
        else
        {
            ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
            return;
        }
    }

    if (auto err = esp_netif_init(); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return;
    }

    if (auto err = esp_event_loop_create_default(); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (auto err = esp_wifi_init(&cfg); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return;
    }

    if (auto err = esp_wifi_set_storage(WIFI_STORAGE_RAM); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_storage failed: %s", esp_err_to_name(err));
        return;
    }

    if (auto err = esp_wifi_set_mode(WIFI_MODE_AP); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
        return;
    }

    if (auto err = esp_wifi_start(); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return;
    }

    if (auto err = esp_wifi_set_ps(WIFI_PS_NONE); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_ps failed: %s", esp_err_to_name(err));
        return;
    }
}

void onRecv(const esp_now_recv_info* info, const uint8_t* data, int data_len)
{
    const std::string_view data_str{reinterpret_cast<const char *>(data), static_cast<size_t>(data_len)};

    size_t sep_pos = data_str.find_first_of(':');
    if (sep_pos == std::string_view::npos)
    {
        ESP_LOGE(TAG, "espnow message malformed: %s", data_str.data());
        return;
    }

    auto msg = espnow_recv_param_t{
        .content = std::string{data_str.substr(sep_pos + 1)},
        .type = std::string{data_str.substr(0, sep_pos)},
    };

    message_queue.push_back(msg);
}

void onSend(const uint8_t* mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "send_cb, status: %d", status);

    if (status != ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGE(TAG, "send_cb, status: %d", status);
    }
}

void init()
{
    if (auto err = esp_now_init(); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return;
    }

    if (auto err = esp_now_register_recv_cb(onRecv); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(err));
        return;
    }

    if (auto err = esp_now_register_send_cb(onSend); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_register_send_cb failed: %s", esp_err_to_name(err));
        return;
    }

    // add BROADCAST peer
    addPeer(broadcast_address);
}

void addPeer(const uint8_t* peer_addr)
{
    esp_now_peer_info_t peer_info;
    std::memset(&peer_info, 0, sizeof(esp_now_peer_info_t));
    std::memcpy(peer_info.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
    peer_info.channel = 0;

    peer_info.ifidx = WIFI_IF_AP;

    if (auto err = esp_now_add_peer(&peer_info); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "peer added");
}

bool send(const uint8_t* peer_addr, std::string_view msg)
{
    if (msg.size() > ESP_NOW_MAX_DATA_LEN)
    {
        ESP_LOGE(TAG, "esp_now_send failed: message too long (%.*s) (%d>%d)", msg.size(), msg.data(), msg.size(), ESP_NOW_MAX_DATA_LEN);
        return false;
    }

    if (auto err = esp_now_send(peer_addr, reinterpret_cast<const uint8_t *>(msg.data()), msg.size()); err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_send failed: %s (%.*s) to %02x:%02x:%02x:%02x:%02x:%02x", esp_err_to_name(err), msg.size(), msg.data(), peer_addr[0], peer_addr[1], peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5]);
        return false;
    }

    ESP_LOGI(TAG, "esp_now_send success (size=%d): %.*s to %02x:%02x:%02x:%02x:%02x:%02x", msg.size(), msg.size(), msg.data(), peer_addr[0], peer_addr[1], peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5]);

    return true;
}

void handle()
{
    using namespace std::chrono_literals;

    if (message_queue.empty())
    {
        return;
    }

    auto msg = message_queue.front();
    message_queue.pop_front();

    ESP_LOGI(TAG, "handle message [%s]: %s", msg.type.c_str(), msg.content.c_str());
}

} // namespace espnow
