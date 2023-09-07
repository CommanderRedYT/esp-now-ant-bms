#pragma once
constexpr const char *const TAG = "AntBms";

// system includes
#include <vector>
#include <string>
#include <string_view>

// 3rdparty includes
#include <espchrono.h>
#include <NimBLEDevice.h>

using namespace std::chrono_literals;

namespace antbms {

struct AntBmsData
{
    float power;
    std::string toString();
};

class AntBms
{
public:
    AntBms();

    // interaction with class
    void update();

    void set_interval(espchrono::millis_clock::duration interval)
    { m_interval = interval; }

    void set_password(const std::string &password)
    { m_password = password; }

    void set_password(const char *password)
    { m_password = password; }

    void set_password(const uint8_t *password, uint8_t password_length)
    { m_password = std::string((const char *) password, password_length); }

    void set_password(std::string_view password)
    { m_password = std::string(password); }

    // bms functions
    bool send_(uint8_t function, uint16_t address, uint8_t value, bool authenticate);

    bool authenticate_();

    bool authenticate_variable_(const uint8_t *data, uint8_t data_length);

    void on_ant_bms_ble_data_(const uint8_t &function, const std::vector<uint8_t> &data);

    void on_status_data_(const std::vector<uint8_t> &data);

    void on_device_info_data_(const std::vector<uint8_t> &data);

    void assemble(const uint8_t *data, uint8_t data_length);

    void write_register(uint16_t address, uint8_t value);

    void push_advertised_device(NimBLEAdvertisedDevice *advertised_device)
    { m_ble_devices.push_back(advertised_device); }

private:
    std::string m_password;
    std::vector<uint8_t> m_frame_buffer;
    espchrono::millis_clock::duration m_interval = 1s;
    espchrono::millis_clock::time_point m_last_update = espchrono::millis_clock::now();

    NimBLERemoteCharacteristic *m_ant_bms_remote_characteristic = nullptr;
    NimBLEScan *m_ble_scan = nullptr;
    NimBLEClient *m_ble_client = nullptr;
    NimBLERemoteCharacteristic *m_ble_characteristic = nullptr;
    std::vector<NimBLEAdvertisedDevice *> m_ble_devices;

    void ble_connect(NimBLEAddress address);

    enum DeviceState
    {
        DEVICE_DISCONNECTED,
        DEVICE_CONNECTED,
        DEVICE_AUTHENTICATED,
    } m_device_state = DEVICE_DISCONNECTED;

    enum BleState
    {
        BLE_IDLE,
        BLE_SCANNING,
        BLE_CONNECTING,
        BLE_CONNECTED,
    } m_ble_state = BLE_IDLE;

    // NimBLE callbacks
    class OnScanResults : public NimBLEScanCallbacks
    {
    public:
        explicit OnScanResults(AntBms &ant_bms)
                : m_ant_bms{ant_bms}
        {}

        void onDiscovered(NimBLEAdvertisedDevice *advertised_device) override;

    private:
        AntBms &m_ant_bms;
    } m_on_scan_results;

    class OnClientCallback : public NimBLEClientCallbacks
    {
    public:
        explicit OnClientCallback(AntBms &ant_bms)
                : m_ant_bms{ant_bms}
        {}

        void onDisconnect(NimBLEClient *pClient, int reason) override;

    private:
        AntBms &m_ant_bms;
    } m_on_client_events;

    class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
    {
    public:
        explicit CharacteristicCallbacks(AntBms &ant_bms)
                : m_ant_bms{ant_bms}
        {}

        void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override;

    private:
        AntBms &m_ant_bms;
    } m_characteristics_callbacks;

    void m_notifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic,
                        uint8_t *pData, size_t length, bool isNotify);

    AntBmsData m_bmsData;
};

} // namespace antbms
