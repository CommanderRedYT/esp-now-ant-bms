#include "antbms.h"

// esp-idf includes
#include <esp_log.h>

// 3rdparty includes
#include <NimBLEDevice.h>

// local includes
#include "helpers/crc16.h"
#include "helpers/format_hex_pretty.h"
#include "espnow.h"

namespace antbms {
constexpr static const uint16_t ANT_BMS_SERVICE_UUID = 0xFFE0;
constexpr static const uint16_t ANT_BMS_CHARACTERISTIC_UUID = 0xFFE1;

constexpr static const uint8_t MAX_RESPONSE_SIZE = 152;

constexpr static const uint8_t ANT_PKT_START_1 = 0x7E;
constexpr static const uint8_t ANT_PKT_START_2 = 0xA1;
constexpr static const uint8_t ANT_PKT_END_1 = 0xAA;
constexpr static const uint8_t ANT_PKT_END_2 = 0x55;

constexpr static const uint8_t ANT_FRAME_TYPE_STATUS = 0x11;
constexpr static const uint8_t ANT_FRAME_TYPE_DEVICE_INFO = 0x12;
constexpr static const uint8_t ANT_FRAME_TYPE_SYSTEM_LOG = 0x13;
constexpr static const uint8_t ANT_FRAME_TYPE_PERMISSION = 0x14;
constexpr static const uint8_t ANT_FRAME_TYPE_SYSTEM_INFO = 0x15;
constexpr static const uint8_t ANT_FRAME_TYPE_GPS_DATA = 0x16;
constexpr static const uint8_t ANT_FRAME_TYPE_UNKNOWN1 = 0x42;
constexpr static const uint8_t ANT_FRAME_TYPE_UNKNOWN2 = 0x43;
constexpr static const uint8_t ANT_FRAME_TYPE_UNKNOWN3 = 0x61;

constexpr static const uint8_t ANT_COMMAND_STATUS = 0x01;
constexpr static const uint8_t ANT_COMMAND_DEVICE_INFO = 0x02;
constexpr static const uint8_t ANT_COMMAND_WRITE_REGISTER = 0x51;

constexpr static const uint8_t CHARGE_MOSFET_STATUS_SIZE = 16;
static const char *const CHARGE_MOSFET_STATUS[CHARGE_MOSFET_STATUS_SIZE] = {
        "Off",                           // 0x00
        "On",                            // 0x01
        "Overcharge protection",         // 0x02
        "Over current protection",       // 0x03
        "Battery full",                  // 0x04
        "Total overpressure",            // 0x05
        "Battery over temperature",      // 0x06
        "MOSFET over temperature",       // 0x07
        "Abnormal current",              // 0x08
        "Balanced line dropped string",  // 0x09
        "Motherboard over temperature",  // 0x0A
        "Unknown",                       // 0x0B
        "Unknown",                       // 0x0C
        "Discharge MOSFET abnormality",  // 0x0D
        "Unknown",                       // 0x0E
        "Manually turned off",           // 0x0F
};

static const uint8_t DISCHARGE_MOSFET_STATUS_SIZE = 16;
static const char *const DISCHARGE_MOSFET_STATUS[DISCHARGE_MOSFET_STATUS_SIZE] = {
        "Off",                           // 0x00
        "On",                            // 0x01
        "Overdischarge protection",      // 0x02
        "Over current protection",       // 0x03
        "Unknown",                       // 0x04
        "Total pressure undervoltage",   // 0x05
        "Battery over temperature",      // 0x06
        "MOSFET over temperature",       // 0x07
        "Abnormal current",              // 0x08
        "Balanced line dropped string",  // 0x09
        "Motherboard over temperature",  // 0x0A
        "Charge MOSFET on",              // 0x0B
        "Short circuit protection",      // 0x0C
        "Discharge MOSFET abnormality",  // 0x0D
        "Start exception",               // 0x0E
        "Manually turned off",           // 0x0F
};

static const uint8_t BALANCER_STATUS_SIZE = 11;
static const char *const BALANCER_STATUS[BALANCER_STATUS_SIZE] = {
        "Off",                                   // 0x00
        "Exceeds the limit equilibrium",         // 0x01
        "Charge differential pressure balance",  // 0x02
        "Balanced over temperature",             // 0x03
        "Automatic equalization",                // 0x04
        "Unknown",                               // 0x05
        "Unknown",                               // 0x06
        "Unknown",                               // 0x07
        "Unknown",                               // 0x08
        "Unknown",                               // 0x09
        "Motherboard over temperature",          // 0x0A
};

std::string format_total_runtime_(const uint32_t value)
{
    int seconds = (int) value;
    int years = seconds / (24 * 3600 * 365);
    seconds = seconds % (24 * 3600 * 365);
    int days = seconds / (24 * 3600);
    seconds = seconds % (24 * 3600);
    int hours = seconds / 3600;
    return (years ? std::to_string(years) + "y " : "") + (days ? std::to_string(days) + "d " : "") +
           (hours ? std::to_string(hours) + "h" : "");
}

bool AntBms::send_(uint8_t function, uint16_t address, uint8_t value, bool authenticate)
{
    ESP_LOGI(TAG, "Executing send");
    if (authenticate)
    {
        authenticate_();
    }

    uint8_t frame[10];
    frame[0] = 0x7e;          // header
    frame[1] = 0xa1;          // header
    frame[2] = function;      // control
    frame[3] = address >> 0;  // address
    frame[4] = address >> 8;  // address
    frame[5] = value;         // value
    auto crc = helpers::crc16(frame + 1, 5);
    frame[6] = crc >> 0;  // CRC
    frame[7] = crc >> 8;  // CRC
    frame[8] = 0xaa;      // footer
    frame[9] = 0x55;      // footer

    ESP_LOGV(TAG, "Send command: %s", format_hex_pretty(frame, sizeof(frame)).c_str());

    if (m_ble_characteristic)
    {
        if (m_ble_characteristic->canWrite())
        {
            if (m_ble_characteristic->writeValue(frame, sizeof(frame), false))
            {
                ESP_LOGI(TAG, "Write successful");
                return true;
            }
            else
            {
                ESP_LOGW(TAG, "Write failed");
            }
        }
        else
        {
            ESP_LOGW(TAG, "Characteristic cannot be written");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Characteristic not found");
    }

    return false;
}

bool AntBms::authenticate_()
{
    uint8_t frame[22];

    frame[0] = 0x7e;   // header
    frame[1] = 0xa1;   // header
    frame[2] = 0x23;   // control
    frame[3] = 0x6a;   // address
    frame[4] = 0x01;   // address
    frame[5] = 0x0c;   // data len
    frame[6] = 0x31;   // 1
    frame[7] = 0x32;   // 2
    frame[8] = 0x33;   // 3
    frame[9] = 0x34;   // 4
    frame[10] = 0x35;  // 5
    frame[11] = 0x36;  // 6
    frame[12] = 0x37;  // 7
    frame[13] = 0x38;  // 8
    frame[14] = 0x39;  // 9
    frame[15] = 0x61;  // a
    frame[16] = 0x62;  // b
    frame[17] = 0x63;  // c

    auto crc = helpers::crc16(frame + 1, 17);
    frame[18] = crc >> 0;  // CRC
    frame[19] = crc >> 8;  // CRC

    frame[20] = 0xaa;
    frame[21] = 0x55;

    ESP_LOGV(TAG, "Send command: %s", format_hex_pretty(frame, sizeof(frame)).c_str());

    if (m_ble_characteristic)
    {
        if (m_ble_characteristic->canWrite())
        {
            if (m_ble_characteristic->writeValue(frame, sizeof(frame), false))
            {
                ESP_LOGI(TAG, "Write successful");
                return true;
            }
            else
            {
                ESP_LOGW(TAG, "Write failed");
            }
        }
        else
        {
            ESP_LOGW(TAG, "Characteristic cannot be written");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Characteristic not found");
    }

    return false;
}

bool AntBms::authenticate_variable_(const uint8_t *data, uint8_t data_length)
{
    std::vector<uint8_t> frame = {0x7E, 0xA1, 0x23, 0x6A, 0x01};
    frame.push_back(data_length);
    for (int i = 0; i < data_length; i++)
    {
        frame.push_back(data[i]);
    }
    auto crc = helpers::crc16(frame.data() + 1, frame.size());
    frame.push_back(crc >> 0);
    frame.push_back(crc >> 8);
    frame.push_back(0xAA);
    frame.push_back(0x55);

    ESP_LOGV(TAG, "Send command: %s", format_hex_pretty(&frame.front(), frame.size()).c_str());

    if (m_ble_characteristic)
    {
        if (m_ble_characteristic->canWrite())
        {
            if (m_ble_characteristic->writeValue(frame.data(), frame.size(), false))
            {
                ESP_LOGI(TAG, "Write successful");
                return true;
            }
            else
            {
                ESP_LOGW(TAG, "Write failed");
            }
        }
        else
        {
            ESP_LOGW(TAG, "Characteristic cannot be written");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Characteristic not found");
    }


    return false;
}

void AntBms::on_ant_bms_ble_data_(const uint8_t &function, const std::vector<uint8_t> &data)
{
    switch (function)
    {
    case ANT_FRAME_TYPE_STATUS:
        on_status_data_(data);
        break;
    case ANT_FRAME_TYPE_DEVICE_INFO:
        on_device_info_data_(data);
        break;
    default:
        ESP_LOGW(TAG, "Unhandled response received (function 0x%02X): %s", function, format_hex_pretty(data).c_str());
    }
}

void AntBms::on_status_data_(const std::vector<uint8_t> &data)
{
    auto ant_get_16bit = [&](size_t i) -> uint16_t {
        return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0);
    };
    auto ant_get_32bit = [&](size_t i) -> uint32_t {
        return (uint32_t(ant_get_16bit(i + 2)) << 16) | (uint32_t(ant_get_16bit(i + 0)) << 0);
    };

    ESP_LOGD(TAG, "Status frame (%d bytes):", data.size());

    if (data.size() != (6 + data[5] + 4))
    {
        ESP_LOGW(TAG, "Skipping status frame because of invalid length");
        return;
    }

    // Status request
    // -> 0x7e 0xa1 0x01 0x00 0x00 0xbe 0x18 0x55 0xaa 0x55
    //
    // Status response
    //
    // Byte Len Payload     Description                      Unit  Precision
    //   0   2  0x7E 0xA1   Start of frame
    //   2   1  0x11        Function
    //   3   2  0x00 0x00   Address
    //   5   1  0x8E        Data length
    //   6   1  0x05        Permissions
    ESP_LOGV(TAG, "  Permissions: %d", data[6]);

    //   7   1  0x01        Battery status (0: Unknown, 1: Idle, 2: Charge, 3: Discharge, 4: Standby, 5: Error)
    ESP_LOGV(TAG, "  Battery status: %d", data[7]);
    m_bmsData.battery_status = static_cast<BatteryStatus>(data[7]);

    //   8   1  0x04        Number of temperature sensors       max 4.
    uint8_t temperature_sensors = data[8];
    ESP_LOGV(TAG, "  Number of temperature sensors: %d", temperature_sensors);

    //   9   1  0x0E        Number of cells (14)                max 32
    uint8_t cells = data[9];

    //  10   8  0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00   Protection bitmask
    //  18   8  0x00 0x00 0x00 0x01 0x00 0x00 0x00 0x00   Warning bitmask
    //  26   8  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00   Balancing? bitmask

    //  34   2  0x11 0x10   Cell voltage 1             uint16_t
    //  36   2  0x11 0x10   Cell voltage 2
    //  38   2  0x11 0x10   Cell voltage 3
    //  40   2  0x11 0x10   Cell voltage 4
    //  42   2  0x11 0x10   Cell voltage 5
    //  44   2  0x11 0x10   Cell voltage 6
    //  46   2  0x11 0x10   Cell voltage 7
    //  48   2  0x11 0x10   Cell voltage 8
    //  50   2  0x11 0x10   Cell voltage 9
    //  52   2  0x11 0x10   Cell voltage 10
    //  54   2  0x11 0x10   Cell voltage 11
    //  56   2  0x11 0x10   Cell voltage 12
    //  58   2  0x11 0x10   Cell voltage 13
    //  60   2  0x11 0x10   Cell voltage 14            uint16_t
    m_bmsData.cell_voltages.clear();
    for (uint8_t i = 0; i < cells; i++)
    {
        m_bmsData.cell_voltages.push_back(ant_get_16bit(i * 2 + 34) * 0.001f);
    }

    uint8_t offset = cells * 2;

    //  62   2  0x1C 0x00   Temperature sensor 1        int16_t
    //  64   2  0x1C 0x00   Temperature sensor 2        int16_t
    //  66   2  0xD8 0xFF   Temperature sensor 3        int16_t
    //  68   2  0x1C 0x00   Temperature sensor 4        int16_t
    //  70   2  0x1C 0x00   Mosfet temperature          int16_t
    //  72   2  0x1C 0x00   Balancer temperature        int16_t
    m_bmsData.temperatures.clear();
    for (uint8_t i = 0; i < temperature_sensors; i++)
    {
        m_bmsData.temperatures.push_back(((int16_t) ant_get_16bit(i * 2 + 34 + offset)) * 1.0f);
    }

    offset = offset + (temperature_sensors * 2);
    //  70   2  0x1C 0x00   Mosfet temperature          int16_t
    m_bmsData.mosfet_temperature = ((int16_t) ant_get_16bit(34 + offset)) * 1.0f;

    //  72   2  0x1C 0x00   Balancer temperature        int16_t
    m_bmsData.balancer_temperature = ((int16_t) ant_get_16bit(36 + offset)) * 1.0f;

    //  74   2  0x7E 0x16   Total voltage              uint16_t
    m_bmsData.total_voltage = ant_get_16bit(38 + offset) * 0.01f;

    //  76   2  0x00 0x00   Current                     int16_t
    m_bmsData.current = ((int16_t) ant_get_16bit(40 + offset)) * 0.1f;

    //  78   2  0x60 0x00   State of charge            uint16_t
    m_bmsData.state_of_charge = ((int16_t) ant_get_16bit(42 + offset)) * 1.0f;

    //  80   2  0x64 0x00   State of health            uint16_t
    //ESP_LOGI(TAG, "  State of health: %.0f %%", ant_get_16bit(44 + offset) * 1.0f);
    m_bmsData.state_of_health = ((int16_t) ant_get_16bit(44 + offset)) * 1.0f;


    //  82   1  0x01        Charge MOS status
    uint8_t raw_charge_mosfet_status = data[46 + offset];
    m_bmsData.charge_mosfet_status = static_cast<ChargeMosfetStatus>(raw_charge_mosfet_status);
    m_bmsData.charge_mosfet_status_string = CHARGE_MOSFET_STATUS[raw_charge_mosfet_status];

    //  83   1  0x02        Discharge MOS status
    uint8_t raw_discharge_mosfet_status = data[47 + offset];
    m_bmsData.discharge_mosfet_status = static_cast<DischargeMosfetStatus>(raw_discharge_mosfet_status);
    m_bmsData.discharge_mosfet_status_string = DISCHARGE_MOSFET_STATUS[raw_discharge_mosfet_status];

    //  84   1  0x00        Balancer status
    uint8_t raw_balancer_status = data[48 + offset];
    m_bmsData.balancer_status = static_cast<BalancerStatus>(raw_balancer_status);
    m_bmsData.balancer_status_string = BALANCER_STATUS[raw_balancer_status];

    //  85   1  0x00        Reserved
    //  86   4  0x80 0xC3 0xC9 0x01    Battery capacity            uint32_t
    m_bmsData.total_battery_capacity_setting = ant_get_32bit(50 + offset) * 0.000001f;

    //  90   4  0x4F 0x55 0xB3 0x01    Battery capacity remaining  uint32_t
    m_bmsData.capacity_remaining = ant_get_32bit(54 + offset) * 0.000001f;

    //  94   4  0x08 0x53 0x00 0x00    Total battery cycles capacity     uint32_t
    m_bmsData.battery_cycle_capacity = ant_get_32bit(58 + offset) * 0.001f;

    //  98   4  0x00 0x00 0x00 0x00    Power
    m_bmsData.power = ((int32_t) ant_get_32bit(62 + offset)) * 1.0f;


    // 102   4  0x6B 0x28 0x12 0x00    Total runtime
    m_bmsData.total_runtime = ant_get_32bit(66 + offset);
    m_bmsData.total_runtime_formatted = format_total_runtime_(ant_get_32bit(66 + offset));

    // 106   4  0x00 0x00 0x00 0x00    Balanced cell bitmask
    m_bmsData.balanced_cell_bitmask = ant_get_32bit(70 + offset);

    // 110   2  0x11 0x10              Maximum cell voltage
    m_bmsData.max_cell_voltage = ant_get_16bit(74 + offset) * 0.001f;

    // 112   2  0x01 0x00              Maximum voltage cell
    m_bmsData.max_voltage_cell = ant_get_16bit(76 + offset) * 1.0f;

    // 114   2  0x11 0x10              Minimum cell voltage
    m_bmsData.min_cell_voltage = ant_get_16bit(78 + offset) * 0.001f;

    // 116   2  0x01 0x00              Minimum voltage cell
    m_bmsData.min_voltage_cell = ant_get_16bit(80 + offset) * 1.0f;

    // 118   2  0x00 0x00              Delta cell voltage
    m_bmsData.delta_cell_voltage = ant_get_16bit(82 + offset) * 0.001f;

    // 120   2  0x11 0x10              Average cell voltage
    m_bmsData.average_cell_voltage = ant_get_16bit(84 + offset) * 0.001f;

    // 122   2  0x02 0x00              Discharge MOSFET, voltage between D-S
    // 124   2  0x70 0x00              Drive voltage (discharge MOSFET)
    // 126   2  0x03 0x00              Drive voltage (charge MOSFET)
    // 128   2  0xAC 0x02              F40com
    // 130   2  0xF1 0xFA              Battery type (0xfaf1: Ternary Lithium, 0xfaf2: Lithium Iron Phosphate,
    //                                               0xfaf3: Lithium Titanate, 0xfaf4: Custom)
    // 132   4  0x7D 0x2E 0x00 0x00    Accumulated discharging capacity
    m_bmsData.accumulated_discharging_capacity = ant_get_32bit(96 + offset) * 0.001f;

    // 136   4  0x94 0x77 0x00 0x00    Accumulated charging capacity
    m_bmsData.accumulated_charging_capacity = ant_get_32bit(100 + offset) * 0.001f;

    // 140   4  0xDE 0x07 0x00 0x00    Accumulated discharging time
    m_bmsData.accumulated_discharging_time = ant_get_32bit(104 + offset);
    m_bmsData.accumulated_discharging_time_formatted = format_total_runtime_(ant_get_32bit(104 + offset));

    // 144   4  0x77 0x76 0x00 0x00    Accumulated charging time
    m_bmsData.accumulated_charging_time = ant_get_32bit(108 + offset);
    m_bmsData.accumulated_charging_time_formatted = format_total_runtime_(ant_get_32bit(108 + offset));

    // 148   2  0x35 0xE2              CRC
    // 150   2  0xAA 0x55              End of frame
}

void AntBms::on_device_info_data_(const std::vector<uint8_t> &data)
{
    ESP_LOGI(TAG, "Device info frame (%d bytes):", data.size());

    // Status request
    // -> 0x7e 0xa1 0x02 0x6c 0x02 0x20 0x58 0xc4 0xaa 0x55
    //
    // Device info response
    //
    // Byte Len Payload     Description                      Unit  Precision
    //   0   2  0x7E 0xA1   Start of frame
    //   2   1  0x12        Function
    //   3   2  0x6C 0x02   Address
    //   5   1  0x20        Data length (32 bytes!)
    //   6  16  0x31 0x36 0x5A 0x4D 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00    Hardware version
    m_bmsData.hardware_version = std::string(data.begin() + 6, data.begin() + 6 + 16);

    //  22  16  0x31 0x36 0x5A 0x4D 0x55 0x42 0x30 0x30 0x2D 0x32 0x31 0x31 0x30 0x32 0x36 0x41    Software version
    m_bmsData.software_version = std::string(data.begin() + 22, data.begin() + 22 + 16);

    //  38   2  0x72 0x08   CRC
    //  40   1  0xFF        Reserved
    //  41   1  0x0B        Reserved
    //  42   1  0x00        Reserved
    //  43   1  0x00        Reserved
    //  44   2  0x41 0xF2   CRC unused
    //  46   2  0xAA 0x55   End of frame
}

void AntBms::assemble(const uint8_t *data, uint8_t data_length)
{
    if (m_frame_buffer.size() > MAX_RESPONSE_SIZE)
    {
        ESP_LOGW(TAG, "Maximum response size (%d bytes) exceeded", m_frame_buffer.size());
        m_frame_buffer.clear();
    }

    // Flush buffer on every preamble
    if (data[0] == ANT_PKT_START_1 && data[1] == ANT_PKT_START_2)
    {
        m_frame_buffer.clear();
    }

    m_frame_buffer.insert(m_frame_buffer.end(), data, data + data_length);

    if (m_frame_buffer.back() == ANT_PKT_END_2)
    {
        const uint8_t *raw = &m_frame_buffer[0];

        uint8_t function = raw[2];
        uint16_t data_len = raw[5];
        uint16_t frame_len = 6 + data_len + 4;
        // It looks like the data_len value of the device info frame is wrong
        if (frame_len != m_frame_buffer.size() && function != ANT_FRAME_TYPE_DEVICE_INFO)
        {
            ESP_LOGW(TAG, "Invalid frame length");
            m_frame_buffer.clear();
            return;
        }

        uint16_t computed_crc = helpers::crc16(raw + 1, frame_len - 5);
        uint16_t remote_crc = uint16_t(raw[frame_len - 3]) << 8 | (uint16_t(raw[frame_len - 4]) << 0);
        if (computed_crc != remote_crc)
        {
            ESP_LOGW(TAG, "CRC Check failed! %04X != %04X", computed_crc, remote_crc);
            m_frame_buffer.clear();
            return;
        }

        std::vector<uint8_t> assembled_data(m_frame_buffer.begin(), m_frame_buffer.end());

        on_ant_bms_ble_data_(function, assembled_data);
        m_frame_buffer.clear();
    }
}

void AntBms::write_register(uint16_t address, uint8_t value)
{
    send_(ANT_COMMAND_WRITE_REGISTER, address, value, true);
}

void AntBms::update()
{
    ESP_LOGD(TAG, "update() called");

    switch (m_ble_state)
    {
    case BleState::BLE_IDLE:
        if (NimBLEDevice::getInitialized())
        {
            ESP_LOGW(TAG, "BLE device initialized?!?");
            return;
        }

        ESP_LOGI(TAG, "Initializing BLE device");
        NimBLEDevice::init("");

        NimBLEDevice::setPower(ESP_PWR_LVL_P9);

        m_ble_scan = NimBLEDevice::getScan();

        m_ble_scan->setScanCallbacks(&m_on_scan_results, false);

        m_ble_scan->setInterval(100);
        m_ble_scan->setWindow(99);

        m_ble_scan->setActiveScan(true);
        m_ble_scan->start(0, false);

        m_ble_state = BleState::BLE_SCANNING;
        break;
    case BleState::BLE_SCANNING:
    {
        if (m_ble_devices.empty())
        {
            return;
        }

        auto &first_element = m_ble_devices[0];

        ble_connect(first_element->getAddress());
        break;
    }
    case BleState::BLE_CONNECTING:
        m_ble_state = BleState::BLE_SCANNING;
        break;
    case BleState::BLE_CONNECTED:
        if (espchrono::ago(m_last_update) > m_interval)
        {
            m_last_update = espchrono::millis_clock::now();
            send_(ANT_COMMAND_STATUS, 0x0000, 0xbe, false);
        }

        if (espchrono::ago(m_last_wireless_update) > m_wireless_interval)
        {
            m_last_wireless_update = espchrono::millis_clock::now();

            static bool flip = false;
            ESP_LOGI(TAG, "[APP] Free memory: %ld bytes", esp_get_free_heap_size());

            if (flip)
            {
                if (!espnow::send(espnow::broadcast_address, m_bmsData.toString()))
                {
                    ESP_LOGE(TAG, "Failed to send data over ESP-NOW");
                }
            }
            else
            {
                if (!espnow::send(espnow::broadcast_address, m_bmsData.toRareString()))
                {
                    ESP_LOGE(TAG, "Failed to send data over ESP-NOW");
                }
            }
            flip = !flip;
        }
        break;
    default:;
    }
}

AntBms::AntBms() : m_on_scan_results{*this}, m_on_client_events{*this}, m_characteristics_callbacks{*this}
{}

void AntBms::ble_connect(NimBLEAddress address)
{
    m_ble_state = BleState::BLE_CONNECTING;

    if (m_ble_client)
    {
        m_ble_client->disconnect();

        m_ble_client = nullptr;
    }

    m_ble_client = NimBLEDevice::createClient();

    m_ble_client->setClientCallbacks(&m_on_client_events, false);

    if (m_ble_client->connect(address))
    {
        m_ble_state = BLE_CONNECTED;

        ESP_LOGI(TAG, "Successfuly connected to %s", address.toString().c_str());

        // add notify callback to ANT_BMS_CHARACTERISTIC_UUID
        if (auto *service = m_ble_client->getService(ANT_BMS_SERVICE_UUID); service)
        {
            if (auto *characteristic = service->getCharacteristic(ANT_BMS_CHARACTERISTIC_UUID); characteristic)
            {
                if (characteristic->canNotify())
                {
                    if (characteristic->subscribe(true, [&](NimBLERemoteCharacteristic *pBLERemoteCharacteristic,
                                                            uint8_t *pData, size_t length, bool isNotify) {
                        m_notifyCallback(pBLERemoteCharacteristic, pData, length, isNotify);
                    }))
                    {
                        ESP_LOGI(TAG, "Subscribed to %s", characteristic->toString().c_str());
                        m_ble_characteristic = characteristic;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Failed to subscribe to %s", characteristic->toString().c_str());
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Characteristic %s does not support notify", characteristic->toString().c_str());
                }
            }
            else
            {
                ESP_LOGI(TAG, "Failed to get characteristic %s",
                         NimBLEUUID{ANT_BMS_CHARACTERISTIC_UUID}.toString().c_str());
            }
        }
        else
        {
            ESP_LOGI(TAG, "Failed to get service %s",
                     NimBLEUUID{ANT_BMS_SERVICE_UUID}.toString().c_str());
        }
    }
    else
    {
        ESP_LOGW(TAG, "Error connecting to %s", address.toString().c_str());
    }
}

void AntBms::m_notifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length,
                              bool isNotify)
{
    // ESP_LOGI(TAG, "Received %s: %s (%.*s)", isNotify ? "notification" : "indication", format_hex_pretty(pData, length).c_str(), length, pData);

    assemble(pData, length);
}

void AntBms::OnScanResults::onDiscovered(NimBLEAdvertisedDevice *advertised_device)
{
    // check if ANT_BMS_SERVICE_UUID
    if (!advertised_device->haveServiceUUID())
    {
        ESP_LOGW(TAG, "[onDiscovered] Found BLE device without service UUIDs: %s",
                 advertised_device->toString().c_str());
        return;
    }

    if (!advertised_device->isAdvertisingService(ANT_BMS_SERVICE_UUID))
    {
        ESP_LOGW(TAG, "[onDiscovered] Found BLE device without MATCHING service UUID: %s",
                 advertised_device->toString().c_str());
        return;
    }

    m_ant_bms.m_ble_devices.push_back(advertised_device);
}

void AntBms::OnClientCallback::onDisconnect(NimBLEClient *pClient, int reason)
{
    m_ant_bms.m_ble_state = BLE_SCANNING;
}

void AntBms::CharacteristicCallbacks::onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
                                                  uint16_t subValue)
{
    // send device info request
    ESP_LOGI(TAG, "Request device info frame");
    // 0x7e 0xa1 0x02 0x6c 0x02 0x20 0x58 0xc4 0xaa 0x55
    m_ant_bms.send_(ANT_COMMAND_DEVICE_INFO, 0x026c, 0x20, false);
}
} // namespace antbms
