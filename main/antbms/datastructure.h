#pragma once

// system includes
#include <vector>
#include <expected>
#include <cmath>

// 3rdparty includes
#include <fmt/format.h>
#include <ArduinoJson.h>

namespace antbms {
template<typename T>
std::string vecToString(std::string prefix, const std::vector<T> &vec)
{
    std::string result;
    for (uint8_t i = 0; i < vec.size(); i++)
    {
        result += fmt::format("{} {}: {} \n", prefix, i + 1, vec[i]);
    }
    return result;
}

// Battery status (0: Unknown, 1: Idle, 2: Charge, 3: Discharge, 4: Standby, 5: Error)
enum class BatteryStatus
{
    Unknown = 0,
    Idle,
    Charge,
    Discharge,
    Standby,
    Error
};

enum class ChargeMosfetStatus
{
    Off = 0x00,
    On = 0x01,
    OverchargeProtection = 0x02,
    OverCurrentProtection = 0x03,
    BatteryFull = 0x04,
    TotalOverpressure = 0x05,
    BatteryOverTemperature = 0x06,
    MosfetOverTemperature = 0x07,
    AbnormalCurrent = 0x08,
    BalancedLineDroppedString = 0x09,
    MotherboardOverTemperature = 0x0A,
    Unknown0x0B = 0x0B,
    Unknown0x0C = 0x0C,
    DischargeMosfetAbnormality = 0x0D,
    Unknown0x0E = 0x0E,
    ManuallyTurnedOff = 0x0F,
};

enum class DischargeMosfetStatus
{
    Off = 0x00,
    On = 0x01,
    OverdischargeProtection = 0x02,
    OverCurrentProtection = 0x03,
    Unknown0x04 = 0x04,
    TotalPressureUndervoltage = 0x05,
    BatteryOverTemperature = 0x06,
    MosfetOverTemperature = 0x07,
    AbnormalCurrent = 0x08,
    BalancedLineDroppedString = 0x09,
    MotherboardOverTemperature = 0x0A,
    ChargeMosfetOn = 0x0B,
    ShortCircuitProtection = 0x0C,
    DischargeMosfetAbnormality = 0x0D,
    StartException = 0x0E,
    ManuallyTurnedOff = 0x0F,
};

enum class BalancerStatus
{
    Off = 0x00,
    ExceedsTheLimitEquilibrium = 0x01,
    ChargeDifferentialPressureBalance = 0x02,
    BalancedOverTemperature = 0x03,
    AutomaticEqualization = 0x04,
    Unknown0x05 = 0x05,
    Unknown0x06 = 0x06,
    Unknown0x07 = 0x07,
    Unknown0x08 = 0x08,
    Unknown0x09 = 0x09,
    MotherboardOverTemperature = 0x0A,
};

struct AntBmsData
{
    BatteryStatus battery_status;
    float power;
    float mosfet_temperature;
    float balancer_temperature;
    float total_voltage;
    float current;
    float state_of_charge;
    float state_of_health;
    float total_battery_capacity_setting;
    float capacity_remaining;
    float battery_cycle_capacity;
    uint32_t total_runtime;
    std::string total_runtime_formatted;
    std::vector<float> cell_voltages;
    std::vector<float> temperatures;
    uint32_t balanced_cell_bitmask;
    float max_cell_voltage;
    float max_voltage_cell;
    float min_cell_voltage;
    float min_voltage_cell;
    float delta_cell_voltage;
    float average_cell_voltage;
    float accumulated_discharging_capacity;
    float accumulated_charging_capacity;
    float accumulated_discharging_time;
    std::string accumulated_discharging_time_formatted;
    float accumulated_charging_time;
    std::string accumulated_charging_time_formatted;

    ChargeMosfetStatus charge_mosfet_status;
    std::string charge_mosfet_status_string;
    DischargeMosfetStatus discharge_mosfet_status;
    std::string discharge_mosfet_status_string;
    BalancerStatus balancer_status;
    std::string balancer_status_string;

    std::string hardware_version;
    std::string software_version;

    [[nodiscard]] std::string toString() const
    {
        ArduinoJson::StaticJsonDocument<1024> doc;
        auto result = toJSON(doc);
        if (!result)
        {
            return result.error();
        }
        std::string json;
        serializeJson(doc, json);
        return fmt::format("BMS:{}", json);
    }

    [[nodiscard]] std::string toRareString() const
    {
        static uint8_t counter = 0;
        ArduinoJson::StaticJsonDocument<1024> doc;
        auto result = toRareJSON(doc, counter);
        if (!result)
        {
            return result.error();
        }
        std::string json;
        serializeJson(doc, json);
        return fmt::format("BMS:{}", json);
    }

    std::expected<void, std::string> toJSON(JsonDocument &doc) const
    {
        doc.clear();

        // 3-letter-keys
        doc["pwr"] = power;
        doc["tvo"] = total_voltage;
        doc["cur"] = current;
        doc["soc"] = state_of_charge;
        doc["cre"] = capacity_remaining;
        doc["bst"] = static_cast<uint8_t>(battery_status);
        doc["cms"] = static_cast<uint8_t>(charge_mosfet_status);
        doc["dms"] = static_cast<uint8_t>(discharge_mosfet_status);
        doc["bst"] = static_cast<uint8_t>(balancer_status);
        doc["dcv"] = delta_cell_voltage;
        doc["mcv"] = max_cell_voltage;
        doc["miv"] = min_cell_voltage;

        return {};
    }

    std::expected<void, std::string> toRareJSON(JsonDocument &doc, uint8_t &counter) const
    {
        doc.clear();

        // 3-letter-keys
        if (counter == 0)
        {
            doc["bte"] = balancer_temperature;
            doc["mot"] = mosfet_temperature;
            doc["soh"] = state_of_health;
            counter++;
        }
        else if (counter == 1)
        {
            doc["tbc"] = total_battery_capacity_setting;
            doc["bcc"] = battery_cycle_capacity;
            doc["trt"] = total_runtime;
            counter++;
        }
        else if (counter == 2)
        {
            doc["bcb"] = balanced_cell_bitmask;
            doc["mvc"] = max_voltage_cell;
            doc["mic"] = min_voltage_cell;
            counter++;
        }
        else if (counter == 3)
        {
            doc["acc"] = accumulated_charging_capacity;
            doc["adt"] = accumulated_discharging_time;
            doc["act"] = accumulated_charging_time;
            counter++;
        }
        else if (counter == 4)
        {
            doc["acv"] = average_cell_voltage;
            doc["adc"] = accumulated_discharging_capacity;
            doc["css"] = charge_mosfet_status_string.c_str();
            counter++;
        }
        else if (counter == 5)
        {
            doc["dss"] = discharge_mosfet_status_string.c_str();
            doc["bss"] = balancer_status_string.c_str();
            doc["dtf"] = accumulated_discharging_time_formatted.c_str();
            counter++;
        }
        else if (counter == 6)
        {
            doc["ctf"] = accumulated_charging_time_formatted.c_str();
            doc["hrd"] = hardware_version.c_str();
            doc["sft"] = software_version.c_str();
            doc["trf"] = total_runtime_formatted.c_str();
            counter++;
        }
        else if (counter == 7)
        {
            auto cell_voltages_json = doc.createNestedArray("vol");
            for (const auto &cell_voltage: cell_voltages)
            {
                cell_voltages_json.add(cell_voltage);
            }
            counter++;
        }
        else
        {
            auto temperatures_json = doc.createNestedArray("tmp");
            for (const auto &temperature: temperatures)
            {
                temperatures_json.add(temperature);
            }

            counter = 0;
        }

        return {};
    }

    void parseDoc(const JsonDocument &doc)
    {
        if (doc.containsKey("pwr"))
        {
            power = doc["pwr"].as<float>();
        }

        if (doc.containsKey("tvo"))
        {
            total_voltage = doc["tvo"].as<float>();
        }

        if (doc.containsKey("cur"))
        {
            current = doc["cur"].as<float>();
        }

        if (doc.containsKey("soc"))
        {
            state_of_charge = doc["soc"].as<float>();
        }

        if (doc.containsKey("cre"))
        {
            capacity_remaining = doc["cre"].as<float>();
        }

        if (doc.containsKey("bst"))
        {
            battery_status = static_cast<BatteryStatus>(doc["bst"].as<uint8_t>());
        }

        if (doc.containsKey("cms"))
        {
            charge_mosfet_status = static_cast<ChargeMosfetStatus>(doc["cms"].as<uint8_t>());
        }

        if (doc.containsKey("dms"))
        {
            discharge_mosfet_status = static_cast<DischargeMosfetStatus>(doc["dms"].as<uint8_t>());
        }

        if (doc.containsKey("bst"))
        {
            balancer_status = static_cast<BalancerStatus>(doc["bst"].as<uint8_t>());
        }

        if (doc.containsKey("dcv"))
        {
            delta_cell_voltage = doc["dcv"].as<float>();
        }

        if (doc.containsKey("mcv"))
        {
            max_cell_voltage = doc["mcv"].as<float>();
        }

        if (doc.containsKey("miv"))
        {
            min_cell_voltage = doc["miv"].as<float>();
        }

        if (doc.containsKey("bte"))
        {
            balancer_temperature = doc["bte"].as<float>();
        }

        if (doc.containsKey("mot"))
        {
            mosfet_temperature = doc["mot"].as<float>();
        }

        if (doc.containsKey("soh"))
        {
            state_of_health = doc["soh"].as<float>();
        }

        if (doc.containsKey("tbc"))
        {
            total_battery_capacity_setting = doc["tbc"].as<float>();
        }

        if (doc.containsKey("bcc"))
        {
            battery_cycle_capacity = doc["bcc"].as<float>();
        }

        if (doc.containsKey("trt"))
        {
            total_runtime = doc["trt"].as<uint32_t>();
        }

        if (doc.containsKey("bcb"))
        {
            balanced_cell_bitmask = doc["bcb"].as<uint16_t>();
        }

        if (doc.containsKey("mvc"))
        {
            max_voltage_cell = doc["mvc"].as<uint8_t>();
        }

        if (doc.containsKey("mic"))
        {
            min_voltage_cell = doc["mic"].as<uint8_t>();
        }

        if (doc.containsKey("acc"))
        {
            accumulated_charging_capacity = doc["acc"].as<float>();
        }

        if (doc.containsKey("adt"))
        {
            accumulated_discharging_time = doc["adt"].as<float>();
        }

        if (doc.containsKey("act"))
        {
            accumulated_charging_time = doc["act"].as<float>();
        }

        if (doc.containsKey("acv"))
        {
            average_cell_voltage = doc["acv"].as<float>();
        }

        if (doc.containsKey("adc"))
        {
            accumulated_discharging_capacity = doc["adc"].as<float>();
        }

        if (doc.containsKey("css"))
        {
            charge_mosfet_status_string = doc["css"].as<std::string>();
        }

        if (doc.containsKey("dss"))
        {
            discharge_mosfet_status_string = doc["dss"].as<std::string>();
        }

        if (doc.containsKey("bss"))
        {
            balancer_status_string = doc["bss"].as<std::string>();
        }

        if (doc.containsKey("dtf"))
        {
            accumulated_discharging_time_formatted = doc["dtf"].as<std::string>();
        }

        if (doc.containsKey("ctf"))
        {
            accumulated_charging_time_formatted = doc["ctf"].as<std::string>();
        }

        if (doc.containsKey("hrd"))
        {
            hardware_version = doc["hrd"].as<std::string>();
        }

        if (doc.containsKey("sft"))
        {
            software_version = doc["sft"].as<std::string>();
        }

        if (doc.containsKey("trf"))
        {
            total_runtime_formatted = doc["trf"].as<std::string>();
        }

        if (doc.containsKey("vol"))
        {
            auto cell_voltages_json = doc["vol"].as<JsonArrayConst>();
            for (const auto &cell_voltage_json: cell_voltages_json)
            {
                cell_voltages.push_back(cell_voltage_json.as<float>());
            }
        }

        if (doc.containsKey("tmp"))
        {
            auto temperatures_json = doc["tmp"].as<JsonArrayConst>();
            for (const auto &temperature_json: temperatures_json)
            {
                temperatures.push_back(temperature_json.as<float>());
            }
        }
    }
};
} // namespace antbms
