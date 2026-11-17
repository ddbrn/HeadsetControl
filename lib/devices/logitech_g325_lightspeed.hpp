#pragma once

#include "protocols/logitech_centurion_protocol.hpp"
#include <array>
#include <chrono>
#include <span>
#include <string_view>

using namespace std::string_view_literals;

namespace headsetcontrol {

/**
 * @brief Logitech G325 LIGHTSPEED wireless gaming headset.
 *
 * Protocol evidence: https://github.com/Sapd/HeadsetControl/issues/581
 * Battery-only implementation. The G325 uses the 0x50 0x23 Centurion frame
 * prefix, but its direct battery packet layout differs from the G522.
 */
class LogitechG325Lightspeed : public protocols::LogitechCenturionProtocol {
public:
    static constexpr std::array<uint16_t, 1> SUPPORTED_PRODUCT_IDS { 0x0b21 };
    static constexpr std::array<uint8_t, 2> FRAME_PREFIX { 0x50, 0x23 };

    constexpr uint16_t getVendorId() const override
    {
        return VENDOR_LOGITECH;
    }

    std::vector<uint16_t> getProductIds() const override
    {
        return { SUPPORTED_PRODUCT_IDS.begin(), SUPPORTED_PRODUCT_IDS.end() };
    }

    std::string_view getDeviceName() const override
    {
        return "Logitech G325 LIGHTSPEED"sv;
    }

    constexpr int getCapabilities() const override
    {
        return B(CAP_BATTERY_STATUS);
    }

    constexpr capability_detail getCapabilityDetail(enum capabilities cap) const override
    {
        switch (cap) {
        case CAP_BATTERY_STATUS:
            return { .usagepage = 0xffa0, .usageid = 0x0001, .interface_id = 3 };
        default:
            return HIDDevice::getCapabilityDetail(cap);
        }
    }

    Result<BatteryResult> getBattery(hid_device* device_handle) override
    {
        auto start_time = std::chrono::steady_clock::now();

        // Captured G325 request: 50 23 0b 00 03 1a 00 03 00 07 0a 00 ...
        std::array<uint8_t, 64> request {};
        request[0]  = 0x50;
        request[1]  = 0x23;
        request[2]  = 0x0b;
        request[3]  = 0x00;
        request[4]  = 0x03;
        request[5]  = 0x1a;
        request[6]  = 0x00;
        request[7]  = 0x03;
        request[8]  = 0x00;
        request[9]  = 0x07;
        request[10] = 0x0a;

        if (auto write_result = writeHID(device_handle, request, request.size()); !write_result) {
            return write_result.error();
        }

        // The device acknowledges, then sends the status frame. Skip ACKs and
        // unrelated frames while accepting only the G325 battery response.
        for (int attempt = 0; attempt < POLL_ATTEMPTS; ++attempt) {
            std::array<uint8_t, 64> response {};
            auto read_result = readHIDTimeout(device_handle, response, hsc_device_timeout);
            if (!read_result) {
                return read_result.error();
            }

            if (*read_result == 0) {
                continue;
            }

            auto payload_result = extractCenturionPayload(response, FRAME_PREFIX);
            if (!payload_result) {
                continue;
            }

            const auto& payload = *payload_result;
            if (payload.size() >= 13
                && payload[0] == 0x0b
                && payload[1] == 0x10
                && payload[2] == 0x00
                && payload[3] == 0x06
                && payload[4] == 0x00
                && payload[5] == 0x07
                && payload[6] == 0x0a) {
                auto battery = parseBatteryPayload(payload.subspan(7));
                if (!battery) {
                    return battery.error();
                }

                auto end_time = std::chrono::steady_clock::now();
                battery->raw_data = std::vector<uint8_t>(response.begin(), response.end());
                battery->query_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                return *battery;
            }
        }

        return DeviceError::timeout("Timed out waiting for Logitech G325 battery response");
    }

    static Result<BatteryResult> parseBatteryPayload(std::span<const uint8_t> payload)
    {
        // Captured layout: <raw> <display percentage> <charging state>.
        if (payload.size() < 3) {
            return DeviceError::protocolError("Unexpected Logitech G325 battery response");
        }

        const uint8_t level = payload[1];
        if (level > 100) {
            return DeviceError::protocolError("G325 battery percentage out of range");
        }

        BatteryStatus status;
        switch (payload[2]) {
        case 0x00:
            status = BATTERY_AVAILABLE;
            break;
        case 0x02:
            status = BATTERY_CHARGING;
            break;
        default:
            return DeviceError::protocolError("Unexpected Logitech G325 charging state");
        }

        return BatteryResult {
            .level_percent = static_cast<int>(level),
            .status = status,
        };
    }

protected:
    std::span<const uint8_t> centurionFramePrefix() const override
    {
        return FRAME_PREFIX;
    }

    protocols::CenturionOptions centurionOptions() const override
    {
        return {
            .exact_direct_length = true,
            .detect_connection_events = true,
            .probe_offline_on_timeout = false,
            .lookup_features_by_id = false,
        };
    }
};

} // namespace headsetcontrol
