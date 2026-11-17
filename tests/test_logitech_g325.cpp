#include "devices/logitech_g325_lightspeed.hpp"
#include "devices/hid_interface.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace headsetcontrol::testing {
namespace {

class TestFailure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

#define CHECK(condition, message) \
    do { \
        if (!(condition)) { \
            throw TestFailure(message); \
        } \
    } while (false)

class ScriptedHID : public HIDInterface {
public:
    std::vector<std::vector<uint8_t>> writes;
    std::deque<std::vector<uint8_t>> replies;

    Result<void> write(hid_device*, std::span<const uint8_t> data) override
    {
        writes.emplace_back(data.begin(), data.end());
        return {};
    }

    Result<void> write(hid_device*, std::span<const uint8_t> data, size_t size) override
    {
        std::vector<uint8_t> frame(size, 0);
        std::copy_n(data.begin(), std::min(data.size(), size), frame.begin());
        writes.push_back(std::move(frame));
        return {};
    }

    Result<size_t> readTimeout(hid_device*, std::span<uint8_t> data, int) override
    {
        if (replies.empty()) {
            return DeviceError::timeout("No scripted G325 response");
        }
        auto reply = std::move(replies.front());
        replies.pop_front();
        std::copy_n(reply.begin(), std::min(reply.size(), data.size()), data.begin());
        return std::min(reply.size(), data.size());
    }

    Result<void> sendFeatureReport(hid_device*, std::span<const uint8_t>) override { return {}; }
    Result<void> sendFeatureReport(hid_device*, std::span<const uint8_t>, size_t) override { return {}; }
    Result<size_t> getFeatureReport(hid_device*, std::span<uint8_t>) override { return size_t { 0 }; }
    Result<size_t> getInputReport(hid_device*, std::span<uint8_t>) override { return size_t { 0 }; }
};

class TestableG325 : public LogitechG325Lightspeed {
public:
    mutable ScriptedHID hid;

protected:
    HIDInterface& getHIDInterface() const override { return hid; }
};

std::vector<uint8_t> frame(std::initializer_list<uint8_t> payload)
{
    auto built = protocols::LogitechCenturionProtocol::buildCenturionFrame(
        std::span<const uint8_t>(payload.begin(), payload.size()), 0x00, LogitechG325Lightspeed::FRAME_PREFIX);
    return { built.begin(), built.end() };
}

void testBatteryPayloadParsing()
{
    auto available = LogitechG325Lightspeed::parseBatteryResponse(
        std::array<uint8_t, 10> { 0x03, 0x10, 0x00, 0x06, 0x00, 0x07, 0x0a, 90, 90, 0x00 });
    CHECK(available.hasValue(), "Valid discharging battery packet should parse");
    CHECK(available->level_percent == 90, "G325 percentage comes from the displayed percent byte");
    CHECK(available->status == BATTERY_AVAILABLE, "Charging state 0 means discharging");

    auto charging = LogitechG325Lightspeed::parseBatteryResponse(
        std::array<uint8_t, 10> { 0x03, 0x10, 0x00, 0x06, 0x00, 0x07, 0x0a, 89, 90, 0x02 });
    CHECK(charging.hasValue(), "Valid charging packet should parse");
    CHECK(charging->level_percent == 90, "Charging packet reports displayed percentage");
    CHECK(charging->status == BATTERY_CHARGING, "Charging state 2 maps to charging");

    CHECK(LogitechG325Lightspeed::parseBatteryResponse(
              std::array<uint8_t, 10> { 0x03, 0x10, 0x00, 0x06, 0x00, 0x07, 0x0a, 101, 101, 0x00 })
              .hasError(),
        "Out-of-range percentage must fail");
    CHECK(LogitechG325Lightspeed::parseBatteryResponse(
              std::array<uint8_t, 10> { 0x03, 0x10, 0x00, 0x06, 0x00, 0x07, 0x0a, 50, 50, 0x01 })
              .hasError(),
        "Unknown charging state must fail");
    CHECK(LogitechG325Lightspeed::parseBatteryResponse(
              std::array<uint8_t, 9> { 0x03, 0x10, 0x00, 0x06, 0x00, 0x07, 0x0a, 50, 50 })
              .hasError(),
        "Truncated battery packet must fail");
}

void testBatteryRequestAndResponseMatching()
{
    TestableG325 device;
    device.hid.replies.push_back(frame({ 0x03, 0x1a, 0x00, 0x00 })); // request ACK
    device.hid.replies.push_back(frame({ 0x03, 0x10, 0x00, 0x06, 0x00, 0x07, 0x0a, 0x5a, 0x5a, 0x02 }));

    auto battery = device.getBattery(nullptr);
    CHECK(battery.hasValue(), "G325 battery query should succeed");
    CHECK(battery->level_percent == 90, "Battery response percentage is decoded");
    CHECK(battery->status == BATTERY_CHARGING, "Battery response charging state is decoded");
    CHECK(device.hid.writes.size() == 1, "Exactly one read-only battery query is sent");

    const std::array<uint8_t, 11> expected { 0x50, 0x23, 0x0b, 0x00, 0x03, 0x1a, 0x00, 0x03, 0x00, 0x07, 0x0a };
    CHECK(std::equal(expected.begin(), expected.end(), device.hid.writes[0].begin()), "G325 request matches captured command");
    CHECK(std::all_of(device.hid.writes[0].begin() + expected.size(), device.hid.writes[0].end(),
              [](uint8_t byte) { return byte == 0; }),
        "Battery request is zero-padded");
    CHECK(device.hid.replies.empty(), "ACK and battery response are consumed");
}

} // namespace

void runAllLogitechG325Tests()
{
    testBatteryPayloadParsing();
    testBatteryRequestAndResponseMatching();
    std::cout << "[OK] Logitech G325 battery tests passed" << std::endl;
}

} // namespace headsetcontrol::testing
