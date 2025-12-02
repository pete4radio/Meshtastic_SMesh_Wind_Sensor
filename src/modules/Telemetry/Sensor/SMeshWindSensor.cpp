#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include("DFRobotLarkSensor.h")

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "SMeshWindSensor.h"
#include "TelemetrySensor.h"
#include "gps/GeoCoord.h"
#include <string>

#ifdef ARCH_ESP32
#include "driver/pcnt.h"
#endif

// Static variable to track last time wind speed was calculated
static uint32_t lastWindSpeedMillis = 0;

SMeshWindSensor::SMeshWindSensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SMESH_WIND_VANE, "SMESH_WIND_VANE") {}

bool SMeshWindSensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    LOG_INFO("Init sensor: %s", sensorName);
    {
        LOG_DEBUG("SMESH_WIND_VANE Init Succeed");
        status = true;
    } 
    initI2CSensor();
    return status;
}

bool SMeshWindSensor::getMetrics(meshtastic_Telemetry *measurement)
{
    // Get wind direction as raw 12-bit value (0-4095) from AS5600
    uint16_t windDirection = lark.getValue("Dir").toInt();
    measurement->variant.environment_metrics.has_wind_direction = true;
    measurement->variant.environment_metrics.wind_direction = windDirection;
    LOG_INFO("Wind Direction: %u (raw AS5600 value)", windDirection);

    // Calculate delta millis since last wind speed reading
    uint32_t currentMillis = millis();
    uint32_t deltaMillis = currentMillis - lastWindSpeedMillis;

    // Only update wind speed if at least 1000ms have passed
    if (deltaMillis < 1000) {
        LOG_DEBUG("Wind speed update skipped (delta: %u ms)", deltaMillis);
        return true;
    }

    // Update the last millis timestamp
    lastWindSpeedMillis = currentMillis;

#ifdef ARCH_ESP32
    // Read the PCNT counter value
    int16_t counterValue = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &counterValue);

    // Log the raw counter value before clearing
    LOG_INFO("Raw counter value: %d", counterValue);

    // Clear the counter
    pcnt_counter_clear(PCNT_UNIT_0);

    // Calculate counts per second
    float countsPerSecond = ((float)counterValue / (float)deltaMillis) * 1000.0f;

    // Store wind speed in measurement
    measurement->variant.environment_metrics.has_wind_speed = true;
    measurement->variant.environment_metrics.wind_speed = countsPerSecond;

    // Log the calculated values
    LOG_INFO("Counts before clearing: %d, Counts/second: %.2f, Delta millis: %u",
             counterValue, countsPerSecond, deltaMillis);
#else
    // Fallback for non-ESP32 platforms
    measurement->variant.environment_metrics.has_wind_speed = true;
    measurement->variant.environment_metrics.wind_speed = 0.0f;
    LOG_WARN("PCNT not available on this platform");
#endif

    return true;
}

#endif