#pragma once

#ifndef _MT_SMESHWINDSENSOR_H
#define _MT_SMESHWINDSENSOR_H
#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include("DFRobotLarkSensor.h")

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include "DFRobotLarkSensor.h"
#include <string>

class SMeshWindSensor : public TelemetrySensor
{
  private:
    DFRobotLarkSensor_I2C lark = DFRobotLarkSensor_I2C();

  public:
    SMeshWindSensor();
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;
};

#endif
#endif