#include "service.h"

#include <Arduino.h>
#include <BLEService.h>
#include <BLEDevice.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

#include "config.h"
#include "gatt_heart_rate.h"

#define SVC_UUID SVC_HEART_RATE

static BLECharacteristic *createCharacteristic(BLEService *svc, BLEUUID service_uuid, uint32_t props)
{
  BLECharacteristic *chr = svc->createCharacteristic(service_uuid, props);
  chr->addDescriptor(new BLE2902());
  return chr;
}

struct DeviceConnectionCallbacks : public BLEServerCallbacks
{
  DeviceConnectionCallbacks(bool *is_connected) : is_connected(is_connected) {}

  void onConnect(BLEServer *)
  {
    *is_connected = true;
  }
  void onDisconnect(BLEServer *)
  {
    *is_connected = false;
  }

  bool *is_connected;
};

void MyowareBLEConnection::init()
{
  BLEDevice::init(this->device_name);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new DeviceConnectionCallbacks(&this->is_connected));
  BLEService *svc = server->createService(BLEUUID(SVC_UUID));
  chrHeartRateMeasurement = createCharacteristic(svc,
                                                 BLEUUID(CHR_HR_HEART_RATE_MEASUREMENT_UUID),
                                                 CHR_HR_HEART_RATE_MEASUREMENT_PROPS);
  chrBodySensorLocation = createCharacteristic(svc,
                                               BLEUUID(CHR_HR_BODY_SENSOR_LOCATION_UUID),
                                               CHR_HR_BODY_SENSOR_LOCATION_PROPS);
  chrHeartRateControlPoint = createCharacteristic(svc,
                                                  BLEUUID(CHR_HR_HEART_RATE_CONTROL_POINT_UUID),
                                                  CHR_HR_HEART_RATE_CONTROL_POINT_PROPS);
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID(SVC_UUID));
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06); // functions that help with iPhone connections issue
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  if (event_handler)
  {
    event_handler->on_start_advertising();
  }
}

void MyowareBLEConnection::tick()
{
  handle_connection_state_changes();
  if (is_connected)
  {
    handle_control_point_state_changes();
  }
}

struct __attribute__((packed)) HRMeasurement
{
  enum class Flags : uint8_t
  {
    VALUE_FORMAT_UINT8 = 0,
    VALUE_FORMAT_UINT16 = 0b00000001,
    SENSOR_CONTACT_UNSUPPORTED = 0,
    SENSOR_CONTACT_NOT_DETECTED = 0b00000100,
    SENSOR_CONTACT_DETECTED = 0b00000110,
    ENERGY_EXPENDED_UNSUPPORTED = 0,
    ENERGY_EXPENDED_SUPPORTED = 0b00001000,
    RR_INTERVAL_UNSUPPORTED = 0,
    RR_INTERVAL_SUPPORTED = 0b00010000
  } flags;
  union Data {
    uint8_t u8;
    uint16_t u16;
  } data;

  uint8_t *raw() const { return (uint8_t *)this; }
};

void MyowareBLEConnection::put_telemetry_values(uint8_t left, uint8_t right)
{
  HRMeasurement m;
  m.flags = HRMeasurement::Flags::VALUE_FORMAT_UINT16;
  m.data.u16 = (left << 8) | right;
  chrHeartRateMeasurement->setValue(m.raw(), sizeof(m));
  chrHeartRateMeasurement->notify();
}

void MyowareBLEConnection::set_trigger_threshold(uint8_t threshold)
{
  chrBodySensorLocation->setValue(&threshold, sizeof(threshold));
}

void MyowareBLEConnection::handle_connection_state_changes()
{
  if (is_connected && !prev_is_connected)
  {
    // disconnected -> connected
    if (event_handler)
    {
      event_handler->on_connected();
    }
  }
  else if (!is_connected && prev_is_connected)
  {
    // connected -> disconnected
    if (event_handler)
    {
      event_handler->on_disconnected();
    }
  }
  prev_is_connected = is_connected;
}

void MyowareBLEConnection::handle_control_point_state_changes()
{
  std::string raw_data = chrHeartRateControlPoint->getValue();
  if (raw_data.size() <= 0)
  {
    return;
  }
  int trigger_threshold = *raw_data.c_str();
  if (trigger_threshold != prev_trigger_threshold)
  {
    prev_trigger_threshold = trigger_threshold;
    if (event_handler)
    {
      event_handler->on_change_trigger_threshold(trigger_threshold);
    }
  }
}
