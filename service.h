#ifndef _SERVICE_H_
#define _SERVICE_H_

#include <BLECharacteristic.h>

class MyowareBLEConnection
{
public:
  struct EventHandler
  {
    virtual void on_connected() {}
    virtual void on_disconnected() {}
    virtual void on_start_advertising() {}
    virtual void on_reset_calibrated_min_max() {}
    virtual void on_change_trigger_threshold(uint8_t threshold) {}
  };

  MyowareBLEConnection(const char *device_name, EventHandler *h = nullptr) : device_name(device_name),
                                                                             event_handler(h) {}

  void init();
  void tick();

  void set_event_listener(EventHandler *h) { event_handler = h; }

  void put_telemetry_values(uint8_t left, uint8_t right);
  void set_trigger_threshold(uint8_t threshold);

private:
  void handle_connection_state_changes();
  void handle_control_point_state_changes();

  const char *device_name = nullptr;
  EventHandler *event_handler = nullptr;
  bool is_connected = false;
  bool prev_is_connected = false;
  int prev_trigger_threshold = 0;
  bool prev_reset_calibrated_min_max = false;
  BLECharacteristic *chrHeartRateMeasurement = nullptr;
  BLECharacteristic *chrBodySensorLocation = nullptr;
  BLECharacteristic *chrHeartRateControlPoint = nullptr;
};

#endif
