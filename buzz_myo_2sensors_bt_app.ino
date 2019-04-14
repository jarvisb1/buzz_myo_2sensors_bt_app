#include <Wire.h>
#include "Adafruit_DRV2605.h"

#include "config.h"
#include "service.h"

MyowareBLEConnection *conn;

Adafruit_DRV2605 drv;

#define EFFECT_SHORT 14 // short
#define EFFECT_LONG 16 // long

#define SHORT_MIN (-32768)
#define SHORT_MAX 32767

#define SAMPLE_RATE_HZ 100
const int sample_delay_ms = 1000 / SAMPLE_RATE_HZ;

//Filter parameters
#define NZEROS 3
#define NPOLES 3
#define GAIN 3.430944333e+04

#define DEFAULT_HAPTIC_THRESHOLD 300
#define HAPTIC_SCALE 1024

#define LONG_BUZZ_PERIOD_MS 3000
#define SHORT_BUZZ_PERIOD_MS 5000

#define NUM_MOTORS 4
uint8_t motor_mux_ports[NUM_MOTORS] = {2, 3, 4, 5};

#define NUM_LONG_BUZZ_MOTORS 2
#define NUM_SHORT_BUZZ_MOTORS 2
uint8_t long_buzz_ports[NUM_LONG_BUZZ_MOTORS] = {2, 3};
uint8_t short_buzz_ports[NUM_SHORT_BUZZ_MOTORS] = {4, 5};

struct sensor_motor_group {
  float xv[NZEROS + 1];
  float yv[NPOLES + 1];
  float norm_filter_out;
  short filter_out;
  short max_val;
  short min_val;
  short threshold;
  uint8_t pin;
  uint8_t motor_port;
};

sensor_motor_group left_group;
sensor_motor_group right_group;

//////////////////
//BT setup
//////////////////
struct ConnectionHandler : public MyowareBLEConnection::EventHandler
{
  void on_change_trigger_threshold(uint8_t threshold) override
  {
    DEBUG_PRINTLN("Received calibration threshold update");
    DEBUG_PRINTLN(threshold);
  }

  void on_reset_calibrated_min_max()
  {
    DEBUG_PRINTLN("Received reset calibration command");
  }

  void on_connected() override
  {
    DEBUG_PRINTLN("Connected :)");
  }

  void on_disconnected() override
  {
    DEBUG_PRINTLN("Disconnected :(");
  }

  void on_start_advertising() override
  {
    DEBUG_PRINTLN("Advertising :o");
  }
};

void setup_bt()
{
  // Create the BT device
  conn = new MyowareBLEConnection("E2M BT", new ConnectionHandler());
  conn->init();
}
//////////////////End BT Setup

//This helper function tells the I2C mux to switch to a particular output. Valid inputs: 0-7
#define TCAADDR 0x70
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  digitalWrite(13, HIGH);
  
  Serial.begin(115200);
  Serial.println();

  //Setup I2C mux
  Wire.begin();
  for (uint8_t i = 0; i < NUM_MOTORS; i++)
  {
    tcaselect(motor_mux_ports[i]);
    drv.begin();
    drv.selectLibrary(1);
  
    // I2C trigger by sending 'go' command 
    // default, internal trigger when sending GO command
    drv.setMode(DRV2605_MODE_INTTRIG); 
  }

  left_group.pin = A2;
  left_group.motor_port = 2;
  left_group.filter_out = 0;
  left_group.threshold = DEFAULT_HAPTIC_THRESHOLD;
  left_group.max_val = SHORT_MIN;
  left_group.min_val = SHORT_MAX;
  
  right_group.pin = A3;
  right_group.motor_port = 3;
  right_group.filter_out = 0;
  right_group.threshold = DEFAULT_HAPTIC_THRESHOLD;
  right_group.max_val = SHORT_MIN;
  right_group.min_val = SHORT_MAX;

  setup_bt(); //This (maybe?) blocks until a connection occurs
}

void do_buzz(uint8_t mux_port, uint8_t effect, uint8_t repeat = 0)
{
  tcaselect(mux_port);

  drv.setWaveform(0, effect);  // play effect 
  uint8_t repeat_i = 0;
  for (repeat_i = 0; repeat_i < repeat; repeat_i++)
    drv.setWaveform(repeat_i+1, effect);

  drv.setWaveform(repeat_i+1, 0);       // end waveform

  // play the effect!
  drv.go();  
}

void do_triple_buzz(uint8_t mux_port)
{
  for (uint8_t i = 0; i < 3; i++)
  {  
    do_buzz(mux_port, 14);
    delay(250);
  }
}

/*
void send_message()
{
  send_str = "\nL: ";
  if (left_group.filter_out > left_group.threshold)
    send_str += "CONTRACTED :(";
  else
    send_str += "relaxed :)";

  send_str += "  R: ";
  if (right_group.filter_out > right_group.threshold)
    send_str += "CONTRACTED :(";
  else
    send_str += "relaxed :)";

  send_chararray = (unsigned char *)const_cast<char*>(send_str.c_str());
  pTxCharacteristic->setValue(send_chararray, send_str.length());
  pTxCharacteristic->notify();
}
*/

unsigned long last_long_buzz_millis = 0;
void possibly_do_buzz_long()
{
  unsigned long curr_millis = millis();
  if ((curr_millis - last_long_buzz_millis) > LONG_BUZZ_PERIOD_MS)
  {
    last_long_buzz_millis = curr_millis;
    
    if (left_group.filter_out > left_group.threshold)
      do_buzz(left_group.motor_port, EFFECT_LONG);

    if (right_group.filter_out > right_group.threshold)
      do_buzz(right_group.motor_port, EFFECT_LONG);

    //send_message();
  }
}

unsigned long last_short_buzz_millis = 0;
void possibly_do_buzz_short()
{
  unsigned long curr_millis = millis();
  if ((curr_millis - last_short_buzz_millis) > SHORT_BUZZ_PERIOD_MS)
  {
    last_short_buzz_millis = curr_millis;
    for (uint8_t i = 0; i < NUM_SHORT_BUZZ_MOTORS; i++)
      do_buzz(short_buzz_ports[i], EFFECT_SHORT);
  }
}

void drive_motors()
{
  possibly_do_buzz_short();
  possibly_do_buzz_long();
}

float filter(float value, sensor_motor_group *g)
{
  g->xv[0] = g->xv[1];
  g->xv[1] = g->xv[2];
  g->xv[2] = g->xv[3];
  g->xv[3] = value / GAIN;
  g->yv[0] = g->yv[1];
  g->yv[1] = g->yv[2];
  g->yv[2] = g->yv[3];
  g->yv[3] = (g->xv[0] + g->xv[3]) + 3 * (g->xv[1] + g->xv[2]) + (0.8818931306 * g->yv[0]) + (-2.7564831952 * g->yv[1]) + (2.8743568927 * g->yv[2]);
  return g->yv[3];
}

void update_bounds_and_normalize()
{
  if (left_group.filter_out > left_group.max_val)
    left_group.max_val = left_group.filter_out;
  if (left_group.filter_out < left_group.min_val)
    left_group.min_val = left_group.filter_out;
  left_group.norm_filter_out = (float)(left_group.filter_out - left_group.min_val) / (left_group.max_val - left_group.min_val);

  if (right_group.filter_out > right_group.max_val)
    right_group.max_val = right_group.filter_out;
  if (right_group.filter_out < right_group.min_val)
    right_group.min_val = right_group.filter_out;
  right_group.norm_filter_out = (float)(right_group.filter_out - right_group.min_val) / (right_group.max_val - right_group.min_val);
}

void loop()
{
    // compute sensor readings: left side
    int raw_l = analogRead(left_group.pin);
    int raw_r = analogRead(right_group.pin);
    
    //short filtered_l = filter_l(raw_l);
    //short filtered_r = filter_r(raw_r);
    short filtered_l = filter(raw_l, &left_group);
    short filtered_r = filter(raw_r, &right_group);
    left_group.filter_out = filtered_l;
    right_group.filter_out = filtered_r;

    update_bounds_and_normalize();
    
    drive_motors();
    
    conn->put_telemetry_values((uint8_t)(left_group.norm_filter_out*128), (uint8_t)(right_group.norm_filter_out*128));
    conn->tick();
    
    Serial.printf("Left: %d %d | Right: %d %d\n", raw_l, filtered_l, raw_r, filtered_r);
    delay(sample_delay_ms); // bluetooth stack will go into congestion if too many packets are sent, so this should be at least 10ms
}
