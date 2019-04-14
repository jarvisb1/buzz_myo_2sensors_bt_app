#ifndef _GATT_HEART_RATE_H_
#define _GATT_HEART_RATE_H_

#define SVC_HEART_RATE "180d" // org.bluetooth.service.heart_rate

#define CHR_HR_HEART_RATE_MEASUREMENT_UUID "2a37" // org.bluetooth.characteristic.heart_rate_measurement
#define CHR_HR_HEART_RATE_MEASUREMENT_PROPS BLECharacteristic::PROPERTY_NOTIFY

#define CHR_HR_BODY_SENSOR_LOCATION_UUID "2a38" // org.bluetooth.characteristic.body_sensor_location
#define CHR_HR_BODY_SENSOR_LOCATION_PROPS BLECharacteristic::PROPERTY_READ

#define CHR_HR_HEART_RATE_CONTROL_POINT_UUID "2a39" // org.bluetooth.characteristic.heart_rate_
#define CHR_HR_HEART_RATE_CONTROL_POINT_PROPS BLECharacteristic::PROPERTY_WRITE

#endif