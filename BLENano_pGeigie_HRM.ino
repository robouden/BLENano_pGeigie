/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <BLE_API.h>
#include <limits.h>

#define DEVICE_NAME       "pGeigie2007"
#define VERSION       "V0.01"

//sliding windows counting variables
    #define NX 60
    #define TIME_INTERVAL 1000
    #define IS_READY (interruptCounterAvailable())



//long for sliding window
    unsigned long shift_reg[NX] = {0};
    unsigned long reg_index = 0;
    unsigned long total_count = 0;
    unsigned long max_count = 0;
    unsigned long uptime = 0;
    unsigned long _start_time;
    unsigned long _delay=1000;
    unsigned long _count=0;
    unsigned long cpm1=0, cpb1=0;

// interruppt
const int interruptMode = FALLING;
const int updateIntervalInMinutes = 1;

//calculate CPM
unsigned long cpm_gen1()
      {
         unsigned int i;
         unsigned long c_p_m1 = 0;

         // sum up
         for (i=0 ; i < NX ; i++)
         c_p_m1 += shift_reg[i];
         //  deadtime compensation (medcom international)
//         c_p_m1 = (unsigned long)((float)c_p_m1/(1-(((float)c_p_m1*1.8833e-6))));
         return c_p_m1;
      }

// Sampling interval (e.g. 60,000ms = 1min)
    unsigned long updateIntervalInMillis = 0;

// The next time to feed
    unsigned long nextExecuteMillis = 0;

// Event flag signals when a geiger event has occurred
    volatile unsigned char eventFlag = 0;       // FIXME: Can we get rid of eventFlag and use counts>0?
    unsigned long int counts_per_sample;
    unsigned long int counts_per_sample2;

// The last connection time to disconnect from the server
// after uploaded feeds
    long lastConnectionTime = 0;

// The conversion coefficient from cpm to µSv/h
    float conversionCoefficient = 0;
    float conversionCoefficient2 = 0;

 

//Pulse counters
    void onPulse()
    {
        counts_per_sample++;  
    }

    //sliding windows setup
      void interruptCounterReset()
      {
        // set start time
        _start_time = millis();
        // set count to zero (optional)
        counts_per_sample = 0;
        counts_per_sample2 = 0;
      }
      
      int interruptCounterAvailable()
      {
        // get current time
        unsigned long now = millis();
        // do basic check for millis overflow
        if (now >= _start_time)
          return (now - _start_time >= _delay);
        else
          return (ULONG_MAX + now - _start_time >= _delay);
      }
      
      // return current number of counts
      unsigned long interruptCounterCount()
      {
        return counts_per_sample;
      }
      
      // return current number of counts
      unsigned long interruptCounterCount2()
      {
        return counts_per_sample2;
      }

//BLE inits
  BLE                       ble;
  Ticker                    ticker_task1;
  
  static uint8_t hrmCounter     = 100;
  static uint8_t bpm[2]         = {0x00, hrmCounter};
  static const uint8_t location = 0x03;
  static uint32_t cnt;
      
// Gatt stuff
      static const uint16_t uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE};
      GattCharacteristic   hrmRate(GattCharacteristic::UUID_HEART_RATE_MEASUREMENT_CHAR, bpm, sizeof(bpm), sizeof(bpm), GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
      GattCharacteristic   hrmLocation(GattCharacteristic::UUID_BODY_SENSOR_LOCATION_CHAR,(uint8_t *)&location, sizeof(location), sizeof(location),GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ);
      GattCharacteristic   *hrmChars[] = {&hrmRate, &hrmLocation, };
      GattService          hrmService(GattService::UUID_HEART_RATE_SERVICE, hrmChars, sizeof(hrmChars) / sizeof(GattCharacteristic *));
      

void disconnectionCallBack(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    Serial.println("Disconnected!");
    Serial.println("Restarting the advertising process");
    ble.startAdvertising();
}

void periodicCallback()
{
    if (ble.getGapState().connected)
    {
        /* Update the HRM measurement */
        /* First byte = 8-bit values, no extra info, Second byte = uint8_t HRM value */
        /* See --> https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml */
        hrmCounter++;
        if (hrmCounter == 175)
            hrmCounter = 100;
        Serial.println(hrmCounter);
        bpm[1] = hrmCounter;
        ble.updateCharacteristicValue(hrmRate.getValueAttribute().getHandle(), bpm, sizeof(bpm));
    }
}

void setup() {

    // put your setup code here, to run once
    Serial.begin(9600);

    pinMode(14, INPUT_PULLUP);
    attachInterrupt(14, onPulse, interruptMode);

    ticker_task1.attach(periodicCallback, 5);

    ble.init();
    ble.onDisconnection(disconnectionCallBack);

    // setup adv_data and srp_data
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t*)uuid16_list, sizeof(uuid16_list));
    ble.accumulateAdvertisingPayload(GapAdvertisingData::HEART_RATE_SENSOR_HEART_RATE_BELT);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));

    // set adv_type
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
      // add service
    ble.addService(hrmService);
    // set device name
    ble.setDeviceName((const uint8_t *)DEVICE_NAME);
    // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
    ble.setTxPower(4);
    // set adv_interval, 100ms in multiples of 0.625ms.
    ble.setAdvertisingInterval(160);
    // set adv_timeout, in seconds
    ble.setAdvertisingTimeout(0);
    // start advertising
    ble.startAdvertising();
}

void loop() {
    ble.waitForEvent();
}
