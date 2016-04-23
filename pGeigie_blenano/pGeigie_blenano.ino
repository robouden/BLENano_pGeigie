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

#define DEVICE_NAME       "pGeigie"

BLE                       ble;
Ticker                    ticker_task1;

static uint16_t geigerCounter     = 0;
static uint16_t bpm = geigerCounter;
static const uint8_t location = 0x03;
static uint32_t cnt;

static const uint16_t uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE};

GattCharacteristic   hrmRate(GattCharacteristic::UUID_HEART_RATE_MEASUREMENT_CHAR, (uint8_t *)&bpm, sizeof(bpm), sizeof(bpm), GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

GattCharacteristic   hrmLocation(GattCharacteristic::UUID_BODY_SENSOR_LOCATION_CHAR,(uint8_t *)&location, sizeof(location), sizeof(location),GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ);

GattCharacteristic   *hrmChars[] = {&hrmRate, &hrmLocation, };

GattService          hrmService(GattService::UUID_HEART_RATE_SERVICE, hrmChars, sizeof(hrmChars) / sizeof(GattCharacteristic *));

void countInterrupt(void)
{
  geigerCounter += 1;
}

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

        bpm = geigerCounter;
        geigerCounter = 0;
        ble.updateCharacteristicValue(hrmRate.getValueAttribute().getHandle(), (uint8_t *)&bpm, sizeof(bpm));
    }
}

void setup() {

    // put your setup code here, to run once
    Serial.begin(9600);

    ticker_task1.attach(periodicCallback, 5);

    // setup the interrupt routine to count the pulses from the tube
    attachInterrupt(D4, countInterrupt, FALLING);

    ble.init();
    ble.onDisconnection(disconnectionCallBack);

    // setup adv_data and srp_data
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t*)uuid16_list, sizeof(uuid16_list));
    ble.accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_HEART_RATE_SENSOR);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));

    // set adv_type
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
      // add service
    ble.addService(hrmService);
    // set device name
    ble.setDeviceName((const uint8_t *)DEVICE_NAME);
    // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
    ble.setTxPower(-20);
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
