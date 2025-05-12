/*
 * Copyright (c) 2023 Particle Industries, Inc.
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

#include "Particle.h"
#include "edge.h"
#include "monitor_edge_ioexpansion.h"

#include "../lib/ModbusMaster/src/ModbusMaster.h"
#include "../lib/JsonParserGeneratorRK/src/JsonParserGeneratorRK.h"

#define NUM_SENSORS 4

const std::chrono::milliseconds logFrequency = 30s;
long lastLogTimestamp = 0;

const std::chrono::milliseconds sensiorFrequency = 5s;
long lastSensorTimestamp = 0;

// Modbus globals
int modbusErrorCode;

SYSTEM_MODE(SEMI_AUTOMATIC);

#if EDGE_PRODUCT_NEEDED
PRODUCT_ID(EDGE_PRODUCT_ID);
#endif // EDGE_PRODUCT_NEEDED
PRODUCT_VERSION(EDGE_PRODUCT_VERSION);

STARTUP(
    Edge::startup(););

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {
                                                         {"app.gps", LOG_LEVEL_WARN},
                                                         {"ncp", LOG_LEVEL_WARN},
                                                         {"net", LOG_LEVEL_WARN},
                                                         {"comm", LOG_LEVEL_WARN},
                                                         {"hal", LOG_LEVEL_WARN},
                                                         {"system.nm", LOG_LEVEL_WARN},
                                                     });

// port 1 address 1
ModbusMaster node(1, 1);
int registersVWC[NUM_SENSORS] = {3201, 3205, 3209, 3213};
int registersTemp[NUM_SENSORS] = {3203, 3207, 3211, 3215};

const char *labelsVWC[NUM_SENSORS] = {"VWC @15cm", "VWC @30cm", "VWC @45cm", "VWC @60cm"};
const char *labelsTemp[NUM_SENSORS] = {" *C @15cm", " *C @30cm", " *C @45cm", " *C @60cm"};
const char *labelsDEP[NUM_SENSORS] = {"DEP @15cm", "DEP @30cm", "DEP @45cm", "DEP @60cm"};
const char *labelsRAW[NUM_SENSORS] = {"RAW @15cm", "RAW @30cm", "RAW @45cm", "RAW @60cm"};

double outputVWC[NUM_SENSORS] = {0.0};
double outputTemp[NUM_SENSORS] = {0.0};
double outputDEP[NUM_SENSORS] = {0.0};
double outputRAW[NUM_SENSORS] = {0.0};

std::string decimalToBinary(int n)
{
    if (n == 0)
    {
        return "0";
    }
    std::string binary;
    while (n > 0)
    {
        binary += (n % 2 == 0 ? '0' : '1');
        n /= 2;
    }
    std::reverse(binary.begin(), binary.end());
    return binary;
}

void setup()
{
    Edge::instance().init();
    node.begin(9600);
    node.enableTXpin(MONITOREDGE_IOEX_RS485_DE_PIN);
    // node.enableTXpin(D4);
}

int readSensorValue(int regAddr, const char *label, double *output);

void accumulateSensor(int registers[], const char *labels[], double output[])
{
    int j = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        output[i] = 0;
        do
        {
            readSensorValue(registers[i], labels[i], &output[i]);
            j++;
        } while (modbusErrorCode == node.ku8MBInvalidSlaveID || modbusErrorCode == node.ku8MBResponseTimedOut);
    }
}

void accumulateVWC()
{
    accumulateSensor(registersVWC, labelsVWC, outputVWC);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        // VWC is a percentage; convert it
        outputVWC[i] *= 100;
    }
}

void accumulateTemp()
{
    accumulateSensor(registersTemp, labelsTemp, outputTemp);
}

void loop()
{
    Edge::instance().loop();

    if (millis() - lastLogTimestamp > logFrequency.count())
    {
        lastLogTimestamp = millis();
    }

    if (millis() - lastSensorTimestamp > sensiorFrequency.count())
    {
        Log.error("polling sensors");
        lastSensorTimestamp = millis();

        // Raw average value 500 to 1,500 mV
        // readSensorValue(3200, "RAW @15cm");
        // readSensorValue(3204, "RAW @30cm");
        // readSensorValue(3208, "RAW @45cm");
        // readSensorValue(3212, "RAW @60cm");

        // 0-100%
        // readSensorValue(3201, "VWC @15cm");
        // readSensorValue(3205, "VWC @30cm");
        // readSensorValue(3209, "VWC @45cm");
        // readSensorValue(3213, "VWC @60cm");

        // 0 to 80
        // readSensorValue(3202, "DEP @15cm");
        // readSensorValue(3206, "DEP @30cm");
        // readSensorValue(3210, "DEP @45cm");
        // readSensorValue(3214, "DEP @60cm");

        // -10 to +60 *C
        // readSensorValue(3203, " *C @15cm");
        // readSensorValue(3207, " *C @30cm");
        // readSensorValue(3211, " *C @45cm");
        // readSensorValue(3215, " *C @60cm");
        accumulateTemp();
        accumulateVWC();

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            Log.info("#%d %s output: %.2f %%", i, labelsVWC[i], outputVWC[i]);
        }
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            Log.info("#%d %s output: %.2f", i, labelsTemp[i], outputTemp[i]);
        }
    }
}

// Read two 16 bit values from regAddr and convert/store them to a 32 bit BE float output value
// returning the modbus modbusErrorCode if any (or -1)
int readSensorValue(int regAddr, const char *label, double *output)
{
    // https://publications.metergroup.com/Integrator%20Guide/18512%20TEROS%2054%20Integrator%20Guide.pdf
    // Register Types
    // 1XXXX    discrete output coils           read/write  on/off status or setup flags for the sensor
    // 2XXXX    discrete input contacts         read        sensor status flags
    // 3XXXX    analog input registers          read        numerical input variables from the sensor (actual sensor measurements)
    // 4XXXX    analog output holding registers read/write  numerical output variables for the sensors (params, setpoints, calibrations, etc.)

    // Register Offsets
    // X001-X009    read/write  16bit   signed integer
    // X101-X199    read/write  16bit   unsigned integer
    // X201-X299    read/write  32 bit  float (big endian)
    // X301-X399    read/write  32 bit  float (little endian)

    // Important Registers
    // 4100 slave addr

    // 3200 raw output @15cm
    // 3201 VWC @15cm
    // 3202 dielectric permittivity @15cm
    // 3203 temp @15cm

    // 3204 raw output @30cm
    // 3205 VWC @30cm
    // 3206 dielectric permittivity @30cm
    // 3207 temp @30cm

    // 3208 raw output @45cm
    // 3209 VWC @45cm
    // 3210 dielectric permittivity @45cm
    // 3211 temp @45cm

    // 3212 raw output @60cm
    // 3213 VWC @60cm
    // 3216 dielectric permittivity @60cm
    // 3217 temp @60cm

    // float waterData[8] = {0.0f};
    // float tempData[8] = {0.0f};

    // int registerAddr = 0xC80;

    // int vwcRegisters[] = {3201, 3205, 3209, 3213};
    // int tempCRegisters[] = {3200, 3204, 3218, 3212};
    // int tempCRegisters[] = {3203, 3207, 3211, 3215};

    // Log.error("%s reading sensor register: %d", label, regAddr);
    modbusErrorCode = -1;
    // Minimum 50ms delay between commands
    delay(50);
    modbusErrorCode = node.readInputRegisters(regAddr, 1);
    delay(50);

    uint16_t data[2];
    float value;

    if (modbusErrorCode == node.ku8MBSuccess)
    {

        data[0] = node.getResponseBuffer(0);
        // Minimum 100ms delay between queries
        delay(100);
        data[1] = node.getResponseBuffer(1);

        uint32_t raw_value = ((uint32_t)data[0] << 16) | data[1];
        memcpy(&value, &raw_value, sizeof(value));
        *output = value;
        Log.error("%s successful read register: %d data[0]: %d data[1]: %d, raw_value: %lu res: %.2f", label, regAddr, data[0], data[1], raw_value, *output);
    }
    else if (modbusErrorCode == node.ku8MBResponseTimedOut)
    {
        Log.error("%s modbus Error 0x%02x (Timeout)", label, modbusErrorCode);
    }
    else if (modbusErrorCode == node.ku8MBIllegalDataAddress)
    {
        Log.error("%s modbus Error 0x%02x (Illegal Address)", label, modbusErrorCode);
    }
    else if (modbusErrorCode == node.ku8MBInvalidSlaveID)
    {
        Log.error("%s modbus Error 0x%02x (Illegal Slave ID)", label, modbusErrorCode);
    }
    else
    {
        Log.error("%s modbus Error 0x%02x", label, modbusErrorCode);
    }

    return modbusErrorCode;
}