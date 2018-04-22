#include <Arduino.h>
#include <SensorFusion.h>
#include <protocol.h>
#include <protocol_debug.h>
#include <MPU9250.h>

MPU9250 IMU(SPI, 10);
int status;

SensorFusion sensorFusion;

#define X_AXIS A0
#define Y_AXIS A1
#define BUTTON 9

#define SerialBT Serial1


void setup()
{
    pinMode(X_AXIS, INPUT);
    pinMode(Y_AXIS, INPUT);
    pinMode(BUTTON, INPUT_PULLUP);

    SerialBT.begin(38400);
    while (!SerialBT)
    {
    }

    status = IMU.begin();
    if (status < 0)
    {
        while (1)
        {
            SerialBT.println("IMU initialization unsuccessful");
            SerialBT.println("Check IMU wiring or try cycling power");
            SerialBT.print("Status: ");
            SerialBT.println(status);
            delay(10000);
        }
    }

    float accelerometerRawValues[3] = {
        IMU.getAccelX_mss(),
        IMU.getAccelY_mss(),
        IMU.getAccelZ_mss()};

    sensorFusion.init(accelerometerRawValues);
}

static const unsigned int timeDelta = 1000U / FREQUENCY;

#define BUFFER_SIZE (128U)
uint8_t buffer[BUFFER_SIZE];
uint8_t bufferIndex = 0U;

void processIncomingPacket(const uint8_t *packet)
{
    const PacketHeader *header = (const PacketHeader *)packet;

    switch (header->payloadId)
    {
        case PAYLOAD_PING:
        {
            const PingPacket *pingPacket = (const PingPacket *)packet;

            PongPacket pongPacket = {0};
            pongPacket.payload.pong = pingPacket->payload.ping * pingPacket->payload.ping;

            setHeader((uint8_t *)&pongPacket, PAYLOAD_PONG);
            Serial.write((uint8_t *)&pongPacket, sizeof(PongPacket));
        }
        default:
            return;
    }
}


void loop()
{
    static uint32_t lastSensorSent = 0U;

    if (SerialBT.available() > 0)
    {
        uint8_t incomingByte = (uint8_t)SerialBT.read();

        processIncomingByte(buffer, &bufferIndex, BUFFER_SIZE, (uint32_t)millis(), incomingByte, processIncomingPacket);
    }

    //delay(10);

    uint32_t millisNow = millis();
    if (((millisNow % timeDelta) == 0U) && (millisNow != lastSensorSent))
    {
        lastSensorSent = millisNow;
        SensorPacket sensorPacket = {0};

        sensorPacket.payload.xJoypad = analogRead(X_AXIS);
    //    delay(10);
        sensorPacket.payload.yJoypad = analogRead(Y_AXIS);
        sensorPacket.payload.joypadButton = digitalRead(BUTTON) == 0;

        IMU.readSensor();

        sensorPacket.payload.accelerometerMSS[0] = IMU.getAccelX_mss();
        sensorPacket.payload.accelerometerMSS[1] = IMU.getAccelY_mss();
        sensorPacket.payload.accelerometerMSS[2] = IMU.getAccelZ_mss();
        sensorPacket.payload.gyroscopeRads[0] = IMU.getGyroX_rads();
        sensorPacket.payload.gyroscopeRads[1] = IMU.getGyroY_rads();
        sensorPacket.payload.gyroscopeRads[2] = IMU.getGyroZ_rads();
        sensorPacket.payload.magnetometerMicroT[0] = IMU.getMagX_uT();
        sensorPacket.payload.magnetometerMicroT[1] = IMU.getMagY_uT();
        sensorPacket.payload.magnetometerMicroT[2] = IMU.getMagZ_uT();
        sensorPacket.payload.temperatureCelcius = IMU.getTemperature_C();

        sensorFusion.apply(
                sensorPacket.payload.accelerometerMSS,
                sensorPacket.payload.gyroscopeRads,
                sensorPacket.payload.magnetometerMicroT,
                sensorPacket.payload.xAngleAfterFusion,
                sensorPacket.payload.yAngleAfterFusion,
                timeDelta
        );

        setHeader((uint8_t *)&sensorPacket, PAYLOAD_SENSOR);
/*
        Serial.print(sensorPacket.payload.xJoypad);
        Serial.print(" ");
        Serial.println(sensorPacket.payload.yJoypad);
        Serial.print(" ");
        Serial.println(sensorPacket.payload.joypadButton);
*/
        SerialBT.write((uint8_t *)&sensorPacket, sizeof(SensorPacket));
    }
}
