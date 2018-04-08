#include <stdio.h>
#include <protocol.h>
#include <protocol_debug.h>


const char *getPayloadLabel(uint8_t payloadId)
{
    switch (payloadId)
    {
        case PAYLOAD_PING:
            return "PAYLOAD_PING";
        case PAYLOAD_PONG:
            return "PAYLOAD_PONG";
        case PAYLOAD_SENSOR:
            return "PAYLOAD_SENSOR";
        default:
            return "UNKNOWN";
    }
}

int formatPacketHeader(char *outputBuffer, const uint8_t *packetHeader)
{
    return sprintf(
        outputBuffer,
        "0x%02X 0x%02X %s 0x%02X ",
        packetHeader[HEADER_START_MARKER_OFFSET],
        packetHeader[HEADER_PAYLOAD_CRC_OFFSET],
        getPayloadLabel(packetHeader[HEADER_PAYLOAD_ID_OFFSET]),
        packetHeader[HEADER_PAYLOAD_LENGTH_OFFSET]);
}

int formatPingPayload(char *outputBuffer, const PingPayload *payload)
{
    return sprintf(
        outputBuffer,
        "[0x%02X]",
        payload->ping);
}

int formatPongPayload(char *outputBuffer, const PongPayload *payload)
{
    return sprintf(
        outputBuffer,
        "[0x%02X]",
        payload->pong);
}

int formatSensorPayload(char *outputBuffer, const SensorPayload *payload)
{return 0; /*
    return sprintf(
        outputBuffer,
        "[%02.3f %02.3f %02.3f]",
        payload->accelerometerMSS[0],
        payload->accelerometerMSS[1],
        payload->accelerometerMSS[2]);*/
}

int formatPayload(char *outputBuffer, const uint8_t *packet)
{
    switch (packet[HEADER_PAYLOAD_ID_OFFSET])
    {
        case PAYLOAD_PING:
        {
            const PingPacket *pingPacket = (const PingPacket *)packet;
            return formatPingPayload(outputBuffer, &pingPacket->payload);
        }
        case PAYLOAD_PONG:
        {
            const PongPacket *pongPacket = (const PongPacket *)packet;
            return formatPongPayload(outputBuffer, &pongPacket->payload);
        }
        case PAYLOAD_SENSOR:
        {
            const SensorPacket *sensorPacket = (const SensorPacket *) packet;
            return formatSensorPayload(outputBuffer, &sensorPacket->payload);
        }
        default:
        {
            return -1;
        }
    }
}

int formatPacket(char *outputBuffer, const uint8_t *packet)
{
    int headerSize = formatPacketHeader(outputBuffer, packet);
    if (headerSize < 0) {
        return -1;
    }
    int payloadSize = formatPayload(&outputBuffer[headerSize], packet);
    if (payloadSize < 0) {
        return -1;
    }
    return headerSize + payloadSize;
}

