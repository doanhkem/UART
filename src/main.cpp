#include <WiFi.h>

#define BAUDRATE 115200

struct PowerInfo {
    uint16_t voltage;
    uint16_t current;
    uint16_t power;
    uint32_t energy;
};

uint8_t channelControlState = 0x00;

uint16_t calcrc(char *ptr, int count) {
    uint16_t crc = 0;
    while (--count >= 0) {
        crc ^= (int)*ptr++ << 8;
        for (char i = 8; i > 0; --i) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc & 0xFFFF;
}

void decodeData(uint8_t* buffer, PowerInfo& info) {
    info.voltage = (buffer[4] << 8) | buffer[5];
    info.current = (buffer[6] << 8) | buffer[7];
    info.power = (buffer[8] << 8) | buffer[9];
    info.energy = (buffer[10] << 24) | (buffer[11] << 16) | (buffer[12] << 8) | buffer[13];
}

void sendMessage(uint16_t eventCode, uint8_t* eventPayload, size_t payloadLength) {
    const uint8_t startOfMessage = 0xA5;
    uint8_t message[255];
    
    message[0] = startOfMessage;
    message[1] = 2 + payloadLength;
    message[2] = highByte(eventCode);
    message[3] = lowByte(eventCode);
    
    for (size_t i = 0; i < payloadLength; i++) {
        message[4 + i] = eventPayload[i];
    }

    uint16_t checksum = calcrc((char*)message, 4 + payloadLength);
    message[4 + payloadLength] = highByte(checksum);
    message[5 + payloadLength] = lowByte(checksum);

    Serial.write(message, 6 + payloadLength);
}

PowerInfo sendAndReceiveDataToStm32(uint16_t eventCode, uint8_t* eventPayload, size_t payloadLength) {
    PowerInfo info = {0, 0, 0, 0};
    uint8_t buffer[255];
    size_t bytesRead = 0;
    size_t expectedLength = 0;
    const int maxAttempts = 3; // Số lần cố gắng tối đa
    int attempts = 0;

    while (attempts < maxAttempts) {
        sendMessage(eventCode, eventPayload, payloadLength);
        
        while (Serial.available() > 0) {
            uint8_t byte = Serial.read();

            if (bytesRead == 0 && byte == 0xA5) {
                buffer[bytesRead++] = byte;
            } else if (bytesRead == 1) {
                buffer[bytesRead++] = byte;
                expectedLength = byte + 3 + 2;
            } else if (bytesRead > 1) {
                buffer[bytesRead++] = byte;

                if (bytesRead == expectedLength) {
                    uint16_t receivedChecksum = (buffer[expectedLength - 2] << 8) | buffer[expectedLength - 1];
                    uint16_t calculatedChecksum = calcrc((char*)buffer, expectedLength - 2);

                    if (receivedChecksum == calculatedChecksum && buffer[3] == 0x03) {
                        decodeData(buffer, info);
                        return info; // Trả về thông tin điện năng nếu phản hồi hợp lệ
                    }

                    bytesRead = 0;
                    expectedLength = 0;
                }
            }
        }
        attempts++; // Tăng số lần cố gắng
        delay(100); // Delay trước khi gửi lại
    }
    return info; // Trả về thông tin điện năng (thường là thông tin mặc định nếu không nhận được phản hồi hợp lệ)
}

void setChannelControl(uint8_t bitPosition, bool value) {
    if (value) {
        channelControlState |= (1 << bitPosition);
    } else {
        channelControlState &= ~(1 << bitPosition);
    }
    uint8_t payload[] = { channelControlState };
    sendMessage(0x0001, payload, sizeof(payload));
}

PowerInfo getPowerInfo(uint8_t channel) {
    uint8_t payload[] = { channel };
    return sendAndReceiveDataToStm32(0x0003, payload, sizeof(payload));
}

void setup() {
    Serial.begin(BAUDRATE);
}

void loop() {
    PowerInfo powerInfo = getPowerInfo(0x08);
    Serial.println(powerInfo.voltage);
    Serial.println(powerInfo.current);
    Serial.println(powerInfo.power);
    Serial.println(powerInfo.energy);
    delay(200);
    setChannelControl(7,1);
    // delay(1000);
    // setChannelControl(0,0);

}
