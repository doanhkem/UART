#include <WiFi.h>

#define BAUDRATE 115200
#define START_OF_MESSAGE 0xA5
#define MAX_ATTEMPTS 5

struct PowerInfo {
    uint16_t voltage;
    uint16_t current;
    uint16_t power;
    uint32_t energy;
};

uint8_t channelControlState = 0x00;

// Khai báo các biến PowerInfo cho từng yêu cầu
PowerInfo powerInfo1 = {0, 0, 0, 0};
PowerInfo powerInfo2 = {0, 0, 0, 0};
PowerInfo powerInfo3 = {0, 0, 0, 0};
PowerInfo powerInfo4 = {0, 0, 0, 0};
PowerInfo powerInfo5 = {0, 0, 0, 0};
PowerInfo powerInfo6 = {0, 0, 0, 0};
PowerInfo powerInfo7 = {0, 0, 0, 0};
PowerInfo powerInfo8 = {0, 0, 0, 0};

uint16_t calcrc(const uint8_t *ptr, size_t count) {
    uint16_t crc = 0;
    while (count--) {
        crc ^= (*ptr++ << 8);
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

void decodeData(const uint8_t* buffer, PowerInfo& info) {
    info.voltage = (buffer[6] << 8) | buffer[7];
    info.current = (buffer[8] << 8) | buffer[9];
    info.power = (buffer[10] << 8) | buffer[11];
    info.energy = (buffer[12] << 24) | (buffer[13] << 16) | (buffer[14] << 8) | buffer[15];
}

void sendMessage(uint16_t messageID, uint16_t eventCode, const uint8_t* eventPayload, size_t payloadLength) {
    uint8_t message[255];
    
    message[0] = START_OF_MESSAGE;
    message[1] = 2 + 2 + payloadLength;  // Byte độ dài
    message[2] = highByte(messageID);    
    message[3] = lowByte(messageID);     
    message[4] = highByte(eventCode);    
    message[5] = lowByte(eventCode);     

    memcpy(&message[6], eventPayload, payloadLength);

    uint16_t checksum = calcrc(message, 6 + payloadLength);
    message[6 + payloadLength] = highByte(checksum);
    message[7 + payloadLength] = lowByte(checksum);

    Serial.write(message, 8 + payloadLength);
}

PowerInfo sendAndReceiveDataToStm32(uint16_t messageID, uint16_t eventCode, const uint8_t* eventPayload, size_t payloadLength, PowerInfo& oldInfo) {
    PowerInfo newInfo = {0, 0, 0, 0};
    uint8_t buffer[255];
    size_t bytesRead = 0;
    size_t expectedLength = 0;
    
    for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
        sendMessage(messageID, eventCode, eventPayload, payloadLength);
        
        unsigned long startTime = millis();
        while (millis() - startTime < 500) { // Timeout 500ms
            if (Serial.available() > 0) {
                uint8_t byte = Serial.read();

                if (bytesRead == 0 && byte == START_OF_MESSAGE) {
                    buffer[bytesRead++] = byte;
                } else if (bytesRead == 1) {
                    buffer[bytesRead++] = byte;
                    expectedLength = byte + 4;  
                } else if (bytesRead > 1) {
                    buffer[bytesRead++] = byte;

                    if (bytesRead == expectedLength) {
                        uint16_t receivedChecksum = (buffer[expectedLength - 2] << 8) | buffer[expectedLength - 1];
                        uint16_t calculatedChecksum = calcrc(buffer, expectedLength - 2);
                        
                        uint16_t receivedMessageID = (buffer[2] << 8) | buffer[3];
                        if (receivedChecksum == calculatedChecksum && receivedMessageID == messageID && buffer[5] == 0x03) {
                            decodeData(buffer, newInfo);
                            return newInfo; // Trả về thông tin điện năng mới nếu phản hồi hợp lệ
                        }

                        bytesRead = 0;
                        expectedLength = 0;
                    }
                }
            }
        }
        delay(100); 
    }
    return oldInfo; // Trả về thông tin cũ nếu không nhận được phản hồi hợp lệ
}

void setChannelControl(uint8_t bitPosition, bool value) {
    channelControlState = value ? (channelControlState | (1 << bitPosition)) : (channelControlState & ~(1 << bitPosition));
    uint8_t payload[] = { channelControlState };
    sendMessage(0x0000, 0x0001, payload, sizeof(payload));
}

PowerInfo getPowerInfo(uint8_t channel, uint16_t messageID, PowerInfo& oldInfo) {
    uint8_t payload[] = { channel };
    return sendAndReceiveDataToStm32(messageID, 0x0003, payload, sizeof(payload), oldInfo);
}



void setup() {
    Serial.begin(BAUDRATE);

}

void loop() {
    powerInfo1 = getPowerInfo(0x01, 0x0001, powerInfo1);
    powerInfo2 = getPowerInfo(0x02, 0x0002, powerInfo2);
    powerInfo3 = getPowerInfo(0x03, 0x0003, powerInfo3);
    powerInfo4 = getPowerInfo(0x04, 0x0004, powerInfo4);
    Serial.print("Channel 1 Energy: ");
    Serial.println(powerInfo1.current);
    Serial.print("Channel 2 Energy: ");
    Serial.println(powerInfo2.current);
    Serial.print("Channel 3 Energy: ");
    Serial.println(powerInfo3.current);
    Serial.print("Channel 4 Energy: ");
    Serial.println(powerInfo4.current);
    // Serial.print("kaccccccccc");
    
    delay(200); // Delay 1 giây trước khi hiển thị lại
}
