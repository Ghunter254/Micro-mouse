#pragma once

#include<Arduino.h>

typedef struct __attribute__((packed)) {
    int8_t x;
    int8_t y;
} Position;

typedef struct __attribute__((packed)) {
    uint8_t status;
    Position position;
    int8_t wheel;
} MouseData;


class PS2Mouse {
private:
    const uint8_t _clockPin;
    const uint8_t _dataPin;

    inline void high(uint8_t pin);
    inline void low(uint8_t pin);
    inline void writeBit(int bit);
    inline int readBit();
    inline void waitForClockState(int state);

    void writeByte(uint8_t data);
    uint8_t readByte();
    void writeAndReadAck(uint8_t data);

    void reset();
    void setSampleRate(uint8_t rate);
    void setResolution(uint8_t resolution);

public:
    PS2Mouse(uint8_t clockPin, uint8_t dataPin);

    void initialize();
    MouseData readData();
};
