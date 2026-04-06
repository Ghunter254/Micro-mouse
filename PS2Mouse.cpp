#include "PS2Mouse.h"

// PS/2 Commands
#define RESET                 0xFF
#define SET_SAMPLE_RATE       0xF3
#define SET_RESOLUTION        0xE8
#define SET_STREAM_MODE       0xEA
#define ENABLE_DATA_REPORTING 0xF4

PS2Mouse::PS2Mouse(uint8_t clockPin, uint8_t dataPin):
    _clockPin(clockPin),
    _dataPin(dataPin) {}


void PS2Mouse::high(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

void PS2Mouse::low(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void PS2Mouse::initialize() {
    high(_clockPin);
    high(_dataPin);
    delay(500);

    reset();

    setResolution(0); 
    setSampleRate(100);

    writeAndReadAck(SET_STREAM_MODE);
    writeAndReadAck(ENABLE_DATA_REPORTING);

    delay(100);
}

void PS2Mouse::writeByte(uint8_t data) {
    uint8_t parity = 1;

    high(_dataPin);
    high(_clockPin);
    delayMicroseconds(300);

    low(_clockPin);
    delayMicroseconds(300);
    low(_dataPin);
    delayMicroseconds(10);

    high(_clockPin);
    waitForClockState(LOW);

    for (int i = 0; i < 8; i++) {
        int bit = (data >> i) & 1;
        writeBit(bit);
        parity ^= bit;
    }

    writeBit(parity); // parity
    high(_dataPin);   // stop bit

    delayMicroseconds(50);
    waitForClockState(LOW);

    uint32_t timeout = 50000;
    while ((digitalRead(_clockPin) == LOW || digitalRead(_dataPin) == LOW ) && timeout > 0) {
      timeout --;
    };

    low(_clockPin);
}

void PS2Mouse::writeBit(int bit) {
    if (bit) high(_dataPin);
    else low(_dataPin);

    waitForClockState(HIGH);
    waitForClockState(LOW);
}

uint8_t PS2Mouse::readByte() {
    uint8_t data = 0;

    high(_clockPin);
    high(_dataPin);
    delayMicroseconds(50);

    waitForClockState(LOW);
    delayMicroseconds(5);

    waitForClockState(HIGH);

    for (int i = 0; i < 8; i++) {
        data |= (readBit() << i);
    }

    readBit(); // parity
    readBit(); // stop

    low(_clockPin);
    return data;
}

int PS2Mouse::readBit() {
    waitForClockState(LOW);
    int bit = digitalRead(_dataPin);
    waitForClockState(HIGH);
    return bit;
}

void PS2Mouse::waitForClockState(int state) {
    uint32_t timeout = 50000;
    while (digitalRead(_clockPin) != state && timeout > 0) {
      timeout--;
    };
}

void PS2Mouse::writeAndReadAck(uint8_t data) {
    writeByte(data);
    readByte(); // ACK
}

void PS2Mouse::reset() {
    writeAndReadAck(RESET);
    readByte(); // 0xAA
    readByte(); // device ID
}

void PS2Mouse::setSampleRate(uint8_t rate) {
    writeAndReadAck(SET_SAMPLE_RATE);
    writeAndReadAck(rate);
}

void PS2Mouse::setResolution(uint8_t resolution) {
    writeAndReadAck(SET_RESOLUTION);
    writeAndReadAck(resolution);
}

MouseData PS2Mouse::readData() {
    MouseData data;

    data.status = readByte();
    data.position.x = (int8_t)readByte(); // 🔥 signed
    data.position.y = (int8_t)readByte(); // 🔥 signed
    data.wheel = 0;

    return data;
}
