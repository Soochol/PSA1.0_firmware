#include "payload.h"

PayloadConvert::PayloadConvert(void) {
    buffer = new(std::nothrow) char[PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE];
    if (!buffer) {
        // Handle allocation failure gracefully
        bufferIndex = 0;
        return;
    }
    reset();
}

PayloadConvert::~PayloadConvert() {
    delete[] buffer;
}

void PayloadConvert::reset(void) {
    bufferIndex = 0;
    if (buffer) {
        buffer[0] = '\0';  // Null-terminate only if buffer exists
    }
}

void PayloadConvert::appendToBuffer(const char* key, const char* value) {
    if (!key || !value || !buffer) return;  // NULL checks
    
    int written = snprintf(
        buffer + bufferIndex,
        PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE - bufferIndex,
        "%s: %s, ",
        key, value
    );
    
    // Safe buffer update with overflow protection
    if (written > 0 && bufferIndex + written < PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE) {
        bufferIndex += written;
    }
    // Note: Silent overflow protection - buffer remains in valid state
}

void PayloadConvert::appendToBuffer(const char* key, int value) {
    if (!key || !buffer) return;  // NULL checks
    
    int written = snprintf(
        buffer + bufferIndex,
        PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE - bufferIndex,
        "%s: %d, ",
        key, value
    );
    
    // Safe buffer update with overflow protection
    if (written > 0 && bufferIndex + written < PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE) {
        bufferIndex += written;
    }
    // Note: Silent overflow protection - buffer remains in valid state
}

char *PayloadConvert::getBuffer(void) {
    if (!buffer) return nullptr;  // Safety check
    
    // Remove the trailing comma and space safely
    if (bufferIndex >= 2 && bufferIndex < PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE) {
        buffer[bufferIndex - 2] = '\0';
    } else if (bufferIndex > 0 && bufferIndex < PAYLOAD_BUFFER_SEND_MAXIMUM_SIZE) {
        buffer[bufferIndex - 1] = '\0';  // At least null-terminate
    } else if (bufferIndex == 0) {
        buffer[0] = '\0';  // Empty buffer
    }
    return buffer;
}

void PayloadConvert::addDeviceId() {
    appendToBuffer("dId", cfg->thing_name);
    appendToBuffer("ver", FV_CODE);
    appendToBuffer("model", MODEL_NUMBER);
}

void PayloadConvert::setTitle(int8_t title) {
    appendToBuffer("title", title);
}

void PayloadConvert::addSensorData() {
    appendToBuffer("gpsLat", allData.latitude);
    appendToBuffer("gpsLong", allData.longitude);
    // breathRate = add_gaussian_noise(breathRate, 2);
    appendToBuffer("breathRate", allData.breathRate);
    // stepCount = add_gaussian_noise(stepCount, 4);
    appendToBuffer("stepCount", allData.stepCount);
    // lmaCount = add_gaussian_noise(lmaCount, 1);
    appendToBuffer("lmaCount", allData.lmaCount);
    // torsoAngleMean = add_gaussian_noise(torsoAngleMean, 1);
    appendToBuffer("torsoAngleMean", allData.torsoAngleMean);
    // torsoAngleMin = add_gaussian_noise(torsoAngleMin, 1);
    appendToBuffer("torsoAngleMin", allData.torsoAngleMin);
    // bentDuration = add_gaussian_noise(bentDuration, 1);
    appendToBuffer("bentDuration", allData.bentDuration);
}

void PayloadConvert::addDeviceData() {
    appendToBuffer("battery", allData.battery);
}

void PayloadConvert::addDeviceStates() {
    appendToBuffer("isConnected", cfg->isConnected ? 1 : 0);
}

void PayloadConvert::addUserId() {
    appendToBuffer("uId", "dummyUId");
}

void PayloadConvert::initialize() {
    reset();  // Simple initialization - just reset the buffer
}
