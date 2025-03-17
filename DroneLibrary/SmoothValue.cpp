#include <SmoothValue.h>

SmoothValue::SmoothValue(float smoothingFactor) : value(0), smoothingFactor(smoothingFactor) {}

SmoothValue::SmoothValue(float smoothingFactor, float initialValue) : value(initialValue), smoothingFactor(smoothingFactor) {}

SmoothValue::operator float() {
    return value;
}

SmoothValue& SmoothValue::operator=(float newValue) {
    value = newValue;
}

SmoothValue& SmoothValue::operator+=(float increment) {
    set(value + increment);
    return *this;
}

SmoothValue& SmoothValue::operator-=(float decrement) {
    set(value - decrement);
    return *this;
}

SmoothValue& SmoothValue::operator*=(float factor) {
    set(value * factor);
    return *this;
}

SmoothValue& SmoothValue::operator/=(float divisor) {
    set(value / divisor);
    return *this;
}

void SmoothValue::set(float newValue) {
    value = value * (1 - smoothingFactor) + newValue * smoothingFactor;
}