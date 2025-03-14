#include <SmoothValue.h>

SmoothValue::SmoothValue(float initialValue, float smoothingFactor) {
    this->value = initialValue;
    this->smoothingFactor = smoothingFactor;
}

SmoothValue::operator float() {
    return value;
}

SmoothValue& SmoothValue::operator=(float newValue) {
    value = value * (1 - smoothingFactor) + newValue * smoothingFactor;
    return *this;
}

SmoothValue& SmoothValue::operator+=(float increment) {
    *this = value + increment;
    return *this;
}

SmoothValue& SmoothValue::operator-=(float decrement) {
    *this = value - decrement;
    return *this;
}

SmoothValue& SmoothValue::operator*=(float factor) {
    *this = value * factor;
    return *this;
}

SmoothValue& SmoothValue::operator/=(float divisor) {
    *this = value / divisor;
    return *this;
}

