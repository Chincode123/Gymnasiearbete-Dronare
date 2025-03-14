class SmoothValue {
    float value;
    float smoothingFactor;
public:
    SmoothValue(float initialValue, float smoothingFactor);
    
    operator float();
    
    SmoothValue& operator=(float newValue);
    
    SmoothValue& operator+=(float increment);

    SmoothValue& operator-=(float decrement);

    SmoothValue& operator*=(float factor);

    SmoothValue& operator/=(float divisor);
};