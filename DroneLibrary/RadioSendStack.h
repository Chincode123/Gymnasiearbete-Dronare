#ifndef RADIO_SEND_STACK_H_
#define RADIO_SEND_STACK_H_

#include <Arduino.h>
#include "RadioData.h"

struct radioStackElement {
    RadioMessage value;
    radioStackElement* next;
};

class RadioSendStack {
    radioStackElement *firstElement, *lastElement;

    radioStackElement* create(const RadioMessage& data);
    bool removeAt(uint8_t index);

    radioStackElement* get(uint8_t index);
    radioStackElement* get(radioStackElement* currentElement, uint8_t curentIndex, uint8_t targetIndex);
public:
    uint8_t count;

    bool push(const RadioMessage& data);
    bool queue(const RadioMessage& data);

    RadioMessage pop();
    RadioMessage pop(uint8_t index);

    void clear();
};
#endif