#ifndef RADIO_SEND_STACK_H_
#define RADIO_SEND_STACK_H_

#include <Arduino.h>

struct radioStackElement {
    void* value;
    uint8_t size;
    radioStackElement* next;
};

class RadioSendStack {
    radioStackElement *firstElement, *lastElement;

    radioStackElement* create(void* data, uint8_t size);
    bool removeAt(uint8_t index);

    radioStackElement* get(uint8_t index);
    radioStackElement* get(radioStackElement* currentElement, uint8_t curentIndex, uint8_t targetIndex);
public:
    uint8_t count;

    bool push(void* data, uint8_t size);
    bool queue(void* data, uint8_t size);

    uint8_t pop(void* buffer);
    uint8_t pop(uint8_t index, void* buffer);

    void clear();
};
#endif