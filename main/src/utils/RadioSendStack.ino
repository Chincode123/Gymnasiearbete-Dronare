#include "RadioSendStack.h"

radioStackElement* RadioSendStack::get(uint8_t index) {
    return get(firstElement, 0, index);
}

radioStackElement* RadioSendStack::get(radioStackElement* currentElement, uint8_t currentIndex, uint8_t targetIndex) {
    if (currentIndex == targetIndex) {
        return currentElement;
    }

    return get(currentElement->next, currentIndex + 1, targetIndex);
}

radioStackElement* RadioSendStack::create(void* data, uint8_t size) {
    count++;
    radioStackElement* element = (radioStackElement*)malloc(size + sizeof(radioStackElement) - 1);
    element->size = size;
    memcpy(element->value, data, size);
    return element;
}

void RadioSendStack::removeAt(uint8_t index) {
    count--;
    if (index == 0) {
        radioStackElement* newFirst = firstElement->next;
        free(firstElement);
        firstElement = newFirst;
    }
    else if (index == count) {
        free(lastElement);
    }

    radioStackElement* previousElement = get(index - 1);
    previousElement->next = previousElement->next->next;
    free(previousElement->next);
}

void RadioSendStack::push(void* data, uint8_t size) {
    radioStackElement* element = create(data, size);
    element->next = firstElement;
    firstElement = element;
}

void RadioSendStack::queue(void* data, uint8_t size) {
    radioStackElement* element = create(data, size);
    lastElement->next = element;
    lastElement = element;
}

void RadioSendStack::pop() {
    pop(0);
}

void RadioSendStack::pop(uint8_t index) {
    pop(index, nullptr);
}

void RadioSendStack::pop(void* buffer) {
    pop(0, buffer);
}

void RadioSendStack::pop(uint8_t index, void* buffer) {
    radioStackElement* element = get(index);
    memcpy(buffer, element->value, element->size);
    removeAt(index);
}

void RadioSendStack::clear() {
    radioStackElement* next = firstElement;

    while (count > 0) {
        radioStackElement* current = next; 
        next = current->next;
        free(current);
        count--;
    }
}