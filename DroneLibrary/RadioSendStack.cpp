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
    radioStackElement* element = (radioStackElement*)malloc(sizeof(radioStackElement));
    if (element == NULL) {
        // Handle allocation error
        return nullptr;
    }
    
    element->value = malloc(size);
    if (element-> value == NULL) {
        // Handle allocation error
        return nullptr;
    }

    element->size = size;
    memcpy(element->value, data, size);
    return element;
}

bool RadioSendStack::removeAt(uint8_t index) {
    count--;
    if (index == 0) {
        radioStackElement* newFirst = firstElement->next;
        free(firstElement->value);
        free(firstElement->value);
        free(firstElement);
        firstElement = newFirst;
        return true;
        return true;
    }
    else if (index == count - 1) {
        radioStackElement* newLast = get(count - 2);
        free(lastElement->value);
        free(lastElement->value);
        free(lastElement);
        lastElement = newLast;
        return true;
        return true;
    }
    else if (index >= count) {
        return false;
    }

    radioStackElement* previousElement = get(index - 1);
    previousElement->next = previousElement->next->next;
    free(previousElement->next->value);
    free(previousElement->next->value);
    free(previousElement->next);
    return true;
}

bool RadioSendStack::push(void* data, uint8_t size) {
    radioStackElement* element = create(data, size);

    if (element == nullptr) {
        return false;
    }

    element->next = firstElement;
    firstElement = element;
    return true;
}

bool RadioSendStack::queue(void* data, uint8_t size) {
    radioStackElement* element = create(data, size);

    if (element == nullptr) {
        return false;
    }

    lastElement->next = element;
    lastElement = element;
    return true;
}

uint8_t RadioSendStack::pop(void* buffer) {
    pop(0, buffer);
}

uint8_t RadioSendStack::pop(uint8_t index, void* buffer) {
    radioStackElement* element = get(index);
    uint8_t size = element->size;
    memcpy(buffer, element->value, size);
    removeAt(index);
    return size;
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