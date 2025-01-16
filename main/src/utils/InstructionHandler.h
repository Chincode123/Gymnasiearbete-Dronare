class Message {
public:
    bool initiated = false;
    uint8_t type, length;

    bool set(uint8_t type);

    void reset();
};

class InstructionReader {
    Message message;
    bool reading = false;
    bool acquiredData = false;
    uint8_t readIndex = 0;
    uint8_t readBuffer[32];
    uint8_t startMarker = 33;

public:
    bool read();

    uint8_t getData(uint8_t* out);
};

class InstructionWriter {
    Message message;

public:
    void write(uint8_t* data, uint8_t type);
};

class InstructionHandler {
public:
    InstructionReader reader;
    InstructionWriter writer;
};