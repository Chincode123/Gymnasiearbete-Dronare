class Message {
public:
    bool initiated = false;
    uint8_t type, length;

    void set(uint8_t type);

    void reset();
};

class InstructionReader {
    Message message;
    bool reading = false;
    bool acquiredData = false;
    uint8_t readIndex = 0;
    uint8_t readBuffer[32];
    char startMarker = '<';
    char endMarker = '>';

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