class Message {
public:
    bool initiated = false;
    uint8_t type, length;

    // Initiate message with a type
    // sets type and length, and initiated = true
    bool set(uint8_t& type);

    // Resets messege
    // initiated = false.
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
    // Read from serial port
    // Returns true if a message is ready, else false
    bool read();

    // Get data after read returns true
    // Returns message type
    uint8_t getData(uint8_t* out);
};

class InstructionWriter {
    Message message;

public:
    // Writes "data" as a message with the type "type"
    void write(uint8_t* data, uint8_t& type);
};

class InstructionHandler {
public:
    InstructionReader reader;
    InstructionWriter writer;
};