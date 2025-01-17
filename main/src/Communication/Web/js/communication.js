class SerialMessage{
    initiated = false;
    type;
    length;

    messageLengths = new Map([[0, 4], [1, 12], [2, 12], [3, 12]]);
    messageTypes = new Map([[0, "controller-instructions"], [1, "pid-velocity"], [2, "pid-pitch"], [3, "pid-roll"]]);

    set = (type) => {
        if (this.messageLengths.has(type)) {
            this.initiated = true;
            this.type = type;
            this.length = this.messageLengths.get(type);
            return true;
        }
        return false;
    }

    reset = () => {
        this.initiated = false;
    }
}

class SerialReader {
    message = new SerialMessage();
    data = [];
    reading = false;
    startMarker = 33;

    read = (byte) => {
        if (this.reading) {
            if (!this.message.initiated) {
                if (!this.message.set(byte)) {
                    this.reading = false;
                }
            }
            else if (this.data.length < this.message.length) {
                this.data[this.data.length] = byte;
            }
            else  {
                this.data[this.data.length] = byte;
                reading = false;
                return {data: new Uint8Array(data), done: true, messageType: this.message.messageTypes.get(this.message.type)};
            }
        }
        else if (byte == this.marker.start) {
            reading = true
        }

        this.message.reset();

        return {done: false};
    }

    readPIDInstruction = (data) => {
        const floatValues = new Float32Array(data.buffer);
        return {p: floatValues[0], i: floatValues[1], d: floatValues[2]};
    }
}
serialReader = new SerialReader();


let port;
document.querySelector("#select-serial").addEventListener("click", async () => {
    port = await navigator.serial.requestPort();
    setInterval(read, 10);
    setInterval(sendControllerInstructions);
});

let writing = false
write = async (data) => {
    try {
        const writer = port.writable.getWriter();
    }
    catch (error){
        console.log(error);
    }

    await writer.write(data);

    // Allow the serial port to be closed later.
    writer.releaseLock();
};

read = async () => {
    
    while (port.readable) {
        const reader = port.readable.getReader();
      
        try {
          while (true) {

            const { value, done } = await reader.read();

            if (done) {
              // Allow the serial port to be closed later.
              document.getElementById('output').innerHTML += "<br>";
              reader.releaseLock();
              break;
            }
            if (value) {
                console.log(value);

                for (const byte of value) {
                    const result = serialReader.read(byte);

                    if (result.done) {
                        console.log(result.value);
                    }
                }

                document.getElementById('output').innerHTML += new TextDecoder().decode(value);
            }
          }
        } catch (error) {
            console.log(error);
        }
    }
};

document.getElementById("open").addEventListener('click', async () => {
    try {
        await port.close();
    }
    catch (error) {
        console.log(error);
    }
    
    await port.open({baudRate: 9600});
});


toInstruction = (values, messageType) => {
    out = new Uint8Array(values.length + 2)
    out[0] = 33;
    out[1] = messageType;
    for (let i = 0; i < values.length; i++){
        out[i + 2] = values[i];
    }
    return out;
}

// -1 <= x, y, power <= 1
getControllerInstructions = (x, y, power) => {
    const values = new Int8Array(3);
    values[0] = (x * 127) & 0x0000FF;
    values[1] = (y * 127) & 0x0000FF;
    values[2] = (power * 127) & 0x0000FF;

    let buttonValues;
    buttons.forEach(button => {
        if (button.activated){
            buttonValues = buttonValues | button.value;
        }
    });
    
    uint8 = new Uint8Array(4)
    uint8[0] = values[0];
    uint8[1] = values[1];
    uint8[2] = values[2];
    uint8[3] = buttonValues;

    const messageType = 0;
    const out = toInstruction(uint8, messageType);

    console.log(out);

    return out;
};

getPIDInstructions = (p, i, d, module) => {
    const floatValues = new Float32Array(3);
    floatValues[0] = p;
    floatValues[1] = i;
    floatValues[2] = d;

    const byteValues = new Uint8Array(floatValues.buffer);

    let messageType;
    if (module == "velocity"){
        messageType = 1;
    } 
    else if (module == "pitch") {
        messageType = 2;
    }
    else if (module == "roll") {
        messageType = 3;
    }
    else {
        throw {name: "PID-ModuleError", message:"PID moudule type does not exist"};
    }

    const out = toInstruction(byteValues, messageType);

    return out;
};


document.getElementById('velocity').querySelector("button").addEventListener('click', () => {
    const instructions = getPIDInstructions(
        document.getElementById("pid-v-p").value,
        document.getElementById("pid-v-i").value,
        document.getElementById("pid-v-d").value,
        "velocity"
    );

    write(instructions);
})

document.getElementById('pitch').querySelector("button").addEventListener('click', () => {
    const instructions = getPIDInstructions(
        document.getElementById("pid-p-p").value,
        document.getElementById("pid-p-i").value,
        document.getElementById("pid-p-d").value,
        "pitch"
    );

    write(instructions);
})

document.getElementById('roll').querySelector("button").addEventListener('click', () => {
    const instructions = getPIDInstructions(
        document.getElementById("pid-r-p").value,
        document.getElementById("pid-r-i").value,
        document.getElementById("pid-r-d").value,
        "roll"
    );

    write(instructions);
})

document.getElementById("write-all-pid").addEventListener('click', () => {
    let instructions = getPIDInstructions(
        document.getElementById("pid-v-p").value,
        document.getElementById("pid-v-i").value,
        document.getElementById("pid-v-d").value,
        "velocity"
    );

    write(instructions);

    instructions = getPIDInstructions(
        document.getElementById("pid-p-p").value,
        document.getElementById("pid-p-i").value,
        document.getElementById("pid-p-d").value,
        "pitch"
    );

    write(instructions);
    
    instructions = getPIDInstructions(
        document.getElementById("pid-r-p").value,
        document.getElementById("pid-r-i").value,
        document.getElementById("pid-r-d").value,
        "roll"
    );

    write(instructions);
})

sendControllerInstructions = () => {
    instructions = getControllerInstructions(
        joystick.x,
        joystick.y,
        document.getElementById('power').value
    );
    write(instructions);
}