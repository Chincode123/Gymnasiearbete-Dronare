let port;

document.querySelector("#select-serial").addEventListener("click", async () => {
    port = await navigator.serial.requestPort();
    setInterval(read, 10);
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

class buttonType {
    activated;
    value;
};

const buttons = new Array(8);
// populate
let buttonActivationValue = 1;
for (let i = 0; i < 8; i++){
    buttons[i] = new buttonType();
    buttons[i].value = buttonActivationValue;
    buttons[i].activated = false;
    buttonActivationValue = buttonActivationValue << 1;
}

switchButtonActivation = (index) => {
    console.log("set: " + index);
    buttons[index].activated = !buttons[index].activated;
};

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

    const messageType = 0;
    out = new Uint8Array(6)
    out[0] = messageType;
    out[1] = values[0];
    out[2] = values[1];
    out[3] = values[2];
    out[4] = buttonValues;
    out[5] = messageType;

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
        messageType = 1
    } 
    else if (module == "pitch") {
        messageType = 2
    }
    else if (module == "roll") {
        messageType = 3
    }
    else {
        throw {name: "PID-ModuleError", message:"PID moudule type does not exist"};
    }

    out = new Uint8Array(14);
    out[0] = messageType;
    out[13] = messageType;
    for (let i = 1; i < 13; i++){
        out[i] = byteValues[i-1];
    }
    return out;
}