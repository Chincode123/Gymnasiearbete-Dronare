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
    out = new Uint8Array(7)
    out[0] = 60;
    out[1] = messageType;
    out[2] = values[0];
    out[3] = values[1];
    out[4] = values[2];
    out[5] = buttonValues;
    out[6] = 62;

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

    out = new Uint8Array(15);
    out[0] = 60;
    out[1] = messageType;
    out[14] = 62;
    for (let i = 2; i < 13; i++){
        out[i] = byteValues[i-2];
    }

    return out;
};


let using_joystick = false;
const joystick = {x: 0, y: 0}; 
const joystick_area = document.getElementById('joystick-area');
const joystick_marker = document.getElementById('joystick-marker');
const joystick_out_x = document.getElementById('joystick-x-out');
const joystick_out_y = document.getElementById('joystick-y-out');

document.addEventListener('mousemove', (event) => {
    if (!using_joystick) {
        return;
    }

    joystick_area_size = joystick_area.getBoundingClientRect();
    joystick_area_width = joystick_area_size.right - joystick_area_size.left;
 
    joystick.x = (((event.clientX - joystick_area_size.right) / joystick_area_width) + 0.5) * 2;
    joystick.y = (((event.clientY - joystick_area_size.top)  / joystick_area_width) - 0.5) * -2;

    joystick.x = Math.min(Math.max(joystick.x, -1), 1);
    joystick.y = Math.min(Math.max(joystick.y, -1), 1);

    const joystick_magnitude = Math.sqrt(joystick.x * joystick.x + joystick.y * joystick.y);
    if (joystick_magnitude > 1) {
        joystick.x /= joystick_magnitude;
        joystick.y /= joystick_magnitude;
    }

    joystick_out_x.innerText = joystick.x.toFixed(2);
    joystick_out_y.innerText = joystick.y.toFixed(2);

    joystick_marker.style.left = `${joystick.x*50}%`;
    joystick_marker.style.top = `${joystick.y*-50}%`;
})

document.getElementById("joystick-area").addEventListener('mousedown', (event) => {
    using_joystick = true;
})

document.addEventListener('mouseup', () => {
    if (!using_joystick) {
        return;
    }
    using_joystick = false;
    joystick_marker.style.left = 0;
    joystick_marker.style.top = 0;
    joystick.x = 0;
    joystick.y = 0;
    joystick_out_x.innerText = joystick.x.toFixed(2);
    joystick_out_y.innerText = joystick.y.toFixed(2);
})


const power_out = document.getElementById("power-out");
document.getElementById('power').addEventListener('input', (event) => {
    power_out.innerText = parseFloat(event.target.value).toFixed(2);
})

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