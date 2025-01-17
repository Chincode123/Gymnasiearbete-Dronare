let port;

document.querySelector("#select-serial").addEventListener("click", async () => {
  port = await navigator.serial.requestPort();
  await port.open({ baudRate: 9600 });
  console.log(port);
  read();
//   setInterval(read);
  setInterval(sendControllerInstructions, 20);
});

write = async (data) => {
  const writer = port.writable.getWriter();

  console.log(writer);

  await writer.write(data);

  // Allow the serial port to be closed later.
  writer.releaseLock();
};

read = async () => {
  while (port.readable) {
    console.log("port", port);
    const reader = port.readable.getReader();
    try {
      while (true) {
        const { value, done } = await reader.read();
        console.log("reader done", done);

        if (done) {
          document.getElementById("output").innerHTML += "<br>";
          // Allow the serial port to be closed later.
          reader.releaseLock();
          console.log("realesed");
          break;
        }
        if (value) {
          for (const byte of value) {
            const result = serialReader.read(byte);

            if (result.done) {
              document.getElementById("output").innerHTML += "<br>";
              console.log(result.data);

              if (result.messageType.split("-")[0] == "pid") {
                console.log(serialReader.readPIDInstruction(result.data));
              }
            }
          }

          document.getElementById("output").innerHTML += new TextDecoder().decode(value);
          if (document.getElementById("output").innerHTML.length > 500) {
            document.getElementById("output").innerHTML = document.getElementById("output").innerHTML.substring(10);
          }
        }
      }
    } catch (error) {
      console.log(error);
    }
  }
};

document.getElementById("open").addEventListener("click", async () => {
  try {
    await port.close();
  } catch (error) {
    console.log(error);
  }

  await port.open({ baudRate: 9600 });
});

class buttonType {
  activated;
  value;
}

const buttons = new Array(8);
// populate
let buttonActivationValue = 1;
for (let i = 0; i < 8; i++) {
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
  values[0] = (x * 127) & 0x0000ff;
  values[1] = (y * 127) & 0x0000ff;
  values[2] = (power * 127) & 0x0000ff;

  let buttonValues;
  buttons.forEach((button) => {
    if (button.activated) {
      buttonValues = buttonValues | button.value;
    }
  });

  uint8 = new Uint8Array(4);
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
  if (module == "velocity") {
    messageType = 1;
  } else if (module == "pitch") {
    messageType = 2;
  } else if (module == "roll") {
    messageType = 3;
  } else {
    throw {
      name: "PID-ModuleError",
      message: "PID moudule type does not exist",
    };
  }

  const out = toInstruction(byteValues, messageType);

  console.log(out);

  return out;
};

toInstruction = (values, messageType) => {
  out = new Uint8Array(values.length + 2);
  out[0] = 33;
  out[1] = messageType;
  for (let i = 0; i < values.length; i++) {
    out[i + 2] = values[i];
  }
  return out;
};

class SerialMessage {
  initiated = false;
  type;
  length;

  messageLengths = new Map([
    [0, 4],
    [1, 12],
    [2, 12],
    [3, 12],
  ]);
  messageTypes = new Map([
    [0, "controller-instructions"],
    [1, "pid-velocity"],
    [2, "pid-pitch"],
    [3, "pid-roll"],
  ]);

  set = (type) => {
    if (this.messageLengths.has(type)) {
      this.initiated = true;
      this.type = type;
      this.length = this.messageLengths.get(type);
      return true;
    }
    return false;
  };

  reset = () => {
    this.initiated = false;
  };
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
      } else if (this.data.length < this.message.length - 1) {
        this.data[this.data.length] = byte;
      } else {
        this.data[this.data.length] = byte;
        this.reading = false;
        const messageType = this.message.messageTypes.get(this.message.type);
        this.message.reset();
        const out = new Uint8Array(this.data);
        this.data = [];
        return {
          data: out,
          done: true,
          messageType: messageType,
        };
      }
    } else if (byte == this.startMarker) {
      this.reading = true;
    }

    return { done: false };
  };

  readPIDInstruction = (data) => {
    const floatValues = new Float32Array(data.buffer);
    return { p: floatValues[0], i: floatValues[1], d: floatValues[2] };
  };
}
serialReader = new SerialReader();

let using_joystick = false;
const joystick = { x: 0, y: 0 };
const joystick_area = document.getElementById("joystick-area");
const joystick_marker = document.getElementById("joystick-marker");
const joystick_out_x = document.getElementById("joystick-x-out");
const joystick_out_y = document.getElementById("joystick-y-out");

document.addEventListener("mousemove", (event) => {
  if (!using_joystick) {
    return;
  }

  joystick_area_size = joystick_area.getBoundingClientRect();
  joystick_area_width = joystick_area_size.right - joystick_area_size.left;

  joystick.x =
    ((event.clientX - joystick_area_size.right) / joystick_area_width + 0.5) *
    2;
  joystick.y =
    ((event.clientY - joystick_area_size.top) / joystick_area_width - 0.5) * -2;

  joystick.x = Math.min(Math.max(joystick.x, -1), 1);
  joystick.y = Math.min(Math.max(joystick.y, -1), 1);

  const joystick_magnitude = Math.sqrt(
    joystick.x * joystick.x + joystick.y * joystick.y
  );
  if (joystick_magnitude > 1) {
    joystick.x /= joystick_magnitude;
    joystick.y /= joystick_magnitude;
  }
});

document
  .getElementById("joystick-area")
  .addEventListener("mousedown", (event) => {
    using_joystick = true;
  });

document.addEventListener("mouseup", () => {
  if (!using_joystick) {
    return;
  }
  using_joystick = false;
  joystick_marker.style.left = 0;
  joystick_marker.style.top = 0;
  joystick.x = 0;
  joystick.y = 0;
});

let gamepad_index;
window.addEventListener("gamepadconnected", (event) => {
  gamepad_index = event.gamepad.index;
  setInterval(gamepadInputLoop);
});

gamepadInputLoop = () => {
  const gamepad = navigator.getGamepads()[gamepad_index];

  joystick.x = gamepad.axes[0];
  joystick.y = -gamepad.axes[1];

  power = -gamepad.buttons[6].value + gamepad.buttons[7].value;
  power *= Math.abs(power);

  console.log(joystick);
  console.log(power);
};

let power = 0;
const power_out = document.getElementById("power-out");
const power_slider = document.getElementById("power");
power_slider.addEventListener("input", (event) => {
  power = event.target.value;
});

updateUI = () => {
  power_out.innerText = (parseFloat(power) * 100).toFixed() + "%";

  power_slider.value = power;

  joystick_out_x.innerText = joystick.x.toFixed(2);
  joystick_out_y.innerText = joystick.y.toFixed(2);

  joystick_marker.style.left = `${joystick.x * 50}%`;
  joystick_marker.style.top = `${joystick.y * -50}%`;
};

setInterval(updateUI);

document
  .getElementById("velocity")
  .querySelector("button")
  .addEventListener("click", () => {
    const instructions = getPIDInstructions(
      document.getElementById("pid-v-p").value,
      document.getElementById("pid-v-i").value,
      document.getElementById("pid-v-d").value,
      "velocity"
    );

    write(instructions);
  });

document
  .getElementById("pitch")
  .querySelector("button")
  .addEventListener("click", () => {
    const instructions = getPIDInstructions(
      document.getElementById("pid-p-p").value,
      document.getElementById("pid-p-i").value,
      document.getElementById("pid-p-d").value,
      "pitch"
    );

    write(instructions);
  });

document
  .getElementById("roll")
  .querySelector("button")
  .addEventListener("click", () => {
    const instructions = getPIDInstructions(
      document.getElementById("pid-r-p").value,
      document.getElementById("pid-r-i").value,
      document.getElementById("pid-r-d").value,
      "roll"
    );

    write(instructions);
  });

document.getElementById("write-all-pid").addEventListener("click", () => {
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
});

sendControllerInstructions = () => {
  instructions = getControllerInstructions(
    joystick.x,
    joystick.y,
    document.getElementById("power").value
  );
  write(instructions);
};
