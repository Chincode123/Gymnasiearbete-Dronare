class SerialMessage {
  initiated = false;
  type;
  length;

  messageTypes = new Map([
    [0, "controller-instructions"],
    [1, "pid-velocity"],
    [2, "pid-pitch"],
    [3, "pid-roll"],
    [4, "request-pid-velocity"],
    [5, "request-pid-pitch"],
    [6, "request-pid-roll"],
    [7, "target-ranges"],
    [8, "request-target-ranges"],
    [9, "acknowledge"],
    [10, "drone-log"],
    [11, "activate"],
    [12, "deactivate"]
  ]);

  messageLengths = new Map([
    [this.messageTypes.get("controller-instructions"), 3],
    [this.messageTypes.get("pid-velocity"), 12],
    [this.messageTypes.get("pid-pitch"), 12],
    [this.messageTypes.get("pid-roll"), 12],
    [this.messageTypes.get("request-pid-velocity"), 0],
    [this.messageTypes.get("request-pid-pitch"), 0],
    [this.messageTypes.get("request-pid-roll"), 0],
    [this.messageTypes.get("target-ranges"), 12],
    [this.messageTypes.get("request-target-ranges"), 0],
    [this.messageTypes.get("acknowledge"), 0],
    [this.messageTypes.get("drone-log"), 31],
    [this.messageTypes.get("activate"), 0],
    [this.messageTypes.get("deactivate"), 0]
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
    } else if (byte == this.marker.start) {
      this.reading = true;
    }

    return { done: false };
  };

  readPIDInstruction = (data) => {
    const floatValues = new Float32Array(data.buffer);
    return { p: floatValues[0], i: floatValues[1], d: floatValues[2] };
  };

  readTargetRanges = (data) => {
    const floatValues = new Float32Array(data.buffer);
    return { maxPitch: floatValues[0], maxRoll: floatValues[1], maxVerticalVelocity: floatValues[2] };
  };
}
const serialReader = new SerialReader();

const portSettings = { baudRate: 9600 };
let port;
document.querySelector("#select-serial").addEventListener("click", async () => {
  port = await navigator.serial.requestPort();
  port.open(portSettings);
  read();
  setInterval(sendControllerInstructions, 20);
});

let hasAcknowledged = true;
write = async (data) => {
  if (!hasAcknowledged) {
    return false;
  }

  const writer = port.writable.getWriter();

  await writer.write(data);
  hasAcknowledged = false;

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
          reader.releaseLock();
          break;
        }
        if (value) {
          for (const byte of value) {
            const result = serialReader.read(byte);

            if (result.done) {
              console.log(result);

              if (result.messageType == "acknowledge") {
                hasAcknowledged = true;
              }

              // pid
              else if (result.messageType == "pid-velocity") {
                const pid = readPIDInstruction(result.value);
                document.getElementById("pid-v-p").value = pid.p;
                document.getElementById("pid-v-i").value = pid.i;
                document.getElementById("pid-v-d").value = pid.d;
              }
              else if (result.messageType == "pid-pitch") {
                const pid = readPIDInstruction(result.value);
                document.getElementById("pid-p-p").value = pid.p;
                document.getElementById("pid-p-i").value = pid.i;
                document.getElementById("pid-p-d").value = pid.d;
              }
              else if (result.messageType == "pid-roll") {
                const pid = readPIDInstruction(result.value);
                document.getElementById("pid-r-p").value = pid.p;
                document.getElementById("pid-r-i").value = pid.i;
                document.getElementById("pid-r-d").value = pid.d;
              }

              // target values
              else if (result.messageType == "target-ranges") {
                const ranges = readTargetRanges(result.value);
                document.getElementById("terget-ranges-velocity").value = ranges.maxVerticalVelocity;
                document.getElementById("terget-ranges-pitch").value = ranges.maxPitch;
                document.getElementById("terget-ranges-roll").value = ranges.maxRoll;
              }
            }
          }

          TextDecoder().decode(value).split("\n").forEach((str) => addToTerminal(str));
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

toInstruction = (values, messageType) => {
  out = new Uint8Array(values.length + 2);
  out[0] = 33;
  out[1] = messageType;
  for (let i = 0; i < values.length; i++) {
    out[i + 2] = values[i];
  }
  return out;
};

// -1 <= x, y, power <= 1
getControllerInstructions = (x, y, power) => {
  const values = new Int8Array(3);
  values[0] = (x * 127) & 0x0000ff;
  values[1] = (y * 127) & 0x0000ff;
  values[2] = (power * 127) & 0x0000ff;

  uint8 = new Uint8Array(SerialMessage.messageLengths.get("controller-instructions"));
  uint8[0] = values[0];
  uint8[1] = values[1];
  uint8[2] = values[2];

  const messageType = SerialMessage.messageTypes("controller-instructions");
  const out = toInstruction(uint8, messageType);

  return out;
};

getPIDInstructions = (p, i, d, module) => {
  const floatValues = new Float32Array(3);
  floatValues[0] = p;
  floatValues[1] = i;
  floatValues[2] = d;

  const byteValues = new Uint8Array(floatValues.buffer);

  const messageType = SerialMessage.messageTypes("pid-" + module);
  const out = toInstruction(byteValues, messageType);

  return out;
};

getTargetRangeInstructions = (maxPitch, maxRoll, maxVerticalVelocity) => {
  const floatValues = new Float32Array(3);
  floatValues[0] = maxPitch;
  floatValues[1] = maxRoll;
  floatValues[2] = maxVerticalVelocity;

  const byteValues = new Uint8Array(floatValues.buffer);

  const messageType = 7;

  const out = toInstruction(byteValues, messageType);

  return out;
};

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

document.getElementById("write-all").addEventListener("click", async () => {
  let instructions = getPIDInstructions(
    document.getElementById("pid-v-p").value,
    document.getElementById("pid-v-i").value,
    document.getElementById("pid-v-d").value,
    "velocity"
  );

  while (!write(instructions));

  instructions = getPIDInstructions(
    document.getElementById("pid-p-p").value,
    document.getElementById("pid-p-i").value,
    document.getElementById("pid-p-d").value,
    "pitch"
  );

  while (!write(instructions));

  instructions = getPIDInstructions(
    document.getElementById("pid-r-p").value,
    document.getElementById("pid-r-i").value,
    document.getElementById("pid-r-d").value,
    "roll"
  );

  while (!write(instructions));
});

sendControllerInstructions = () => {
  instructions = getControllerInstructions(
    joystick.x,
    joystick.y,
    document.getElementById("power").value
  );
  write(instructions);
};

let previousLog = "";
addToTerminal = (log) => {
  const output = document.getElementById("output");
  if (log == previousLog) {
    output.children[0].children[0].innerText = parseInt(output.children[0].children[0].innerText) + 1;
    return;
  }

  output.innerHTML = `<div><span>1</span><span> | </span>${log}</div>` + output.innerHTML;
  previousLog = log;
  if (output.children.length > 40) {
    output.removeChild(output.lastChild);
  }
}