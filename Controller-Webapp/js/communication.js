class SerialMessage {
  initiated = false;
  type;
  length;

  messageTypeFromIndex = new Map([
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
    [12, "deactivate"],
    [13, "drone-velocity"],
    [14, "drone-acceleration"],
    [15, "drone-angular-velocity"],
    [16, "drone-angles"],
    [17, "drone-delta-time"],
    [18, "receiver-delta-time"]
  ]);

  messageTypeFromName = new Map([
    ["controller-instructions", 0],
    ["pid-velocity", 1],
    ["pid-pitch", 2],
    ["pid-roll", 3],
    ["request-pid-velocity", 4],
    ["request-pid-pitch", 5],
    ["request-pid-roll", 6],
    ["target-ranges", 7],
    ["request-target-ranges", 8],
    ["acknowledge", 9],
    ["drone-log", 10],
    ["activate", 11],
    ["deactivate", 12],
    ["drone-velocity", 13],
    ["drone-acceleration", 14],
    ["drone-angular-velocity", 15],
    ["drone-angles", 16],
    ["drone-delta-time", 17],
    ["receiver-delta-time", 18]
  ]);

  messageLengths = new Map([
    [this.messageTypeFromName.get("controller-instructions"), 3],
    [this.messageTypeFromName.get("pid-velocity"), 12],
    [this.messageTypeFromName.get("pid-pitch"), 12],
    [this.messageTypeFromName.get("pid-roll"), 12],
    [this.messageTypeFromName.get("request-pid-velocity"), 0],
    [this.messageTypeFromName.get("request-pid-pitch"), 0],
    [this.messageTypeFromName.get("request-pid-roll"), 0],
    [this.messageTypeFromName.get("target-ranges"), 12],
    [this.messageTypeFromName.get("request-target-ranges"), 0],
    [this.messageTypeFromName.get("acknowledge"), 0],
    [this.messageTypeFromName.get("drone-log"), 31],
    [this.messageTypeFromName.get("activate"), 0],
    [this.messageTypeFromName.get("deactivate"), 0],
    [this.messageTypeFromName.get("drone-velocity"), 12],
    [this.messageTypeFromName.get("drone-acceleration"), 12],
    [this.messageTypeFromName.get("drone-angular-velocity"), 12],
    [this.messageTypeFromName.get("drone-angles"), 12],
    [this.messageTypeFromName.get("drone-delta-time"), 4],
    [this.messageTypeFromName.get("receiver-delta-time"), 4]
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
        const messageType = this.message.messageTypeFromIndex.get(this.message.type);
        this.message.reset();
        const out = new Uint8Array(this.data);
        this.data = [];
        return {
          data: out,
          done: true,
          messageType: messageType,
          reading: this.reading
        };
      }
    } else if (byte == this.marker.start) {
      this.reading = true;
    }

    return { done: false, reading: this.reading };
  };

  readPIDInstruction = (data) => {
    const floatValues = new Float32Array(data.buffer);
    return { p: floatValues[0], i: floatValues[1], d: floatValues[2] };
  };

  readTargetRanges = (data) => {
    const floatValues = new Float32Array(data.buffer);
    return { maxPitch: floatValues[0], maxRoll: floatValues[1], maxVerticalVelocity: floatValues[2] };
  };

  readVector = (data) => {
    const floatValues = new Float32Array(data.buffer);
    return { x: floatValues[0], y: floatValues[1], z: floatValues[2] };
  };

  readDeltaTime = (data) =>  {
    const floatValues = new Float32Array(data.buffer);
    return floatValues[0];
  }
}
const serialReader = new SerialReader();


class Terminal {
  decoder = new TextDecoder();
  currentMessage = new Uint8Array(64);
  currentIndex = 0;
  
  previousLog = "";

  readByte = (byte) => {
    if (byte == 10) {
      const message = this.decoder.decode(this.currentMessage);
      this.addToTerminal(message);
      this.currentMessage = new Uint8Array(64);
      currentIndex = 0; 
      return
    }

    this.currentMessage[this.currentIndex++] = byte;
  }

  addToTerminal = (log) => {
    const output = document.getElementById("output");
    if (log == this.previousLog) {
      output.children[0].children[0].innerText = parseInt(output.children[0].children[0].innerText) + 1;
      return;
    }
  
    output.innerHTML = `<div><span>1</span> | ${log}</div>` + output.innerHTML;
    this.previousLog = log;
    if (output.children.length > 40) {
      output.removeChild(output.lastChild);
    }
  }
}
const terminal = new Terminal();


const portSettings = { baudRate: 115200 };
let port;
document.querySelector("#select-serial").addEventListener("click", async () => {
  port = await navigator.serial.requestPort();
  port.open(portSettings);
  read();
  setInterval(sendControllerInstructions, 20);
});

let hasAcknowledged = true;
let acknowledgeAttempts = 0;
let maxAcknowledgeAttempts = 5;
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

            if (!result.reading) {
              terminal.readByte(byte);
            }

            if (result.done) {
              console.log(result);

              if (!hasAcknowledged){
                acknowledgeAttempts++;
                if (acknowledgeAttempts >= maxAcknowledgeAttempts) {
                  hasAcknowledged = true;
                  acknowledgeAttempts = 0;
                }
              }

              if (result.messageType == "acknowledge") {
                hasAcknowledged = true;
              }

              // pid
              else if (result.messageType == "pid-velocity") {
                const pid = serialReader.readPIDInstruction(result.value);
                document.getElementById("pid-v-p").value = pid.p;
                document.getElementById("pid-v-i").value = pid.i;
                document.getElementById("pid-v-d").value = pid.d;
              }
              else if (result.messageType == "pid-pitch") {
                const pid = serialReader.readPIDInstruction(result.value);
                document.getElementById("pid-p-p").value = pid.p;
                document.getElementById("pid-p-i").value = pid.i;
                document.getElementById("pid-p-d").value = pid.d;
              }
              else if (result.messageType == "pid-roll") {
                const pid = serialReader.readPIDInstruction(result.value);
                document.getElementById("pid-r-p").value = pid.p;
                document.getElementById("pid-r-i").value = pid.i;
                document.getElementById("pid-r-d").value = pid.d;
              }

              // target values
              else if (result.messageType == "target-ranges") {
                const ranges = serialReader.readTargetRanges(result.value);
                document.getElementById("terget-ranges-velocity").value = ranges.maxVerticalVelocity;
                document.getElementById("terget-ranges-pitch").value = ranges.maxPitch;
                document.getElementById("terget-ranges-roll").value = ranges.maxRoll;
              }

              // drone info
              else if (result.messageType == "drone-velocity") {
                const velocity = serialReader.readVector(result.value);
                document.getElementById("vel-x").innerHTML = velocity.x.toFixed(2);
                document.getElementById("vel-y").innerHTML = velocity.y.toFixed(2);
                document.getElementById("vel-z").innerHTML = velocity.z.toFixed(2);
              }
              else if (result.messageType == "drone-acceleration") {
                const acceleration = serialReader.readVector(result.value);
                document.getElementById("acc-x").innerHTML = acceleration.x.toFixed(2);
                document.getElementById("acc-y").innerHTML = acceleration.y.toFixed(2);
                document.getElementById("acc-z").innerHTML = acceleration.z.toFixed(2);
              }
              else if (result.messageType == "drone-angular-velocity") {
                const angular_velocity = serialReader.readVector(result.value);
                document.getElementById("angular-vel-x").innerHTML = angular_velocity.x.toFixed(2);
                document.getElementById("angular-vel-y").innerHTML = angular_velocity.y.toFixed(2);
                document.getElementById("angular-vel-z").innerHTML = angular_velocity.z.toFixed(2);
              } 
              else if (result.messageType == "drone-angles") {
                const angles = serialReader.readVector(result.value);
                document.getElementById("pitch-value").innerHTML = angles.x.toFixed(2);
                document.getElementById("roll-value").innerHTML = angles.y.toFixed(2);
                document.getElementById("yaw-value").innerHTML = angles.z.toFixed(2);
              }
              else if(result.messageType == "drone-delta-time") {
                const updateRate = 1 / SerialReader.readDeltaTime(result.value);
                document.getElementById("drone-update").innerHTML = updateRate.toFixed(0);
              }
              else if(result.messageType == "receiver-delta-time") {
                const updateRate = 1 / SerialReader.readDeltaTime(result.value);
                document.getElementById("receiver-update").innerHTML = updateRate.toFixed(0);
              }
            }
          }

          new TextDecoder().decode(value).split("\n").forEach((str) => addToTerminal(str));
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

  const messageType = new SerialMessage().messageTypeFromName.get("controller-instructions");
  uint8 = new Uint8Array(new SerialMessage().messageLengths.get(messageType));
  uint8[0] = values[0];
  uint8[1] = values[1];
  uint8[2] = values[2];

  const out = toInstruction(uint8, messageType);

  return out;
};

getPIDInstructions = (p, i, d, module) => {
  const floatValues = new Float32Array(3);
  floatValues[0] = p;
  floatValues[1] = i;
  floatValues[2] = d;

  const byteValues = new Uint8Array(floatValues.buffer);

  const messageType = new SerialMessage().messageTypeFromName.get("pid-" + module);
  const out = toInstruction(byteValues, messageType);

  return out;
};

getTargetRangeInstructions = (maxPitch, maxRoll, maxVerticalVelocity) => {
  const floatValues = new Float32Array(3);
  floatValues[0] = maxPitch;
  floatValues[1] = maxRoll;
  floatValues[2] = maxVerticalVelocity;

  const byteValues = new Uint8Array(floatValues.buffer);

  const messageType = new SerialMessage().messageTypeFromName.get("target-ranges");

  const out = toInstruction(byteValues, messageType);

  return out;
};

getRequestInstruction = (type) => {
  const out = toInstruction(new Uint8Array(0), type);
  return out;
};

document
  .getElementById("velocity")
  .querySelector(".send")
  .addEventListener("click", async () => {
    const instructions = getPIDInstructions(
      document.getElementById("pid-v-p").value,
      document.getElementById("pid-v-i").value,
      document.getElementById("pid-v-d").value,
      "velocity"
    );

    while(!write(instructions));
  });

document
  .getElementById("pitch")
  .querySelector(".send")
  .addEventListener("click", async () => {
    const instructions = getPIDInstructions(
      document.getElementById("pid-p-p").value,
      document.getElementById("pid-p-i").value,
      document.getElementById("pid-p-d").value,
      "pitch"
    );

    while(!write(instructions));
  });

document
  .getElementById("roll")
  .querySelector(".send")
  .addEventListener("click", async () => {
    const instructions = getPIDInstructions(
      document.getElementById("pid-r-p").value,
      document.getElementById("pid-r-i").value,
      document.getElementById("pid-r-d").value,
      "roll"
    );

    while(!write(instructions));
  });

document
  .getElementById("target-ranges")
  .querySelector(".send")
  .addEventListener("click", async () => {
    const instructions = getTargetRangeInstructions(
      document.getElementById("arget-ranges-pitch").value,
      document.getElementById("target-ranges-roll").value,
      document.getElementById("target-ranges-velocity").value
    );

    while(!write(instructions));
  });

document
  .getElementById("velocity")
  .querySelector(".get")
  .addEventListener("click", async () => {
    const instructions = getRequestInstruction(new SerialMessage().messageTypeFromName.get("request-pid-velocity"));
    while(!write(instructions));
  });

document
  .getElementById("pitch")
  .querySelector(".get")
  .addEventListener("click", async () => {
    const instructions = getRequestInstruction(new SerialMessage().messageTypeFromName.get("request-pid-pitch"));
    while(!write(instructions));
  });

document
  .getElementById("roll")
  .querySelector(".get")
  .addEventListener("click", async () => {
    const instructions = getRequestInstruction(new SerialMessage().messageTypeFromName.get("request-pid-roll"));
    while(!write(instructions));
  });

document
  .getElementById("target-ranges")
  .querySelector(".get")
  .addEventListener("click", async () => {
    const instructions = getRequestInstruction(new SerialMessage().messageTypeFromName.get("request-target-ranges"));
    while(!write(instructions));
  });

document
  .getElementById("on")
  .addEventListener("click", async () => {
    const instructions = getRequestInstruction(new SerialMessage().messageTypeFromName.get("activate"));
    while(!write(instructions));
  });

document
  .getElementById("off")
  .addEventListener("click", async () => {
    const instructions = getRequestInstruction(new SerialMessage().messageTypeFromName.get("deactivate"));
    while(!write(instructions));
  });

sendControllerInstructions = () => {
  instructions = getControllerInstructions(
    joystick.x,
    joystick.y,
    document.getElementById("power").value
  );
  write(instructions);
};