let using_keyboard = false;
let using_joystick = false;
const joystick = { x: 0, y: 0 };
const joystick_area = document.getElementById("joystick-area");
const joystick_marker = document.getElementById("joystick-marker");
const joystick_info_x = document.getElementById("joystick-x-info");
const joystick_info_y = document.getElementById("joystick-y-info");

function normalizeJoystick(joystick) {
  const joystick_magnitude = Math.sqrt(
    joystick.x * joystick.x + joystick.y * joystick.y
  );
  if (joystick_magnitude > 1) {
    joystick.x /= joystick_magnitude;
    joystick.y /= joystick_magnitude;
  }
}

document.addEventListener("mousemove", (event) => {
  if (!using_joystick) {
    return;
  }

  const joystick_area_size = joystick_area.getBoundingClientRect();
  const joystick_area_width = joystick_area_size.right - joystick_area_size.left;

  joystick.x =
    ((event.clientX - joystick_area_size.right) / joystick_area_width + 0.5) *
    2;
  joystick.y =
    ((event.clientY - joystick_area_size.top) / joystick_area_width - 0.5) * -2;

  joystick.x = Math.min(Math.max(joystick.x, -1), 1);
  joystick.y = Math.min(Math.max(joystick.y, -1), 1);

  normalizeJoystick(joystick);
});

document
  .getElementById("joystick-area")
  .addEventListener("mousedown", () => {
    using_joystick = true;
    using_keyboard = false;
  });

function resetMarker() {
  joystick.x = 0;
  joystick.y = 0;
}

document.addEventListener("mouseup", () => {
  if (!using_joystick) {
    return;
  }
  using_joystick = false;
  resetMarker();
});

function lerp(a, b, t) {
  const out = a + ((b - a) * t);
  const difference = Math.abs(out - b);
  const minDifference = 0.01;
  if (difference <= minDifference) {
    return b;
  }
  return out;
}

const keyboardAxes = {vertical: 0, horizontal: 0,};
function setKeyAxes(key) {
  if (key == "w" || key == "W") {
    keyboardAxes.vertical = 1;
  }
  else if (key == "a" || key == "A") {
    keyboardAxes.horizontal = -1;
  }
  else if (key == "s" || key == "S") {
    keyboardAxes.vertical = -1;
  }
  else if (key == "d" || key == "D") {
    keyboardAxes.horizontal = 1;
  }

  setJoystickFromKeyboard();
}

function unsetKeyAxes(key) {
  if ((key == "w" || key == "W") || (key == "s" || key == "S")) {
    keyboardAxes.vertical = 0;
  }
  else if ((key == "a" || key == "A") || (key == "d" || key == "D")) {
    keyboardAxes.horizontal = 0;
  }

  setJoystickFromKeyboard();
}

function setJoystickFromKeyboard() {
  joystick.x = keyboardAxes.horizontal;
  joystick.y = keyboardAxes.vertical;

  normalizeJoystick(joystick);
}

let targetPower = 0
document.addEventListener("keydown", (event) => {
  using_keyboard = true;
  
  if (event.key == " ") {
    targetPower = 1;
    return;
  }

  if (event.key == "Shift") {
    event.preventDefault();
    targetPower = -1;
    return;
  }

  setKeyAxes(event.key)
});

document.addEventListener("keyup", (event) => {
  if (event.key == " " || event.key == "Shift") {
    targetPower = 0;
    return;
  }

  unsetKeyAxes(event.key)
});

setInterval(() => {
  power = lerp(parseFloat(power), targetPower, 0.1);
});

let gamepad_index;
window.addEventListener("gamepadconnected", (event) => {
  gamepad_index = event.gamepad.index;
  setInterval(gamepadInputLoop);
});

function gamepadInputLoop () {
  const gamepad = navigator.getGamepads()[gamepad_index];

  gamepad.axes.forEach((axis) => {
    if (axis >= 0.1) {
      using_keyboard = false;
    }
  });

  gamepad.buttons.forEach((button) => {
    if (button.pressed) {
      using_keyboard = false;
    }
  });

  if (using_keyboard) {
    return;
  }

  joystick.x = gamepad.axes[0];
  joystick.y = -gamepad.axes[1];
  normalizeJoystick(joystick);

  targetPower = -gamepad.buttons[6].value + gamepad.buttons[7].value;
  targetPower *= Math.abs(targetPower);
};

let power = 0;
const power_info = document.getElementById("power-info");
const power_slider = document.getElementById("power");
power_slider.addEventListener("input", () => {
  targetPower = power_slider.value;
  using_keyboard = true;
});

function updateUI() {
  power_info.innerText = (parseFloat(power) * 100).toFixed() + "%";

  power_slider.value = power;

  joystick_info_x.innerText = joystick.x.toFixed(2);
  joystick_info_y.innerText = joystick.y.toFixed(2);

  const marker_position = {
    horizontal: lerp(parseFloat(joystick_marker.style.left), joystick.x * 50, 0.1),
    vertical: lerp(parseFloat(joystick_marker.style.top), joystick.y * -50, 0.1),
  }
  joystick_marker.style.left = `${marker_position.horizontal}%`;
  joystick_marker.style.top = `${marker_position.vertical}%`;
};

setInterval(updateUI);