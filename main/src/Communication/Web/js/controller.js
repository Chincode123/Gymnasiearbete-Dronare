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
