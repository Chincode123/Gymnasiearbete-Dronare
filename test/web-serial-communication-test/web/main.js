let port;

document.querySelector("button").addEventListener('click', async () => {
    port = await navigator.serial.requestPort();
    console.log(port);
});
