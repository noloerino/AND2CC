const readline = require("readline");
const process = require("process");
const noble = require("@abandonware/noble");

const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout,
  prompt: "ddd> "
});

const SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70";
const CHAR_UUID = "32e6108A-2b22-4db5-a914-43ce41986c70";
const NAME = "EE 149 | DDD";

let led_status = false; // true = on
let l_drive = 0;
let r_drive = 0;

const SPEED = 50;

function callback(peripheral) {
    if (peripheral.advertisement.localName !== NAME) {
        console.log("Declined to connect to", peripheral.advertisement.localName,
            "(id", peripheral.id, ")");
        return;
    }
    peripheral.connect()
    console.log("Connecting to", NAME);
    peripheral.discoverServices([SERVICE_UUID], (_e, services) => {
        console.log("Found service");
        services[0].discoverCharacteristics([CHAR_UUID], (_e, characteristics) => {
            let characteristic = characteristics[0];
            console.log("Found characteristic");
            characteristic.read((_e, data) => {
                console.log("Initial value:", data);
                led_status = !isNan(data[0]) ? parseInt(data[0]) > 0 : false;
                l_drive = !isNan(data[1]) ? parseInt(data[1]) : 0;
                r_drive = !isNan(data[2]) ? parseInt(data[2]) : 0;
            });
            rl.prompt();
            rl.on("line", (line) => {
                switch (line.trim()) {
                    case "on":
                        led_status = true;
                        break;
                    case "off":
                        led_status = false;
                        break;
                    case "l":
                        l_drive = SPEED;
                        r_drive = -SPEED;
                        break;
                    case "r":
                        l_drive = -SPEED;
                        r_drive = SPEED;
                        break;
                    case "f":
                        l_drive = SPEED;
                        r_drive = SPEED;
                        break;
                    case "b":
                        l_drive = -SPEED;
                        r_drive = -SPEED;
                        break;
                    default:
                        console.log("Invalid command:", line);
                        break;
                }
                let buf = [led_status, l_drive, r_drive];
                console.log("Writing new values:", buf);
                characteristics.write(buf);
            }).on("close", () => {
                console.log("Exiting");
                peripheral.disconnect();
                process.exit();
            });
        });
    });
}

noble.on("scanStart", () => console.log("Starting scan..."));
noble.on("discover", callback);
noble.startScanning([SERVICE_UUID]);

