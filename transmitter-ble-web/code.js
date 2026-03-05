import crc32 from './crc.js';

const dpad_lut = [15, 6, 2, 15, 0, 7, 1, 0, 4, 5, 3, 4, 15, 6, 2, 15];

// BLE Service and Characteristic UUIDs
// Using Nordic UART Service UUIDs as they're commonly supported
const UART_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const UART_TX_CHARACTERISTIC_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // Write
const UART_RX_CHARACTERISTIC_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // Notify

const END = 0o300;     /* indicates end of packet */
const ESC = 0o333;     /* indicates byte stuffing */
const ESC_END = 0o334; /* ESC ESC_END means END data byte */
const ESC_ESC = 0o335; /* ESC ESC_ESC means ESC data byte */

// Virtual button states
let virtualButtons = {
    a: false,
    b: false,
    x: false,
    y: false,
    l: false,
    r: false,
    zl: false,
    zr: false,
    minus: false,
    plus: false,
    home: false,
    ls: false,
    rs: false,
    dpad_up: false,
    dpad_down: false,
    dpad_left: false,
    dpad_right: false
};

// Virtual analog stick states
let virtualSticks = {
    lx: 128,
    ly: 128,
    rx: 128,
    ry: 128
};

document.addEventListener("DOMContentLoaded", function () {
    document.getElementById("connect_ble").addEventListener("click", connect_ble);
    document.getElementById("disconnect_ble").addEventListener("click", disconnect_ble);
    
    // Add event handlers for all virtual buttons
    const buttonMappings = {
        'button_a': 'a',
        'button_b': 'b', 
        'button_x': 'x',
        'button_y': 'y',
        'button_l': 'l',
        'button_r': 'r',
        'button_zl': 'zl',
        'button_zr': 'zr',
        'button_minus': 'minus',
        'button_plus': 'plus',
        'button_home': 'home',
        'button_ls': 'ls',
        'button_rs': 'rs',
        'dpad_up': 'dpad_up',
        'dpad_down': 'dpad_down',
        'dpad_left': 'dpad_left',
        'dpad_right': 'dpad_right'
    };
    
    // Set up event handlers for each button
    for (const [elementId, buttonKey] of Object.entries(buttonMappings)) {
        const button = document.getElementById(elementId);
        if (button) {
            button.addEventListener("click", function(e) {
                e.preventDefault();
                virtualButtons[buttonKey] = !virtualButtons[buttonKey]; // Toggle the state
                
                if (virtualButtons[buttonKey]) {
                    button.classList.add("pressed");
                } else {
                    button.classList.remove("pressed");
                }
            });
        }
    }
    
    // Add event handlers for analog stick sliders
    const stickMappings = {
        'left_stick_x': 'lx',
        'left_stick_y': 'ly',
        'right_stick_x': 'rx',
        'right_stick_y': 'ry'
    };
    
    for (const [elementId, stickKey] of Object.entries(stickMappings)) {
        const slider = document.getElementById(elementId);
        if (slider) {
            slider.addEventListener("input", function(e) {
                virtualSticks[stickKey] = parseInt(e.target.value);
            });
        }
    }
    
    output = document.getElementById("output");
    setInterval(loop, 8);
});

// Check if Web Bluetooth is supported
if (!navigator.bluetooth) {
    document.body.innerHTML = '<h1>Web Bluetooth API not supported</h1><p>Please use Chrome/Edge with HTTPS or enable experimental features.</p>';
}

let device = null;
let server = null;
let service = null;
let txCharacteristic = null;
let rxCharacteristic = null;
let prev_report = new Uint8Array([0, 0, 15, 0, 0, 0, 0, 0, 0]);
let output;

async function connect_ble() {
    try {
        write("Searching for BLE devices...\n");
        
        // Request a device with more flexible filtering
        // This allows connecting to devices like "playAbility" that might not advertise the service UUID
        device = await navigator.bluetooth.requestDevice({
            // Accept all devices but prefer those with UART service or specific names
            acceptAllDevices: true,
            optionalServices: [UART_SERVICE_UUID]
        });

        write(`Selected device: ${device.name || 'Unknown'}\n`);
        
        // Connect to GATT server
        server = await device.gatt.connect();
        write("Connected to GATT server\n");
        
        // Try to get the UART service
        try {
            service = await server.getPrimaryService(UART_SERVICE_UUID);
            write("Found UART service\n");
        } catch (serviceError) {
            write(`UART service not found: ${serviceError.message}\n`);
            throw new Error("Device doesn't have Nordic UART Service. Please select a compatible device.");
        }
        
        // Get the TX characteristic (for writing data to device)
        try {
            txCharacteristic = await service.getCharacteristic(UART_TX_CHARACTERISTIC_UUID);
            write("Got TX characteristic\n");
        } catch (txError) {
            write(`TX characteristic error: ${txError.message}\n`);
            throw new Error("Cannot find TX characteristic");
        }
        
        // Get the RX characteristic (for reading data from device)
        try {
            rxCharacteristic = await service.getCharacteristic(UART_RX_CHARACTERISTIC_UUID);
            await rxCharacteristic.startNotifications();
            rxCharacteristic.addEventListener('characteristicvaluechanged', handleNotification);
            write("Got RX characteristic with notifications\n");
        } catch (rxError) {
            write("RX characteristic not available (write-only mode)\n");
            // This is OK, the device might be write-only
        }
        
        // Update UI
        document.getElementById("connect_ble").style.display = "none";
        document.getElementById("disconnect_ble").style.display = "inline";
        
        write("BLE connection established!\n\n");
        
    } catch (error) {
        write(`Error: ${error.message}\n`);
        console.error('BLE connection error:', error);
        
        // Clean up on error
        if (server && server.connected) {
            server.disconnect();
        }
        device = null;
        server = null;
        service = null;
        txCharacteristic = null;
        rxCharacteristic = null;
    }
}

async function disconnect_ble() {
    try {
        if (rxCharacteristic) {
            await rxCharacteristic.stopNotifications();
            rxCharacteristic.removeEventListener('characteristicvaluechanged', handleNotification);
        }
        if (server && server.connected) {
            server.disconnect();
        }
        
        device = null;
        server = null;
        service = null;
        txCharacteristic = null;
        rxCharacteristic = null;
        
        // Update UI
        document.getElementById("connect_ble").style.display = "inline";
        document.getElementById("disconnect_ble").style.display = "none";
        
        write("Disconnected from BLE device\n\n");
        
    } catch (error) {
        write(`Disconnect error: ${error.message}\n`);
        console.error('BLE disconnect error:', error);
    }
}

function handleNotification(event) {
    const value = event.target.value;
    const decoder = new TextDecoder();
    const data = decoder.decode(value);
    write(`Received: ${data}\n`);
}

let transmit_buffer = [];

function ble_write(c) {
    transmit_buffer.push(c);
}

async function flush() {
    if (!txCharacteristic || transmit_buffer.length === 0) {
        return;
    }
    
    try {
        // BLE characteristics typically have a 20-byte MTU limit
        const MTU_SIZE = 20;
        const data = new Uint8Array(transmit_buffer);
        
        // Send data in chunks if it exceeds MTU
        for (let i = 0; i < data.length; i += MTU_SIZE) {
            const chunk = data.slice(i, i + MTU_SIZE);
            await txCharacteristic.writeValue(chunk);
        }
        
        transmit_buffer = [];
    } catch (error) {
        write(`Write error: ${error.message}\n`);
        console.error('BLE write error:', error);
    }
}

function send_escaped_byte(b) {
    switch (b) {
        case END:
            ble_write(ESC);
            ble_write(ESC_END);
            break;

        case ESC:
            ble_write(ESC);
            ble_write(ESC_ESC);
            break;

        default:
            ble_write(b);
    }
}

async function send_report(report) {
    if (!server || !server.connected || !txCharacteristic) {
        return;
    }

    let data = new Uint8Array(4 + 8 + 4);
    data[0] = 1;  // protocol version
    data[1] = 2;  // descriptor number
    data[2] = 8;  // length
    data[3] = 1;  // report_id (now 1 to match virtual gamepad descriptor)
    data.set(report, 4);
    const crc = crc32(new DataView(data.buffer), 12);
    data[12] = (crc >> 0) & 0xFF;
    data[13] = (crc >> 8) & 0xFF;
    data[14] = (crc >> 16) & 0xFF;
    data[15] = (crc >> 24) & 0xFF;

    ble_write(END);
    for (let i = 0; i < 16; i++) {
        send_escaped_byte(data[i]);
    }
    ble_write(END);
    await flush();
}

async function loop() {
    try {
        clear_output();
        if (server && server.connected) {
            write(`BLE CONNECTED (${device.name || 'Unknown'})\n\n`);
        } else {
            write("BLE NOT CONNECTED\n\n");
        }

        let b = false;
        let a = false;
        let y = false;
        let x = false;
        let l = false;
        let r = false;
        let zl = false;
        let zr = false;
        let minus = false;
        let plus = false;
        let ls = false;
        let rs = false;
        let home = false;
        let capture = false;
        let dpad_left = false;
        let dpad_right = false;
        let dpad_up = false;
        let dpad_down = false;
        let lx = 128;
        let ly = 128;
        let rx = 128;
        let ry = 128;
        
        // Process physical gamepads
        for (const gamepad of navigator.getGamepads()) {
            if (!gamepad) {
                continue;
            }
            write(gamepad.id);
            write("\n");
            if ((gamepad.mapping == 'standard') && !gamepad.id.includes('HID Receiver')) {
                for (const b of gamepad.buttons) {
                    write(b.value);
                    write(" ");
                }
                for (const b of gamepad.axes) {
                    write(b);
                    write(" ");
                }
                write("\n");
                b |= gamepad.buttons[0].pressed;
                a |= gamepad.buttons[1].pressed;
                y |= gamepad.buttons[2].pressed;
                x |= gamepad.buttons[3].pressed;
                l |= gamepad.buttons[4].pressed;
                r |= gamepad.buttons[5].pressed;
                zl |= gamepad.buttons[6].value > 0.25;
                zr |= gamepad.buttons[7].value > 0.25;
                minus |= gamepad.buttons[8].pressed;
                plus |= gamepad.buttons[9].pressed;
                ls |= gamepad.buttons[10].pressed;
                rs |= gamepad.buttons[11].pressed;
                home |= gamepad.buttons[16].pressed;
                dpad_up |= gamepad.buttons[12].pressed;
                dpad_down |= gamepad.buttons[13].pressed;
                dpad_left |= gamepad.buttons[14].pressed;
                dpad_right |= gamepad.buttons[15].pressed;
                lx = Math.max(0, Math.min(255, 128 + gamepad.axes[0] * 128));
                ly = Math.max(0, Math.min(255, 128 + gamepad.axes[1] * 128));
                rx = Math.max(0, Math.min(255, 128 + gamepad.axes[2] * 128));
                ry = Math.max(0, Math.min(255, 128 + gamepad.axes[3] * 128));
            } else {
                write("IGNORED\n");
            }
            write("\n");
        }
        
        // Include virtual button states
        a |= virtualButtons.a;
        b |= virtualButtons.b;
        x |= virtualButtons.x;
        y |= virtualButtons.y;
        l |= virtualButtons.l;
        r |= virtualButtons.r;
        zl |= virtualButtons.zl;
        zr |= virtualButtons.zr;
        minus |= virtualButtons.minus;
        plus |= virtualButtons.plus;
        home |= virtualButtons.home;
        ls |= virtualButtons.ls;
        rs |= virtualButtons.rs;
        dpad_up |= virtualButtons.dpad_up;
        dpad_down |= virtualButtons.dpad_down;
        dpad_left |= virtualButtons.dpad_left;
        dpad_right |= virtualButtons.dpad_right;
        
        // Include virtual analog stick values (only if no physical gamepad is moving them)
        if (navigator.getGamepads().every(gamepad => !gamepad || gamepad.mapping !== 'standard' || gamepad.id.includes('HID Receiver'))) {
            lx = virtualSticks.lx;
            ly = virtualSticks.ly;
            rx = virtualSticks.rx;
            ry = virtualSticks.ry;
        }
        
        // Show virtual button status
        if (virtualButtons.a) {
            write("VIRTUAL: Button A PRESSED\n");
        }
        if (virtualButtons.b) {
            write("VIRTUAL: Button B PRESSED\n");
        }
        if (virtualButtons.x) {
            write("VIRTUAL: Button X PRESSED\n");
        }
        if (virtualButtons.y) {
            write("VIRTUAL: Button Y PRESSED\n");
        }
        if (virtualButtons.l) {
            write("VIRTUAL: Button L PRESSED\n");
        }
        if (virtualButtons.r) {
            write("VIRTUAL: Button R PRESSED\n");
        }
        if (virtualButtons.zl) {
            write("VIRTUAL: Button ZL PRESSED\n");
        }
        if (virtualButtons.zr) {
            write("VIRTUAL: Button ZR PRESSED\n");
        }
        if (virtualButtons.minus) {
            write("VIRTUAL: Button MINUS PRESSED\n");
        }
        if (virtualButtons.plus) {
            write("VIRTUAL: Button PLUS PRESSED\n");
        }
        if (virtualButtons.home) {
            write("VIRTUAL: Button HOME PRESSED\n");
        }
        if (virtualButtons.dpad_up) {
            write("VIRTUAL: D-Pad UP PRESSED\n");
        }
        if (virtualButtons.dpad_down) {
            write("VIRTUAL: D-Pad DOWN PRESSED\n");
        }
        if (virtualButtons.dpad_left) {
            write("VIRTUAL: D-Pad LEFT PRESSED\n");
        }
        if (virtualButtons.dpad_right) {
            write("VIRTUAL: D-Pad RIGHT PRESSED\n");
        }

        let report = new Uint8Array(8);

        report[0] = (y << 0) | (b << 1) | (a << 2) | (x << 3) | (l << 4) | (r << 5) | (zl << 6) | (zr << 7);
        report[1] = (minus << 0) | (plus << 1) | (ls << 2) | (rs << 3) | (home << 4) | (capture << 5);
        report[2] = dpad_lut[(dpad_left << 0) | (dpad_right << 1) | (dpad_up << 2) | (dpad_down << 3)];
        report[3] = Math.max(0, Math.min(255, lx));
        report[4] = Math.max(0, Math.min(255, ly));
        report[5] = Math.max(0, Math.min(255, rx));
        report[6] = Math.max(0, Math.min(255, ry));

        write("OUTPUT\n");
        for (let i = 0; i < 8; i++) {
            write(report[i].toString(16).padStart(2, '0'));
            write(" ");
        }
        write("\n");

        if (!reports_equal(prev_report, report)) {
            await send_report(report);
            prev_report = report;
        }
    } catch (e) {
        console.log(e);
    }
}

function write(s) {
    output.innerText += s;
}

function clear_output() {
    output.innerHTML = '';
}

function reports_equal(a, b) {
    if (a.length != b.length) {
        return false;
    }
    for (let i = 0; i < a.length; i++) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}