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
    dpad_up: false,
    dpad_down: false,
    dpad_left: false,
    dpad_right: false
};

document.addEventListener("DOMContentLoaded", function () {
    document.getElementById("connect_ble").addEventListener("click", connect_ble);
    document.getElementById("disconnect_ble").addEventListener("click", disconnect_ble);
    
    // Setup virtual button event handlers
    setupVirtualButton("button_a", "a");
    setupVirtualButton("button_b", "b");
    setupVirtualButton("button_x", "x");
    setupVirtualButton("button_y", "y");
    setupVirtualButton("button_l", "l");
    setupVirtualButton("button_r", "r");
    setupVirtualButton("button_zl", "zl");
    setupVirtualButton("button_zr", "zr");
    setupVirtualButton("button_minus", "minus");
    setupVirtualButton("button_plus", "plus");
    setupVirtualButton("button_home", "home");
    setupVirtualButton("dpad_up", "dpad_up");
    setupVirtualButton("dpad_down", "dpad_down");
    setupVirtualButton("dpad_left", "dpad_left");
    setupVirtualButton("dpad_right", "dpad_right");
    
    output = document.getElementById("output");
    setInterval(loop, 8);
});

function setupVirtualButton(elementId, buttonKey) {
    const button = document.getElementById(elementId);
    if (button) {
        button.addEventListener("click", function(e) {
            e.preventDefault();
            virtualButtons[buttonKey] = !virtualButtons[buttonKey];
            
            if (virtualButtons[buttonKey]) {
                button.classList.add("pressed");
            } else {
                button.classList.remove("pressed");
            }
        });
    }
}

// Check if Web Bluetooth is supported
if (!navigator.bluetooth) {
    document.body.innerHTML = '<h1>Web Bluetooth API not supported</h1><p>Please use Chrome/Edge with HTTPS or enable experimental features.</p>';
}

let device = null;
let server = null;
let service = null;
let txCharacteristic = null;
let rxCharacteristic = null;
let prev_report = new Uint8Array([128, 128, 128, 128, 0, 0]); // Updated to 6 bytes: neutral axes + no buttons
let output;
let keepAliveCounter = 0;
let connectionLostDetected = false;

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
        
        // Add disconnection event listener
        device.addEventListener('gattserverdisconnected', onDisconnected);
        
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
        
        // Reset connection state
        connectionLostDetected = false;
        keepAliveCounter = 0;
        
        write("BLE connection established!\n");
        
        // Send initial data immediately to establish communication
        write("Sending initial gamepad data...\n");
        const initialReport = new Uint8Array([0, 0, 15, 128, 128, 128, 128, 0]); // Neutral state
        await send_report(initialReport);
        write("Initial data sent successfully!\n\n");
        
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

function onDisconnected() {
    write("Device disconnected unexpectedly!\n");
    connectionLostDetected = true;
    
    // Clean up connection state
    server = null;
    service = null;
    txCharacteristic = null;
    rxCharacteristic = null;
    
    // Update UI
    document.getElementById("connect_ble").style.display = "inline";
    document.getElementById("disconnect_ble").style.display = "none";
    
    // Auto-reconnect after 2 seconds
    setTimeout(() => {
        if (device && connectionLostDetected) {
            write("Attempting to reconnect...\n");
            connect_ble();
        }
    }, 2000);
}

async function disconnect_ble() {
    try {
        connectionLostDetected = false; // Prevent auto-reconnect
        
        if (rxCharacteristic) {
            await rxCharacteristic.stopNotifications();
            rxCharacteristic.removeEventListener('characteristicvaluechanged', handleNotification);
        }
        if (device) {
            device.removeEventListener('gattserverdisconnected', onDisconnected);
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
    if (!server || !server.connected || !txCharacteristic || connectionLostDetected) {
        return;
    }

    try {
        let data = new Uint8Array(4 + 6 + 4);  // Changed from 8 to 6 bytes for report
        data[0] = 1;    // protocol_version
        data[1] = 0;    // descriptor_number (use virtual gamepad descriptor)
        data[2] = 6;    // length (changed from 8 to 6)
        data[3] = 1;    // report_id (changed from 0 to 1 to match virtual gamepad)
        data.set(report, 4);
        const crc = crc32(new DataView(data.buffer), 10);  // Changed from 12 to 10
        data[10] = (crc >> 0) & 0xFF;   // Changed indices
        data[11] = (crc >> 8) & 0xFF;
        data[12] = (crc >> 16) & 0xFF;
        data[13] = (crc >> 24) & 0xFF;

        ble_write(END);
        for (let i = 0; i < 14; i++) {  // Changed from 16 to 14
            send_escaped_byte(data[i]);
        }
        ble_write(END);
        await flush();
    } catch (error) {
        write(`Send error: ${error.message}\n`);
        console.error('BLE send error:', error);
        // Connection might be lost
        if (error.name === 'NetworkError' || error.name === 'NotConnectedError') {
            connectionLostDetected = true;
        }
    }
}

async function loop() {
    try {
        clear_output();
        if (server && server.connected && !connectionLostDetected) {
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
        let lx = 128;  // Center stick positions
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
                b |= gamepad.buttons[0].value;
                a |= gamepad.buttons[1].value;
                y |= gamepad.buttons[2].value;
                x |= gamepad.buttons[3].value;
                l |= gamepad.buttons[4].value;
                r |= gamepad.buttons[5].value;
                zl |= gamepad.buttons[6].value > 0.25;
                zr |= gamepad.buttons[7].value > 0.25;
                minus |= gamepad.buttons[8].value;
                plus |= gamepad.buttons[9].value;
                ls |= gamepad.buttons[10].value;
                rs |= gamepad.buttons[11].value;
                home |= gamepad.buttons[16].value;
                dpad_up |= gamepad.buttons[12].value;
                dpad_down |= gamepad.buttons[13].value;
                dpad_left |= gamepad.buttons[14].value;
                dpad_right |= gamepad.buttons[15].value;
                
                // Fix stick calculation - don't accumulate, convert from -1.0..1.0 to 0..255
                lx = Math.max(0, Math.min(255, Math.round(128 + gamepad.axes[0] * 127)));
                ly = Math.max(0, Math.min(255, Math.round(128 + gamepad.axes[1] * 127)));
                rx = Math.max(0, Math.min(255, Math.round(128 + gamepad.axes[2] * 127)));
                ry = Math.max(0, Math.min(255, Math.round(128 + gamepad.axes[3] * 127)));
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
        dpad_up |= virtualButtons.dpad_up;
        dpad_down |= virtualButtons.dpad_down;
        dpad_left |= virtualButtons.dpad_left;
        dpad_right |= virtualButtons.dpad_right;
        
        // Show virtual button status
        let virtualPressed = [];
        for (const [key, value] of Object.entries(virtualButtons)) {
            if (value) virtualPressed.push(key.toUpperCase());
        }
        if (virtualPressed.length > 0) {
            write(`VIRTUAL: ${virtualPressed.join(', ')} PRESSED\n`);
        }

        let report = new Uint8Array(6);  // Changed from 8 to 6 bytes

        // Virtual gamepad format:
        // Byte 0: X axis (left stick X)
        // Byte 1: Y axis (left stick Y) 
        // Byte 2: Z axis (right stick X)
        // Byte 3: Rz axis (right stick Y)
        // Byte 4-5: 16 buttons packed into 2 bytes

        report[0] = lx;   // X axis
        report[1] = ly;   // Y axis  
        report[2] = rx;   // Z axis
        report[3] = ry;   // Rz axis

        // Pack 16 buttons into 2 bytes
        // Buttons 0-7 in byte 4, buttons 8-15 in byte 5
        let buttons_low = (a ? 1 : 0) | (b ? 2 : 0) | (x ? 4 : 0) | (y ? 8 : 0) | 
                         (l ? 16 : 0) | (r ? 32 : 0) | (zl ? 64 : 0) | (zr ? 128 : 0);
        
        let buttons_high = (minus ? 1 : 0) | (plus ? 2 : 0) | (ls ? 4 : 0) | (rs ? 8 : 0) | 
                          (home ? 16 : 0) | (capture ? 32 : 0) |
                          (dpad_up ? 64 : 0) | (dpad_down ? 128 : 0);
        
        // Handle d-pad left/right by combining with other buttons or using different mapping
        // Since we only have 16 buttons total, map dpad_left to ls and dpad_right to rs if not already used
        if (dpad_left && !ls) buttons_high |= 4;   // Use ls bit if not already pressed
        if (dpad_right && !rs) buttons_high |= 8;  // Use rs bit if not already pressed
        
        report[4] = buttons_low;
        report[5] = buttons_high;

        write("OUTPUT\n");
        for (let i = 0; i < 6; i++) {  // Changed from 8 to 6
            write(report[i].toString(16).padStart(2, '0'));
            write(" ");
        }
        write("\n");

        // Send data regularly to keep connection alive
        // Send every 3 seconds (375 loops × 8ms) to stay within the 4-second receiver timeout
        keepAliveCounter++;
        const shouldSendKeepAlive = keepAliveCounter >= 375; // ~3 seconds
        
        if (!reports_equal(prev_report, report) || shouldSendKeepAlive) {
            await send_report(report);
            prev_report = new Uint8Array(report); // Create a copy
            
            if (shouldSendKeepAlive) {
                write("Keep-alive data sent (3s interval)\n");
                keepAliveCounter = 0;
            }
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