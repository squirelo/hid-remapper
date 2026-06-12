import crc32 from './crc.js';

const NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';

const SLIP_END = 0xc0;
const SLIP_ESC = 0xdb;
const SLIP_ESC_END = 0xdc;
const SLIP_ESC_ESC = 0xdd;

let nusDevice;
let nusServer;
let rxCharacteristic;
let report = neutralReport();
let encryptedLinkReady = false;
let reportDirty = false;
let writeInFlight = false;

function neutralReport() {
    return new Uint8Array([0x00, 0x00, 0x0f, 0x80, 0x80, 0x80, 0x80, 0x00]);
}

function setStatus(message) {
    document.getElementById('nus_status').textContent = message;
}

function setConnectedState(connected) {
    document.getElementById('nus_connect').disabled = connected;
    document.getElementById('nus_connect_filtered').disabled = connected;
    document.getElementById('nus_disconnect').disabled = !connected;
    document.getElementById('nus_pair').disabled = !connected;
}

function formatWriteError(error) {
    const message = error.message || String(error);

    if (error.name === 'NetworkError' || /encrypt|security|auth|pair/i.test(message)) {
        return `${message}. The firmware requires an encrypted BLE link; accept the browser or OS pairing prompt, then try again.`;
    }

    if (error.name === 'NotAllowedError') {
        return `${message}. Bluetooth access or pairing was cancelled.`;
    }

    return message;
}

function framePacket(payload) {
    const packet = new Uint8Array(payload.length + 4);
    packet.set(payload);

    const view = new DataView(packet.buffer);
    const crc = crc32(new DataView(payload.buffer, payload.byteOffset, payload.byteLength), payload.length);
    view.setUint32(payload.length, crc, true);

    const framed = [SLIP_END];
    for (const byte of packet) {
        if (byte === SLIP_END) {
            framed.push(SLIP_ESC, SLIP_ESC_END);
        } else if (byte === SLIP_ESC) {
            framed.push(SLIP_ESC, SLIP_ESC_ESC);
        } else {
            framed.push(byte);
        }
    }
    framed.push(SLIP_END);
    return new Uint8Array(framed);
}

async function sendCurrentReport(requireResponse = false) {
    if (!rxCharacteristic) {
        setStatus('Not connected');
        return;
    }

    const payload = new Uint8Array(4 + report.length);
    payload[0] = 1;
    payload[1] = Number(document.getElementById('nus_descriptor').value);
    payload[2] = report.length;
    payload[3] = 1;
    payload.set(report, 4);

    const framed = framePacket(payload);
    const useResponse = requireResponse || !encryptedLinkReady;
    const t0 = performance.now();

    if (useResponse && rxCharacteristic.writeValueWithResponse) {
        await rxCharacteristic.writeValueWithResponse(framed);
        encryptedLinkReady = true;
    } else if (rxCharacteristic.writeValueWithoutResponse) {
        await rxCharacteristic.writeValueWithoutResponse(framed);
    } else if (rxCharacteristic.writeValue) {
        await rxCharacteristic.writeValue(framed);
    } else if (rxCharacteristic.writeValueWithResponse) {
        await rxCharacteristic.writeValueWithResponse(framed);
        encryptedLinkReady = true;
    }

    const elapsedMs = performance.now() - t0;
    const mode = useResponse ? 'encrypted write' : 'fast write';
    setStatus(`Sent ${report.length} byte report (${mode}, ${elapsedMs.toFixed(1)} ms) to ${nusDevice?.name || 'HID Remapper'}`);
}

async function flushReportWrites(requireResponse = false) {
    writeInFlight = true;
    try {
        while (reportDirty) {
            reportDirty = false;
            const useResponse = requireResponse || !encryptedLinkReady;
            await sendCurrentReport(useResponse);
            requireResponse = false;
        }
    } catch (error) {
        if (requireResponse || !encryptedLinkReady) {
            encryptedLinkReady = false;
        }
        setStatus(formatWriteError(error));
    } finally {
        writeInFlight = false;
        if (reportDirty) {
            await flushReportWrites(false);
        }
    }
}

function queueCurrentReport(requireResponse = false) {
    reportDirty = true;
    if (writeInFlight) {
        return Promise.resolve();
    }
    return flushReportWrites(requireResponse);
}

function onDisconnected() {
    rxCharacteristic = undefined;
    nusServer = undefined;
    encryptedLinkReady = false;
    reportDirty = false;
    writeInFlight = false;
    setConnectedState(false);
    setStatus('Disconnected');
}

async function connectWithOptions(options) {
    try {
        if (!navigator.bluetooth) {
            setStatus('Web Bluetooth is not available in this browser');
            return;
        }

        setStatus('Opening Bluetooth picker...');
        nusDevice = await navigator.bluetooth.requestDevice(options);
        setStatus(`Selected ${nusDevice.name || nusDevice.id}; connecting GATT...`);
        nusDevice.addEventListener('gattserverdisconnected', onDisconnected);

        nusServer = await nusDevice.gatt.connect();
        setStatus(`Connected to ${nusDevice.name || nusDevice.id}; opening NUS service...`);
        const service = await nusServer.getPrimaryService(NUS_SERVICE_UUID);
        setStatus(`NUS service found on ${nusDevice.name || nusDevice.id}; opening write characteristic...`);
        rxCharacteristic = await service.getCharacteristic(NUS_RX_UUID);

        encryptedLinkReady = false;
        setConnectedState(true);
        setStatus(`Connected to ${nusDevice.name || 'HID Remapper'}. Use Pair / test write to trigger encryption, then reports use fast writes.`);
    } catch (error) {
        setStatus(error.message || String(error));
    }
}

function resetSliders() {
    for (const slider of document.querySelectorAll('#nus_tester_card input[type="range"]')) {
        slider.value = 128;
    }
}

export function initNusTester() {
    const unavailableEl = document.getElementById('nus_tester_unavailable');
    const controlsEl = document.getElementById('nus_tester_controls');

    if (!navigator.bluetooth) {
        unavailableEl.classList.remove('d-none');
        controlsEl.classList.add('d-none');
        return;
    }

    document.getElementById('nus_connect').addEventListener('click', async () => {
        await connectWithOptions({
            acceptAllDevices: true,
            optionalServices: [NUS_SERVICE_UUID],
        });
    });

    document.getElementById('nus_connect_filtered').addEventListener('click', async () => {
        await connectWithOptions({
            filters: [{ services: [NUS_SERVICE_UUID] }],
            optionalServices: [NUS_SERVICE_UUID],
        });
    });

    document.getElementById('nus_disconnect').addEventListener('click', () => {
        if (nusDevice?.gatt?.connected) {
            nusDevice.gatt.disconnect();
        }
    });

    document.getElementById('nus_pair').addEventListener('click', async () => {
        report = neutralReport();
        encryptedLinkReady = false;
        await queueCurrentReport(true);
    });

    document.getElementById('nus_neutral').addEventListener('click', async () => {
        report = neutralReport();
        resetSliders();
        await queueCurrentReport();
    });

    for (const button of document.querySelectorAll('.nus-button')) {
        const bit = Number(button.dataset.button);
        button.addEventListener('click', async () => {
            report[0] |= 1 << bit;
            await queueCurrentReport();
            report[0] &= ~(1 << bit);
            await queueCurrentReport();
        });
    }

    for (const button of document.querySelectorAll('.nus-hat')) {
        button.addEventListener('click', async () => {
            report[2] = Number(button.dataset.hat);
            await queueCurrentReport();
        });
    }

    for (const [selector, index] of [
        ['#nus_axis_x', 3],
        ['#nus_axis_y', 4],
        ['#nus_axis_z', 5],
        ['#nus_axis_rz', 6],
    ]) {
        document.querySelector(selector).addEventListener('input', async (event) => {
            report[index] = Number(event.target.value);
            await queueCurrentReport();
        });
    }
}
