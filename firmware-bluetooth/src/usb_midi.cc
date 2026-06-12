#include "usb_midi.h"

#include <stdint.h>
#include "remapper.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/usb_audio.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(usb_midi, LOG_LEVEL_DBG);

#define USB_MIDI_BULK_EP_MPS 64
#define USB_DESC_CS_INTERFACE 0x24
#define USB_DESC_CS_ENDPOINT 0x25
#define USB_AUDIO_MS_HEADER 0x01
#define USB_AUDIO_MS_IN_JACK 0x02
#define USB_AUDIO_MS_OUT_JACK 0x03
#define USB_AUDIO_MS_ENDPOINT 0x01

static uint8_t midi_bulk_out_buf[USB_MIDI_BULK_EP_MPS];

static void usb_midi_process_packet(const uint8_t* data, uint32_t len) {
    for (uint32_t offset = 0; offset + 3 < len; offset += 4) {
        uint8_t cin = data[offset] & 0x0F;
        uint8_t msg[4] = { data[offset], data[offset + 1], data[offset + 2], data[offset + 3] };

        switch (cin) {
            case 0x08:
            case 0x09:
            case 0x0A:
            case 0x0B:
            case 0x0C:
            case 0x0D:
            case 0x0E:
                handle_received_midi(0, msg);
                break;
            case 0x04:
                msg[1] = data[offset + 1];
                msg[2] = data[offset + 2];
                msg[3] = 0;
                handle_received_midi(0, msg);
                break;
            case 0x05:
            case 0x06:
                msg[1] = data[offset + 1];
                msg[2] = data[offset + 2];
                msg[3] = 0;
                handle_received_midi(0, msg);
                break;
            case 0x07:
                msg[1] = data[offset + 1];
                msg[2] = 0;
                msg[3] = 0;
                handle_received_midi(0, msg);
                break;
            default:
                break;
        }
    }
}

static void midi_bulk_out(uint8_t ep, enum usb_dc_ep_cb_status_code ep_status) {
    uint32_t bytes_read = 0;

    if (ep_status != USB_DC_EP_DATA_OUT) {
        return;
    }

    if (usb_read(ep, midi_bulk_out_buf, sizeof(midi_bulk_out_buf), &bytes_read) != 0) {
        return;
    }

    if (bytes_read > 0) {
        usb_midi_process_packet(midi_bulk_out_buf, bytes_read);
    }

    usb_read(ep, midi_bulk_out_buf, sizeof(midi_bulk_out_buf), NULL);
}

static struct usb_ep_cfg_data midi_ep[] = {
    {
        .ep_cb = midi_bulk_out,
        .ep_addr = 0x03,
    },
};

static int midi_class_handler(struct usb_setup_packet* setup, int32_t* len, uint8_t** data) {
    ARG_UNUSED(setup);
    ARG_UNUSED(len);
    ARG_UNUSED(data);
    return -ENOTSUP;
}

static void midi_usb_status(struct usb_cfg_data* cfg, enum usb_dc_status_code cb_status, const uint8_t* param) {
    ARG_UNUSED(cfg);
    ARG_UNUSED(param);

    if (cb_status == USB_DC_CONFIGURED) {
        usb_read(midi_ep[0].ep_addr, midi_bulk_out_buf, sizeof(midi_bulk_out_buf), NULL);
    }
}

static struct {
    struct usb_if_descriptor if0;
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubtype;
        uint16_t bcdADC;
        uint16_t wTotalLength;
        uint8_t bInCollection;
        uint8_t baInterfaceNr;
    } if0_cs_header;
    struct usb_if_descriptor if1;
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubtype;
        uint16_t bcdADC;
        uint16_t wTotalLength;
    } if1_cs_header;
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubtype;
        uint8_t bJackType;
        uint8_t bJackID;
    } in_embedded;
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubtype;
        uint8_t bJackType;
        uint8_t bJackID;
        uint8_t iJack;
    } out_embedded;
    struct usb_ep_descriptor ep_out;
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubtype;
        uint16_t wTotalLength;
    } ep_out_cs;
} __packed midi_desc = {
    .if0 = {
        .bLength = sizeof(struct usb_if_descriptor),
        .bDescriptorType = USB_DESC_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 0,
        .bInterfaceClass = USB_BCC_AUDIO,
        .bInterfaceSubClass = USB_AUDIO_AUDIOCONTROL,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0,
    },
    .if0_cs_header = {
        .bLength = 9,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubtype = USB_AUDIO_HEADER,
        .bcdADC = sys_cpu_to_le16(0x0100),
        .wTotalLength = sys_cpu_to_le16(9),
        .bInCollection = 1,
        .baInterfaceNr = 1,
    },
    .if1 = {
        .bLength = sizeof(struct usb_if_descriptor),
        .bDescriptorType = USB_DESC_INTERFACE,
        .bInterfaceNumber = 1,
        .bAlternateSetting = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = USB_BCC_AUDIO,
        .bInterfaceSubClass = USB_AUDIO_MIDISTREAMING,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0,
    },
    .if1_cs_header = {
        .bLength = 7,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubtype = USB_AUDIO_MS_HEADER,
        .bcdADC = sys_cpu_to_le16(0x0100),
        .wTotalLength = sys_cpu_to_le16(7 + 6 + 6),
    },
    .in_embedded = {
        .bLength = 6,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubtype = USB_AUDIO_MS_IN_JACK,
        .bJackType = 0x01,
        .bJackID = 1,
    },
    .out_embedded = {
        .bLength = 6,
        .bDescriptorType = USB_DESC_CS_INTERFACE,
        .bDescriptorSubtype = USB_AUDIO_MS_OUT_JACK,
        .bJackType = 0x01,
        .bJackID = 1,
        .iJack = 0,
    },
    .ep_out = {
        .bLength = sizeof(struct usb_ep_descriptor),
        .bDescriptorType = USB_DESC_ENDPOINT,
        .bEndpointAddress = 0x03,
        .bmAttributes = USB_DC_EP_BULK,
        .wMaxPacketSize = sys_cpu_to_le16(USB_MIDI_BULK_EP_MPS),
        .bInterval = 0,
    },
    .ep_out_cs = {
        .bLength = 5,
        .bDescriptorType = USB_DESC_CS_ENDPOINT,
        .bDescriptorSubtype = USB_AUDIO_MS_ENDPOINT,
        .wTotalLength = sys_cpu_to_le16(5),
    },
};

USBD_DEFINE_CFG_DATA(midi_config) = {
    .interface_descriptor = (void*) &midi_desc,
    .cb_usb_status = midi_usb_status,
    .interface = {
        .class_handler = midi_class_handler,
    },
    .num_endpoints = ARRAY_SIZE(midi_ep),
    .endpoint = midi_ep,
};

void usb_midi_init(void) {
    LOG_INF("USB MIDI interface registered");
}
