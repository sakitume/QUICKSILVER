#include "io/usb_configurator.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "drv_usb.h"
#include "flight/control.h"
#include "io/msp.h"
#include "io/quic.h"
#include "profile.h"
#include "project.h"
#include "reset.h"
#include "util/util.h"

static uint8_t decode_buffer[USB_BUFFER_SIZE];

void usb_msp_send(uint8_t direction, uint8_t code, uint8_t *data, uint32_t len) {
  const uint8_t size = len + MSP_HEADER_LEN + 1;

  static uint8_t frame[64];

  frame[0] = '$';
  frame[1] = 'M';
  frame[2] = '>';
  frame[3] = len;
  frame[4] = code;

  for (uint8_t i = 0; i < len; i++) {
    frame[i + MSP_HEADER_LEN] = data[i];
  }

  uint8_t chksum = len;
  for (uint8_t i = 4; i < (size - 1); i++) {
    chksum ^= frame[i];
  }
  frame[len + MSP_HEADER_LEN] = chksum;

  usb_serial_write(frame, size);
}

static msp_t msp = {
    .buffer = decode_buffer,
    .buffer_size = USB_BUFFER_SIZE,
    .buffer_offset = 0,
    .send = usb_msp_send,
};

void usb_quic_send(uint8_t *data, uint32_t len, void *priv) {
  usb_serial_write(data, len);
}

static quic_t quic = {
    .send = usb_quic_send,
};

void usb_quic_logf(const char *fmt, ...) {
  const uint32_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);

  quic_send_str(&quic, QUIC_CMD_LOG, QUIC_FLAG_NONE, str);
}

// double promition in the following is intended
#pragma GCC diagnostic ignored "-Wdouble-promotion"
// This function will be where all usb send/receive coms live
void usb_configurator() {
  uint8_t magic = 0;
  if (usb_serial_read(&magic, 1) != 1) {
    return;
  }

  switch (magic) {
  case USB_MAGIC_REBOOT:
    //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
    system_reset_to_bootloader();
    break;
  case USB_MAGIC_SOFT_REBOOT:
    system_reset();
    break;
  case USB_MAGIC_MSP: {
    uint32_t decode_buffer_size = 0;
    decode_buffer[decode_buffer_size++] = magic;
    decode_buffer[decode_buffer_size++] = usb_serial_read_byte();

    while (true) {
      msp_status_t status = msp_process_serial(&msp, decode_buffer, decode_buffer_size);
      if (status == MSP_EOF) {
        decode_buffer[decode_buffer_size++] = usb_serial_read_byte();
      } else {
        break;
      }
    }
    break;
  }
  case USB_MAGIC_QUIC: {
    uint32_t decode_buffer_size = 0;
    decode_buffer[decode_buffer_size++] = magic;
    decode_buffer[decode_buffer_size++] = usb_serial_read_byte();

    while (true) {
      if (!quic_process(&quic, decode_buffer, decode_buffer_size)) {
        decode_buffer[decode_buffer_size++] = usb_serial_read_byte();
      } else {
        break;
      }
    }
    break;
  }
  }

  // this will block and handle all usb traffic while active
  reset_looptime();
}
#pragma GCC diagnostic pop