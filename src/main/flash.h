#pragma once

#include "rx.h"
#include "rx_bayang.h"
#include "rx_frsky.h"
#include "rx_unified_serial.h"

#define FLASH_STORAGE_OFFSET 4
#define FLASH_STORAGE_SIZE 32

typedef struct {
  float pid_identifier;
  float accelcal[3];

  uint8_t flash_feature_1; //SETUP WIZARD
  uint8_t lvc_lower_throttle;
} flash_storage_t;

extern flash_storage_t flash_storage;

#define BIND_STORAGE_OFFSET (FLASH_STORAGE_OFFSET + FLASH_STORAGE_SIZE)
#define BIND_STORAGE_SIZE 64
#define BIND_RAW_STORAGE_SIZE 63

typedef struct {
  uint8_t bind_saved;
  union {
    rx_frsky_bind_data_t frsky;
    rx_bayang_bind_data_t bayang;
    rx_unified_bind_data_t unified;
    uint8_t raw[BIND_RAW_STORAGE_SIZE];
  };
} rx_bind_storage_t;

extern rx_bind_storage_t bind_storage;

cbor_result_t cbor_encode_rx_bind_storage_t(cbor_value_t *enc, const rx_bind_storage_t *s);

#define PROFILE_STORAGE_OFFSET (BIND_STORAGE_OFFSET + BIND_STORAGE_SIZE)
#define PROFILE_STORAGE_SIZE 2048

void flash_save(void);
void flash_load(void);