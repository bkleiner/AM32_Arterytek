/*
 * dshot.c
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

#include "dshot.h"
#include "common.h"
#include "functions.h"
#include "main.h"
#include "targets.h"

int dpulse[16] = {0};

static const char gcr_encode_table[16] = {
    0b11001,
    0b11011,
    0b10010,
    0b10011,
    0b11101,
    0b10101,
    0b10110,
    0b10111,
    0b11010,
    0b01001,
    0b01010,
    0b01011,
    0b11110,
    0b01101,
    0b01110,
    0b01111,
};

char EDT_ARM_ENABLE = 0;
char EDT_ARMED = 0;

extern uint32_t e_com_time;
extern int zero_crosses;
extern char send_telemetry;
extern int smoothedinput;
extern uint8_t max_duty_cycle_change;
extern char play_tone_flag;
uint8_t command_count = 0;
uint8_t last_command = 0;
uint8_t high_pin_count = 0;
uint32_t gcr[GCR_BUFFER_SIZE] = {0};
uint16_t dshot_frametime;
uint16_t dshot_goodcounts;
uint16_t dshot_badcounts;
char dshot_extended_telemetry = 0;
uint16_t send_extended_dshot = 0;
uint16_t halfpulsetime = 0;

void computeDshotDMA() {

  int j = 0;
  dshot_frametime = dma_buffer[31] - dma_buffer[0];
  ////UTILITY_TIMER->c1dt = dshot_frametime;
  halfpulsetime = (dshot_frametime >> 5) + (dshot_frametime >> 8);
  if ((dshot_frametime < 500) && (dshot_frametime > 300)) {
    for (int i = 0; i < 16; i++) {
      dpulse[i] = ((dma_buffer[j + (i << 1) + 1] - dma_buffer[j + (i << 1)]) > (halfpulsetime));
    }

    uint8_t calcCRC = ((dpulse[0] ^ dpulse[4] ^ dpulse[8]) << 3 | (dpulse[1] ^ dpulse[5] ^ dpulse[9]) << 2 | (dpulse[2] ^ dpulse[6] ^ dpulse[10]) << 1 | (dpulse[3] ^ dpulse[7] ^ dpulse[11]));
    uint8_t checkCRC = (dpulse[12] << 3 | dpulse[13] << 2 | dpulse[14] << 1 | dpulse[15]);

    if (!armed) {
      if (dshot_telemetry == 0) {
        if ((INPUT_PIN_PORT->idt & INPUT_PIN)) { // if the pin is high for 100 checks between signal pulses its inverted
          high_pin_count++;
          if (high_pin_count > 100) {
            dshot_telemetry = 1;
          }
        }
      }
    }
    if (dshot_telemetry) {
      checkCRC = ~checkCRC + 16;
    }

    int tocheck = (dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8 | dpulse[3] << 7 | dpulse[4] << 6 | dpulse[5] << 5 | dpulse[6] << 4 | dpulse[7] << 3 | dpulse[8] << 2 | dpulse[9] << 1 | dpulse[10]);

    if (calcCRC == checkCRC) {
      signaltimeout = 0;
      dshot_goodcounts++;
      if (dpulse[11] == 1) {
        send_telemetry = 1;
      }
      if (tocheck > 47) {
        if (EDT_ARMED) {
          newinput = tocheck;
          dshotcommand = 0;
          command_count = 0;
          return;
        }
      }

      if ((tocheck <= 47) && (tocheck > 0)) {
        newinput = 0;
        dshotcommand = tocheck; //  todo
      }
      if (tocheck == 0) {
        if (EDT_ARM_ENABLE == 1) {
          EDT_ARMED = 0;
        }
        newinput = 0;
        dshotcommand = 0;
        command_count = 0;
      }

      if ((dshotcommand > 0) && (running == 0) && armed) {
        if (dshotcommand != last_command) {
          last_command = dshotcommand;
          command_count = 0;
        }
        if (dshotcommand < 5) { // beacons
          command_count = 6;    // go on right away
        }
        command_count++;
        if (command_count >= 6) {
          command_count = 0;
          switch (dshotcommand) { // todo

          case 1:
            playInputTune();
            break;
          case 2:
            playInputTune2();
            break;
          case 3:
            playBeaconTune3();
            break;
          case 7:
            dir_reversed = 0;
            forward = 1 - dir_reversed;
            play_tone_flag = 1;
            break;
          case 8:
            dir_reversed = 1;
            forward = 1 - dir_reversed;
            play_tone_flag = 2;
            break;
          case 9:
            bi_direction = 0;
            armed = 0;
            zero_input_count = 0;
            break;
          case 10:
            bi_direction = 1;
            zero_input_count = 0;
            armed = 0;
            break;
          case 12:
            saveEEpromSettings();
            // delayMillis(100);
            //	NVIC_SystemReset();
            break;
          case 13:
            dshot_extended_telemetry = 1;
            send_extended_dshot = 0b111000000000;
            if (EDT_ARM_ENABLE == 1) {
              EDT_ARMED = 1;
            }
            break;
          case 14:
            dshot_extended_telemetry = 0;
            send_extended_dshot = 0b111011111111;
            //		make_dshot_package();
            break;
          case 20:
            forward = 1 - dir_reversed;
            break;
          case 21:
            forward = dir_reversed;
            break;
          }
          last_dshot_command = dshotcommand;
          dshotcommand = 0;
        }
      }
    } else {
      dshot_badcounts++;
    }
  } else {
    dshot_badcounts++;
  }
  //	UTILITY_TIMER->c1dt = dshot_badcounts;
}

void make_dshot_package() {
  uint32_t telemetry_value;

  if (send_extended_dshot > 0) {
    telemetry_value = send_extended_dshot;
    send_extended_dshot = 0;
  } else {
    volatile uint16_t eprm = 0;
    if (!running || e_com_time > 65408) {
      eprm = 65408;
    } else {
      eprm = e_com_time;
    }

    // calculate shift amount for data in format eee mmm mmm mmm, first 1 found in first seven bits of data determines shift amount
    // this allows for a range of up to 65408 microseconds which would be shifted 0b111 (eee) or 7 times.
    uint32_t shift_amount = 0;
    for (uint32_t i = 15; i >= 9; i--) {
      if ((eprm >> i) == 1) {
        shift_amount = i + 1 - 9;
        break;
      } else {
        shift_amount = 0;
      }
    }

    // shift the commutation time to allow for expanded range and put shift amount in first three bits
    telemetry_value = ((shift_amount << 9) | (eprm >> shift_amount));
  }

  // calculate checksum
  uint16_t csum = 0;
  uint16_t csum_data = telemetry_value;
  for (int i = 0; i < 3; i++) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
  }
  csum = ~csum; // invert it
  csum &= 0xf;

  telemetry_value = (telemetry_value << 4) | csum; // put checksum at the end of 12 bit dshot number

  // GCR RLL encode 16 to 20 bit
  const uint32_t gcr_val = gcr_encode_table[(telemetry_value >> 12)] << 15                     // first set of four digits
                           | gcr_encode_table[(((1 << 4) - 1) & (telemetry_value >> 8))] << 10 // 2nd set of 4 digits
                           | gcr_encode_table[(((1 << 4) - 1) & (telemetry_value >> 4))] << 5  // 3rd set of four digits
                           | gcr_encode_table[(((1 << 4) - 1) & (telemetry_value >> 0))];      // last four digits
  // GCR RLL encode 20 to 21bit output

#ifdef MCU_AT421
  gcr[1 + buffer_padding] = 78;
  for (int i = 19; i >= 0; i--) {                                                                                // each digit in gcr_val
    gcr[buffer_padding + 20 - i + 1] = ((((gcr_val & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 6)) * 78; // exclusive ored with number before it multiplied by 64 to match output timer.
  }
  gcr[buffer_padding] = 0;
#endif
#ifdef MCU_AT415
  gcr[1 + buffer_padding] = 97;
  for (int i = 19; i >= 0; i--) {                                                                                // each digit in gcr_val
    gcr[buffer_padding + 20 - i + 1] = ((((gcr_val & 1 << i)) >> i) ^ (gcr[buffer_padding + 20 - i] >> 6)) * 97; // exclusive ored with number before it multiplied by 64 to match output timer.
  }
  gcr[buffer_padding] = 0;
#endif
}
