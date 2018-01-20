//#ifndef __COMM_H__
#define __COMM_H__

#include "stm32f0xx_hal.h"

#define MRU    						 100  	// max length of received data
#define FLAG_SOF           0x7e   // Flag
#define CONTROL_ESCAPE     0x7d   // Control Escape octet
#define ESCAPE_BIT         0x20   // Transparency modifier octet (XOR bit)
#define CRC_INIT_VAL       0xffff



typedef enum
{
	SOF_WAIT,
	DATARX,
	PROC_ESC,
} rx_state_t;


typedef struct
{
    uint8_t             ctrl;
    uint8_t             *p_tx_frame;            // tx frame buffer
    uint8_t             *p_rx_frame;            // rx frame buffer
    uint8_t             *p_payload;                // payload pointer
    uint16_t            rx_frame_index;
    uint16_t            rx_frame_fcs;
    rx_state_t          state;
} receiver_t;


void process_rx_byte(uint8_t rx_byte);
void tx_frame(const uint8_t *txbuffer, uint8_t len);
uint16_t crc16(const uint8_t *data_p, uint8_t length);
uint8_t check_crc_receive(const uint8_t *data_p, uint8_t length);
