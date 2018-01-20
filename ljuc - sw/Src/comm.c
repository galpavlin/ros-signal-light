#include "stm32f0xx_hal.h"
#include "comm.h"
#include <string.h>
#include "bigcase.h"
#include "usbd_cdc_if.h"

static receiver_t receiver;
void process_rx_frame(uint8_t *buf, uint16_t len); //najava big casa
void receiver_init(void); //se ena najava

//Flag 8 bits
//Control         (R/W + register address)
//Information Variable length, n * 8 bits
//FCS 16  bits
//Flag 8 bits



//The frame boundary octet is 01111110, (0x7E in hexadecimal notation).

//Escape sequences

//A “control escape octet”, has the bit sequence ‘01111101’, (7D hexadecimal).
//If either frame boundary octet or escape octet appears in the transmitted data, 
//an escape octet is sent, followed by the original data octet with bit 5 inverted. 
//For example, the data sequence “01111110” (7E hex) would be transmitted 
//as “01111101 01011110” (“7D 5E” hex).


// Static buffer allocations
static uint8_t  _rx_frame[MRU];   // rx frame buffer allocation
static uint8_t  _tx_frame[MRU];   // tx frame buffer allocation
static uint8_t  _payload[MRU];    // payload buffer allocation


//Pošiljanje....
uint8_t tx_buffer[20];
uint8_t length = 0;
uint8_t endoftx = 0;
void tx_byte(uint8_t byte)
{
	if(endoftx == 0)
	{
		tx_buffer[length] = byte; //fill buffer
		length++;
	}
	else
	{
		tx_buffer[length] = byte; //add end of frame byte
		length++;
		USB_write(tx_buffer,length); //transmit whole buffer
		length = 0; //reset length for next tranmittion
	}
}

static void escaped_tx_byte(uint8_t byte)
{
    if((byte == CONTROL_ESCAPE) || (byte == FLAG_SOF))
    {
        tx_byte(CONTROL_ESCAPE);
        byte ^= ESCAPE_BIT;
        tx_byte(byte);
    }
    else
        tx_byte(byte);
}



//sprejemanje
void process_rx_byte(uint8_t rx_byte)
{
    switch (receiver.state)
    {
        case SOF_WAIT:   /// Waiting for SOF flag
            if (rx_byte == FLAG_SOF)
            {
							receiver_init();
							receiver.state = DATARX;
            }
        break;

        case DATARX:     /// Data reception process running
            if (rx_byte == CONTROL_ESCAPE)  // is esc received ?
            {
							receiver.state = PROC_ESC; // handle ESCaped byte
                break;
            }
            // not ESC, check for next sof
            if (rx_byte == FLAG_SOF) // sof received ... process frame
            {
                if (receiver.rx_frame_index == 0) // sof after sof ... drop and continue
                    break;
                if (receiver.rx_frame_index > 2) // at least register addresses + crc
									process_rx_frame(receiver.p_rx_frame, receiver.rx_frame_index);
									receiver_init();
									receiver.state = DATARX;
            } else // "normal" - not ESCaped byte
            {
                if (receiver.rx_frame_index < MRU) // check for max buffer size
                {
									receiver.p_rx_frame[receiver.rx_frame_index] = rx_byte;
									receiver.rx_frame_index++;
                } else // frame overrun, drop frame, init (wait for next SOF)
                {
									receiver_init();   // drop frame and start over
                }
            }
        break;

        case PROC_ESC:  /// process ESCaped byte
					receiver.state = DATARX; // return to normal reception after this
          if (receiver.rx_frame_index < MRU) // check for overrun
            {
              rx_byte ^= ESCAPE_BIT;  // XOR with ESC bit
							receiver.p_rx_frame[receiver.rx_frame_index] = rx_byte;
							receiver.rx_frame_index++;
            } else // frame overrun
            {
							receiver_init();  // drop frame and start over
            }
        break;


    }
}




/* initializatiuon of the receiver state machine, buffer pointers and status variables */
void receiver_init(void)
{
	receiver.rx_frame_index = 0;
	receiver.rx_frame_fcs   = CRC_INIT_VAL;
	receiver.p_rx_frame     = _rx_frame;
  memset(receiver.p_rx_frame, 0, MRU);
	receiver.p_tx_frame     = _tx_frame;
  memset(receiver.p_tx_frame, 0, MRU);
	receiver.p_payload      = _payload;
  memset(receiver.p_payload, 0, MRU);
	receiver.state          = SOF_WAIT;
}



//"big case"
void process_rx_frame(uint8_t *buf, uint16_t len)
{
	bigcase(buf,len);
}


// calculate crc16 CCITT
uint16_t crc16(const uint8_t *data_p, uint8_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--){
			x = crc >> 8 ^ *data_p++;
			x ^= x>>4;
			crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

uint8_t check_crc_receive(const uint8_t *data_p, uint8_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--){
			x = crc >> 8 ^ *data_p++;
			x ^= x>>4;
			crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    if((((crc&0xFF00)>>8) == data_p[1]) && ((crc&0xFF) == data_p[2]))
			return 1;
		else
			return 0;
}


// Transmit "RAW"  frame --- just added SOF and CRC
void tx_frame(const uint8_t *txbuffer, uint8_t len)
{
    uint8_t  byte;
    uint16_t crc = crc16(txbuffer, len);
		endoftx = 0;

    tx_byte(FLAG_SOF);            // Send flag - indicate start of frame
    while(len)
    {
        byte = *txbuffer++;       // Get next byte from buffer
        escaped_tx_byte(byte);        // Send byte with esc checking
        len--;
    }

    escaped_tx_byte((uint8_t)((crc>>8)&0xff));  // Send CRC MSB with esc check
    escaped_tx_byte((uint8_t)(crc&0xff));      	// Send CRC LSB with esc check
		endoftx = 1;
    tx_byte(FLAG_SOF);         									// Send flag - stop frame
}
