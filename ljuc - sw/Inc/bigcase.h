#include "stm32f0xx_hal.h"

void bigcase(uint8_t *buf, uint16_t len);

typedef union
{
	struct
	{
		uint8_t Address			:6; /*!< bit: 0...5 Register address */
		uint8_t Ack					:1; /*!< bit: 6     Acknowledge */
		uint8_t	ReadWrite		:1;	/*!< bit: 7     Read/Write flag */
	} b;
  uint8_t r;
} RegisterCmd_t;


typedef union
{
	struct
	{
		uint32_t byte0			:8; /*!< bits: 0...7 byte0 */
		uint32_t byte1			:8; /*!< bits: 8...15 byte1 */
		uint32_t byte2			:8; /*!< bits: 16...23 byte2 */
		uint32_t byte3			:8; /*!< bits: 24...31 byte3 */
	} b;
  uint32_t r;
} moj32bitniRegister_t;

