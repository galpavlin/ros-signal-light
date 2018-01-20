#include "stm32f0xx_hal.h"
#include "string.h"
#include "comm.h"
#include "bigcase.h"
#include "lights.h"
#include "beeper.h"


extern float voltage;

//reset
void reset_board(void)
{
	while(1);
}


void bigcase(uint8_t *buf, uint16_t len)
{

	RegisterCmd_t regcmd;
	uint8_t txbuf [4];
	regcmd.r = buf[0];
	uint8_t color[3],flag = 0,i;
	
	/*--------------WRITE---------------*/
	if(regcmd.b.ReadWrite) //write bit set
	{
		//calculate and check CRC
		uint16_t crc_res = crc16(buf, 4);
		if(((crc_res>>8)&0xFF) == buf[4] && (crc_res&0xFF) == buf[5])
		{		
			//beepy(1,2);
			switch(regcmd.b.Address)
			{
				case 0x00 : for(i=0;i<2;i++) {
									color[i] = buf[i+1]; }
								set_color(0,color[0],color[1],color[2]); 
								regcmd.b.Ack = 1;
								tx_frame(&regcmd.r,1); break;
				case 0x01 : for(i=0;i<2;i++) {
									color[i] = buf[i+1]; }
								set_color(1,color[0],color[1],color[2]);
								regcmd.b.Ack = 1;
								tx_frame(&regcmd.r,1); break;
				case 0x02 : for(i=0;i<2;i++) {
									color[i] = buf[i+1]; }
								set_color(2,color[0],color[1],color[2]);
								regcmd.b.Ack = 1;
								tx_frame(&regcmd.r,1); break;
				case 0x03 : regcmd.b.Ack = 1; 					//beeper
								beepy(buf[1],buf[2]);
								tx_frame(&regcmd.r,1); break;
				case 0x04 : if(buf[1]==0x69) {
									for(i=0;i<3;i++)
										set_color(i,0,0,0);
									beepy(0,0); }
								if(buf[2]==0x69)
									reset_board();
								regcmd.b.Ack = 1;
								tx_frame(&regcmd.r,1); break;
			}					
		}		
	}		
				
	
	/*--------------READ---------------*/
	else
	{
		uint16_t crc_res = crc16(buf, 1);
		if(((crc_res>>8)&0xFF) == buf[1] && (crc_res&0xFF) == buf[2])
		{
			switch(regcmd.b.Address)
			{
				case 0x03 :	regcmd.b.Ack=1;
										txbuf[0] = regcmd.r;
										txbuf[1] = get_beepmode();
										txbuf[2] = get_beeptime();
										txbuf[3] = 0x00;
										tx_frame(txbuf,4); break;
				case 0x05 :	regcmd.b.Ack=1;
										txbuf[0] = regcmd.r;
										txbuf[1] = (uint16_t)(voltage*1000)>>8;
										txbuf[2] = ((uint16_t)(voltage*1000))&0xFF;
										txbuf[3] = 0x00;
										tx_frame(txbuf,4); break;
			}
		}
	}
}

