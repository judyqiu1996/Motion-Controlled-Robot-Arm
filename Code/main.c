/*
 * TELE_ROBO_MIMCRY 
 * 
 * ECE 395
 * Spring 2013
 * 
 * Matthew Johnson
 * Eric Badger
 */

#include "driver_config.h"

#include "ssp.h"
#include "cpu_lpc1000.h"
#include "nrf24l01.h"
#include "uart.h"
#include "mpu6050.h"
#include "kalman.h"
#include "complementary.h"
#include "i2c.h"
#include "timer32.h"
#include <math.h>
#include "AX12.h"
#include <stdio.h>
#include <stdlib.h>
#include <rt_misc.h>
#include <string.h>	 
//#include "LPC11xx.h"

#define TRANSMITTER 0x0
#define RECEIVER 0x1
#define SET_ID		0x2


/*** CHOOSE BOARD ***/
/********************/

//#define BOARD TRANSMITTER
#define BOARD RECEIVER
//#define BOARD SET_ID
/********************/

// Channel 0..125
#define	_CH1 5  //robot 1's frequency		    
#define _CH2 16	//robot 2's frequency
#define	_Address_Width	5	// 3..5
#define _Buffer_Size 32    // 1..32

//LED Functions
#define LED_DIR_OUT LPC_GPIO0->DIR |= (1<<7);
#define LED_ON LPC_GPIO0->MASKED_ACCESS[(1<<7)] = 0
#define LED_OFF  LPC_GPIO0->MASKED_ACCESS[(1<<7)] = (1<<7)	


/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void SER_init (void) {
   
  /* configure PINs GPIO1.6, GPIO1.7 for UART */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1UL <<  6) |   /* enable clock for GPIO      */
                                (1UL << 16) );  /* enable clock for IOCON     */

  LPC_IOCON->PIO1_6  =  (1UL <<  0);            /* P1.6 is RxD                */
  LPC_IOCON->PIO1_7  =  (1UL <<  0);            /* P1.7 is TxD                */

  /* configure UART0 */
  LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 12);    /* Enable clock to UART       */
  LPC_SYSCON->UARTCLKDIV     =  (4UL <<  0);    /* UART clock =  CCLK / 4     */

  LPC_UART->LCR = 0x83;                   /* 8 bits, no Parity, 1 Stop bit    */
  LPC_UART->DLL = 4;                      /* 115200 Baud Rate @ 12.0 MHZ PCLK */
  LPC_UART->FDR = 0x85;                   /* FR 1.627, DIVADDVAL 5, MULVAL 8  */
  LPC_UART->DLM = 0;                      /* High divisor latch = 0           */
  LPC_UART->LCR = 0x03;                   /* DLAB = 0                         */
}




/* Import external functions from Serial.c file                               */
//extern void SER_init (void);

void Led_Blink(void) {
 	LED_ON;
	delay32Ms(1, 1000);
	LED_OFF;
	delay32Ms(1, 500);

}

//Globals for interrupt from nRF24L01+
volatile uint8_t MPU_Request = 0;
char Buf[_Buffer_Size] = {'\0'}; 

void NRF24L01_Print_Register()
{
	uint32_t i;
	uint32_t j = 0;
	for(i=0;i<24;i++)
	{
		if (j <5)
		{
			printf("%d:0x%x ",i,NRF24L01_ReadReg(i));
			j++;
		}
		
		else
		{
			printf("%d:0x%x ",i,NRF24L01_ReadReg(i));
			printf("\r\n");
			j=0;
		}
	}
	printf("\r\n");
}


NRF24L01_Print_R(char Reg, int Size)
{
	char Buf[10];
	int i;
	NRF24L01_ReadRegBuf(Reg, Buf, Size);
	printf("%d: ",Reg );
	for (i=0;i<Size;i++)
		printf("%x ",Buf[i]);
	printf("\r\n");
	
}
//Reads from RX FIFO, blocks until success or timeout
void NRF24L01_Receive(char Buf[_Buffer_Size]) {
	uint32_t i = 0;
	uint32_t j = 0;

	while ((NRF24L01_Get_Status() & _RX_DR) != _RX_DR && i < 0xFF) i++;
	if(i >= 0xFF)
	{
		NRF24L01_WriteReg(W_REGISTER | STATUS, _RX_DR );
		for(j=0; j < _Buffer_Size; j++) Buf[j] = '\0';
		return;
	}

	NRF24L01_CE_LOW;

	NRF24L01_Read_RX_Buf(Buf, _Buffer_Size);
	NRF24L01_WriteReg(W_REGISTER | STATUS, _RX_DR );
}

//Transmits data, blocks until success or timeout
void NRF24L01_Send(char Buf[_Buffer_Size]) {
	uint32_t i = 0;
	NRF24L01_Set_Device_Mode(_TX_MODE);
	NRF24L01_Write_TX_Buf(Buf, _Buffer_Size);															    
	while(i == 0)
	{
	NRF24L01_RF_TX();
	//delay32Ms(1,1);
	while ((NRF24L01_Get_Status() & _TX_DS)!= _TX_DS && (NRF24L01_Get_Status() & _MAX_RT)!= _MAX_RT);
	if ((NRF24L01_Get_Status() & _MAX_RT) != _MAX_RT)i = 1;
	NRF24L01_WriteReg(W_REGISTER | STATUS, _TX_DS | _MAX_RT);
	}
	NRF24L01_Set_Device_Mode(_RX_MODE);
}

int main(void) {
	

//BASE STATION MASTER_ARM
int32_t i=0;
	int ID = 11;
	float gyrox,gyroy,gyroz,y,x,z,dt;
	float temp;
	float pitch,pitch2;
	float * value;
	char * cvalue;
	float list[4];
	//int position[] = {511, 511, 511, 511, 511, 511};
//	uint8_t robot1_2;
	//uint16_t CH;
	char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55 };
	char Address2[_Address_Width] = {0x11, 0x21, 0x33, 0x44, 0x55};
	char Address3[_Address_Width] = {0x11, 0x23, 0x33, 0x44, 0x55};
	//char * sensor[] = {"R_ROTATOR", "R_SHOULDER", "R_ELBOW", "L_ROTATOR", "L_SHOULDER", "L_ELBOW"};

	
	delay32Ms(1, 2000);//allow time for devices to power up
	LED_DIR_OUT;
	LPC_GPIO1->DIR &= ~(1<<5); //input mode	
	SER_init();	
	Led_Blink();
	
/*
	//initialize serial, also initializes GPIO
	AX12_begin(_1MHZ);
	LED_DIR_OUT;		 

	//toggle servo status LEDs
	AX12_ledStatus(BROADCAST_ID, ON);
	delay32Ms(1,1000);
	AX12_ledStatus(BROADCAST_ID, OFF);

	//initialize servos to 180 degrees
	for(i=0; i < 6; i++)
	{
		 AX12_move(i, position[i]);
		 delay32Ms(1, 500);
	}
*/
	
	//initialize SPI protocol
	SSP_IOConfig(0);
	SSP_Init(0); 
	
	Led_Blink();
/*	
	//Read Robot Switch
	robot1_2 =  LPC_GPIO1->MASKED_ACCESS[(1<<2)];
	if(robot1_2) CH = _CH1;
	else CH = _CH2;
*/

	//Initialize transceiver		
	
	
	Led_Blink();
#if BOARD == TRANSMITTER
	AX12_begin(_1MHZ);
	NRF24L01_Init(_RX_MODE, _CH1, _1Mbps, Address, _Address_Width, _Buffer_Size);
		//init_timer32(0, 48); //us
	//enable_timer32(0);
	while(1)
	{
		//AX12_move(BROADCAST_ID,512);
			//delay32Ms(1,5);
		//printf("Starting\r\n");
		
		
	
			
		
		
		
		
			NRF24L01_Set_TX_Address(Address, _Address_Width);
			NRF24L01_Set_RX_Pipe(0, Address, _Address_Width, _Buffer_Size);
		
			
			//Address[2] = i;
			//Switch Address
			//NRF24L01_Set_TX_Address(Address, _Address_Width);
			//NRF24L01_Set_RX_Pipe(0, Address, _Address_Width, _Buffer_Size);
			//Clear and Initiialize Buffer
			for(i=0; i < _Buffer_Size; i++) Buf[i] = '\0'; 
				strcpy(Buf,"HELLO\0");
			//Transmit Request	   		
			NRF24L01_Send(Buf);	
			delay32Ms(1,5);
			//Receive Response
			//NRF24L01_Print_Register();
			//NRF24L01_Print_R(0xA,5);
			//NRF24L01_Print_R(0x10,5);
			NRF24L01_Receive(Buf);
			
			//printf("%d				%d\r\n",i,k);
			value = (float*) Buf;
			//printf("%f ",value[0]);
			//printf("%f\r\n",value[1]);
			//if(i==0){i=1;angle=180;}
			//else{angle =(angle + value[1]) ;}
			
			
			
			//if(value[0] == 0.0 || value[2] ==0.0 )continue;
		
			//printf("1:%.2f,%.2f\r\n",value[0],value[1]);
			
			if(value[1] == 0.0 ) {}
			else{
			temp = value[1];
			if(temp <-60) temp = -60;
			if(temp > 60) temp = 60;
			//value[1] = value[1];
			temp = (150 + temp)*1023/300;
			i = temp;
			//printf("%d,",i);
			AX12_move(14,i);
				
			}
			delay32Ms(1,1);
			
			if(value[0] == 0.0) {}
			
			else{
			pitch = value[0];
			//value[1] = (value[1]>0)?value[1]:  360+(value[1]);
			//value[1] = 360.0-value[1];
				temp = value[0];
			if(temp <-90) temp = -90;
			if(temp > 90) temp = 90;
			//value[1] = value[1] - 30;
			temp = ( 150 +  temp )*1023/300;
			i = temp;
			AX12_move(15,i);
			//printf("%d,",i);
			}
			
		
			//delay32Ms(1,2);
			//printf("%d\r\n",l);
			
			//printf("%d\n",i);
			//delay32Ms(1,20);
			//printf("%f\r\n",*value);
		
				NRF24L01_Set_TX_Address(Address2, _Address_Width);
				NRF24L01_Set_RX_Pipe(0, Address2, _Address_Width, _Buffer_Size);
				
				
				for(i=0; i < _Buffer_Size; i++) Buf[i] = '\0'; 
				strcpy(Buf,"HELLO\0");
			//Transmit Request	   		
				NRF24L01_Send(Buf);	
				delay32Ms(1,5);
			//Receive Response
				//NRF24L01_Print_Register();
			//	NRF24L01_Print_R(0xA,5);
				//NRF24L01_Print_R(0x10,5);
				NRF24L01_Receive(Buf);
			
				value = (float*) Buf;
				//printf("2:%.2f\r\n",value[0]);
				if(value[0] == 0.0 ) {}
				else{
				temp = value[0] - pitch;
				//value[1] = (value[1]>0)?value[1]:  360+(value[1]);
				//value[1] = 360.0-value[1];
				if(temp <0) temp = 0;
				if(temp > 90) temp = 90;
				//value[1] = value[1] - 30;
				temp = ( 150 -  temp )*1023/300;
				i = temp;
				AX12_move(16,i);
				//printf("%d\r\n",i);
				
					} 
				
					//delay32Ms(1,1000);
					
					
						NRF24L01_Set_TX_Address(Address3, _Address_Width);
				NRF24L01_Set_RX_Pipe(0, Address3, _Address_Width, _Buffer_Size);
				
				
				for(i=0; i < _Buffer_Size; i++) Buf[i] = '\0'; 
				strcpy(Buf,"HELLO\0");
			//Transmit Request	   		
				NRF24L01_Send(Buf);	
				delay32Ms(1,5);
			//Receive Response
				//NRF24L01_Print_Register();
			//	NRF24L01_Print_R(0xA,5);
				//NRF24L01_Print_R(0x10,5);
				NRF24L01_Receive(Buf);

				value = (float*) Buf;
					
				if(value[1] == 0.0 ) {}
				else{
				temp = value[1];
				if(temp <-60) temp = -60;
				if(temp > 60) temp = 60;
				//value[1] = value[1];
				temp = (150 + temp)*1023/300;
				i = temp;
				//printf("%d,",i);
				AX12_move(12,i);
				
			}
			delay32Ms(1,1);
			
			if(value[0] == 0.0) {}
			
			else{
			pitch2 = value[0];
			//value[1] = (value[1]>0)?value[1]:  360+(value[1]);
			//value[1] = 360.0-value[1];
				temp = value[0];
			if(temp <-90) temp = -90;
			if(temp > 90) temp = 90;
			//value[1] = value[1] - 30;
			temp = ( 150 -  temp )*1023/300;
			i = temp;
			AX12_move(13,i);
			//printf("%d,",i);
			}
			//delay32Ms(1,1);
			//AX12_move(11,65);
				
		}
#endif	
		
#if BOARD == RECEIVER			
	NRF24L01_Init(_RX_MODE, _CH1, _1Mbps, Address3, _Address_Width, _Buffer_Size);
	NRF24L01_DRint_Init();
	I2CInit(I2CMASTER);
  //MPU6050_initialize();
//	MPU6050_dmp_init();
//	printf("All done!!\r\n");
		
		
		
			while(MPU6050_init() != 0)
	{
		for(i=0;i<100000;i++);
	} 
	
	
		delay32Ms(1,500);
	MPU6050_setZero();
	//kalman_init();
	init_timer32(0, 48); //us
	enable_timer32(0);
	while(1)
	{
	
		
		
		gyroy =  MPU6050_getGyroRoll_degree();
		gyrox =  MPU6050_getGyroX();
		gyroz = MPU6050_getGyroZ();
		dt = (float)read_timer32(0) / 1000000;
		
		//printf("%f\r\n",gyroz);
		//gyro = gyro;
		reset_timer32(0);
		enable_timer32(0);	
		x = MPU6050_getAccel_x();
		z = MPU6050_getAccel_z();
		y = MPU6050_getAccel_y();
		
		
		
		
		
		complementary_filter(list,x,y,z,gyrox,gyroy, gyroz, dt);
		
			//angle = kalman_update(acc_angle, gyro, dt);
			//angle = 0.1*acc_angle + 0.1*j + 0.1*k + 0.7*(gyro*dt + angle);
		//angle = 0.5*acc_angle + 0.5*(gyro*dt - l);
	
		//angle = 0.98*(gyro*dt + angle) + 0.02 * acc_angle;
			cvalue = (char*) list;
			//Receive Response
			if(LPC_GPIO1->MASKED_ACCESS[1<<5] == 0)
			{
			//NRF24L01_Print_Register();
			NRF24L01_Receive(Buf);
			if(strncmp(Buf,"HELLO",_Buffer_Size)==0)
			{
				memset(Buf,0,_Buffer_Size);
				for(i=0;i<sizeof(float)*2;i++)
					Buf[i] = cvalue[i];
				NRF24L01_Send(Buf);
			}
			



			
		}
		
		
		}
			
			
		

	
	
		
		
#endif
#if BOARD == SET_ID
	

	//BASE STATION MASTER FOR SETTING ID
	AX12_begin(_1MHZ);


	AX12_setID(BROADCAST_ID, ID);

	LED_ON;

	AX12_ledStatus(ID, ON);	  //Check if loaded
	AX12_move(ID, 512);
	
#endif
	
	}


//------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------



