#include <stdio.h>
#include "NuMicro.h"
#include "stdbool.h"


#define SLAVE_ADDR   0x43

#define PART_ID       0x00 
#define DIE_REV       0x02 
#define UID           0x04 
#define SYS_CTRL      0x10 
#define SYS_STAT      0x11 
#define SENS_RUN      0x21 
#define SENS_START    0x22 
#define SENS_STOP     0x23 
#define SENS_STAT     0x24 
#define T_VAL         0x30 
#define H_VAL         0x33 


#define CLK_HIRC    1
#define CLK_HXT     0
#define CLK_SOURCE  CLK_HIRC
#define PLL_CLOCK   FREQ_48MHZ


#define CRC7WIDTH 7 
#define CRC7POLY 0x89 
#define CRC7IVEC 0x7F 
#define DATA7WIDTH 17
#define DATA7MASK ((1UL<<DATA7WIDTH)-1)
#define DATA7MSB (1UL<<(DATA7WIDTH-1)) 




void SYS_Init(void)
{	
		/* Unlock protected registers */
    SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable IP clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set IP clock */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /---------------------------------------------------------------------------------------------------------/
    /* Init I/O Multi-function                                                                                 */
    /---------------------------------------------------------------------------------------------------------/

    
    SYS->GPA_MFPH = 0x00000000;
    SYS->GPA_MFPL = 0x00000000;
    SYS->GPC_MFPH = 0x00000000;
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC1MFP_I2C0_SCL | SYS_GPC_MFPL_PC0MFP_I2C0_SDA;
    SYS->GPD_MFPH = 0x00000000;
    SYS->GPD_MFPL = 0x00000000;
    SYS->GPE_MFPH = 0x00000000;
    SYS->GPE_MFPL = 0x00000000;
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT;
      Uart0DefaultMPF();
    /* Lock protected registers */
    SYS_LockReg();
}





 uint32_t crc7( uint32_t val ) 
{

uint32_t pol= CRC7POLY;

pol = pol << (DATA7WIDTH-CRC7WIDTH-1);

uint32_t bit = DATA7MSB;

val = val << CRC7WIDTH;
bit = bit << CRC7WIDTH;
pol = pol <<CRC7WIDTH;

val |= CRC7IVEC;

while( bit & (DATA7MASK<<CRC7WIDTH) ) {
if( bit & val ) val ^= pol;
bit >>= 1;
pol >>= 1;
}
return val;
}





void TH09C_StartMeasurement()
{
			uint8_t rdata[3]={0,0,0};
			I2C_WriteByteOneReg(I2C0, SLAVE_ADDR, SENS_RUN, 0x03);
			I2C_WriteByteOneReg(I2C0, SLAVE_ADDR, SENS_START, 0x03);
			printf("status: %d\n",I2C_ReadByteOneReg(I2C0, SLAVE_ADDR, SENS_STAT));
			CLK_SysTickDelay(130000);
	    float raw_t_value=I2C_ReadByteOneReg(I2C0, SLAVE_ADDR, T_VAL);
			printf("raw t value: %d\n",raw_t_value);
			CLK_SysTickDelay(130000);
			printf("raw t value: %d\n",I2C_ReadByteOneReg(I2C0, SLAVE_ADDR, T_VAL));

			I2C_ReadMultiBytesOneReg(I2C0,  SLAVE_ADDR,  T_VAL,  &rdata[0],  6);
			
			uint32_t Real_t= (rdata[2]<<16)+(rdata[1]<<8)+(rdata[0]);
			printf("Real data: %d \n", Real_t);
			
			float TinK = Real_t/64;
			float TinC = TinK-273.15;
			float TinF = TinC * 1.8 +32.0;
			printf("Kelvin: %lf \n Celcius: %lf \n Fahrenheit: %lf \n", TinK, TinC, TinF);
			
}





void ReadMeasurement()
{
		uint8_t rbuf[6];
		bool i2c_ok= true;
		i2c_ok &= I2C_WriteByteOneReg(I2C0, SLAVE_ADDR, SENS_START, 0x03);
	  CLK_SysTickDelay(130000);
		i2c_ok &= I2C_ReadMultiBytesOneReg(I2C0, SLAVE_ADDR, T_VAL, &rbuf[0],6);
		uint32_t t_val = (rbuf[2]<<16) + (rbuf[1]<<8) + (rbuf[0]<<0);
		uint32_t h_val= (rbuf[5]<<16) + (rbuf[4]<<8) +(rbuf[3]<<0);
		uint32_t t_data = (t_val>>0 ) & 0xffff;
		uint32_t t_valid= (t_val>>16) & 0x1;
		uint32_t t_crc = (t_val>>17) & 0x7f;
//		printf("TH09C: T: %06x %02x %01x %04x\n", t_val, t_crc, t_valid, t_data );
		uint32_t t_payl = (t_val>>0 ) & 0x1ffff;
		bool t_crc_ok= crc7(t_payl)==t_crc;
		
		float TinK = (float)t_data / 64; // Temperature in Kelvin
		float TinC = TinK - 273.15; // Temperature in Celsius
		float TinF = TinC * 1.8 + 32.0; // Temperature in Fahrenheit
		printf("TH09C: T:(i2c=%d crc=%d valid=%d) \nTemp in kelvin:%5.1f K \nTemp in celcius:%4.1f C \nTemp in fahrenheit:%4.1f F\n\n\n",i2c_ok,t_crc_ok,t_valid,TinK, TinC, TinF);
		
	
	
		
		uint32_t h_data = (h_val>>0 ) & 0xffff;
		uint32_t h_valid= (h_val>>16) & 0x1;
		uint32_t h_crc = (h_val>>17) & 0x7f;

		uint32_t h_payl = (h_val>>0 ) & 0x1ffff;
		bool h_crc_ok= crc7(h_payl)==h_crc;
		
		float H= (float)h_data/512; 

}





void I2C0_Init(void)
{
		I2C_Open(I2C0, 100000);
}




int main()
{
    SYS_Init();
		/* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);
		I2C0_Init();
		CLK_SysTickDelay(10000);
	
//		TH09C_StartMeasurement();
    
		
		ReadMeasurement();
		
    while(1)
			{
				ReadMeasurement();
				CLK_SysTickDelay(3000000);
			};
}
