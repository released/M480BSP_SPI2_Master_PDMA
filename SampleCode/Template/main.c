/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LED_R						(PH0)
#define LED_Y						(PH1)
#define LED_G						(PH2)

#define SPI_MASTER_TX_DMA_CH 		(0)
#define SPI_MASTER_RX_DMA_CH 		(1)
#define SPI_MASTER_OPENED_CH   	((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))

#define SPI_TARGET_FREQ				(8000000ul)	//(48000000ul)

#define DATA_NUM					(8)

uint8_t g_au8MasterToSlaveTestPattern[DATA_NUM]={0};
uint8_t g_au8SlaveToMasterTestPattern[DATA_NUM]={0};
uint8_t g_au8MasterRxBuffer[DATA_NUM]={0};
uint8_t g_au8SlaveRxBuffer[DATA_NUM]={0};

enum
{
	SPI_TX = 0,
	SPI_RX = 1,		
};


typedef enum{
	flag_DEFAULT = 0 ,

	flag_SPI_Transmit_timing ,
	flag_SPI_Transmit_finish ,	

	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

uint32_t conter_tick = 0;

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}


void tick_counter(void)
{
	conter_tick++;
}

uint32_t get_tick(void)
{
	return (conter_tick);
}

void set_tick(uint32_t t)
{
	conter_tick = t;
}


void SPI_Master_RX_PDMA(uint8_t* Rx , uint16_t len)
{
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	
	
    PDMA_Open(PDMA, (1 << SPI_MASTER_RX_DMA_CH));

	//RX	
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI2->RX, PDMA_SAR_FIX, (uint32_t)Rx, PDMA_DAR_INC);
    /* Set request source; set basic mode. */

    PDMA_SetTransferMode(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_SPI2_RX, FALSE, 0);
	
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    SPI_TRIGGER_RX_PDMA(SPI2);

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & (1 << SPI_MASTER_RX_DMA_CH)) == (1 << SPI_MASTER_RX_DMA_CH))
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA,1 << SPI_MASTER_RX_DMA_CH);
                /* Disable SPI PDMA RX function */
                SPI_DISABLE_RX_PDMA(SPI2);
                break;
            }

            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS(PDMA);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
                break;
            }
        }
    }

}

void SPI_Master_TX_PDMA(uint8_t* Tx , uint16_t len)
{
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	

    PDMA_Open(PDMA, (1 << SPI_MASTER_TX_DMA_CH));

	//TX
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_TX_DMA_CH, (uint32_t)Tx, PDMA_SAR_INC, (uint32_t)&SPI2->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
	
    PDMA_SetTransferMode(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_SPI2_TX, FALSE, 0);
	
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    SPI_TRIGGER_TX_PDMA(SPI2);

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & (1 << SPI_MASTER_TX_DMA_CH)) == (1 << SPI_MASTER_TX_DMA_CH))
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA,1 << SPI_MASTER_TX_DMA_CH);
                /* Disable SPI PDMA TX function */
                SPI_DISABLE_TX_PDMA(SPI2);
                break;
            }

            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS(PDMA);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
                break;
            }
        }
    }

}

void SPI_Master_PDMA_Enable(uint8_t TxRx)
{
    uint16_t i = 0;
    static uint16_t j = 0;

	LED_Y = 0;
	
	if (TxRx == SPI_TX)
	{
		//prepare master TX data
		g_au8MasterToSlaveTestPattern[0] = 0xAA;
		g_au8MasterToSlaveTestPattern[1] = 0xDD;

		j = 1;
	    for (i = 2; i < DATA_NUM ; i++)
	    {
	        g_au8MasterToSlaveTestPattern[i] = (i + 0x10*(j++));
	    }
		j = 0;
	
		//TX
		SPI_Master_TX_PDMA(g_au8MasterToSlaveTestPattern , DATA_NUM);

		LED_Y = 1;
		
	}
	else
	{
		SPI_Master_RX_PDMA(g_au8MasterRxBuffer,DATA_NUM);		

		LED_Y = 1;
	}
	
}

void SPI_Master_Init(void)
{
	uint32_t clk_count = SPI_TARGET_FREQ;
	uint8_t u8Item = 0;
    uint32_t u32Div;

	
    printf("1 ) 8MHz \r\n");
    printf("2 ) 16MHz \r\n");
    printf("3 ) 24MHz \r\n");
    printf("4 ) 48MHz \r\n");
    printf("5 ) 60MHz \r\n");
    printf("6 ) 72MHz \r\n");	
    printf("7 ) 84MHz \r\n");	
    printf("8 ) 96MHz \r\n");	
	
    u8Item = getchar();
    printf("\r\n");

	
	switch(u8Item) 
	{
		case '1':
		    clk_count = 8000000ul;
		    break;
		case '2':
		    clk_count = 16000000ul;
		    break;
		case '3':
		    clk_count = 24000000ul;
		    break;
		case '4':
		    clk_count = 48000000ul;
		    break;
		case '5':
		    clk_count = 60000000ul;
		    break;
		case '6':
		    clk_count = 72000000ul;
		    break;
		case '7':
		    clk_count = 84000000ul;
		    break;
		case '8':
		    clk_count = 96000000ul;
		    break;			
		default : 
			clk_count = SPI_TARGET_FREQ;
			break;
			
	}
	
	printf("Select : %c , %8d\r\n" , u8Item , clk_count);


    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_0, 8, clk_count);
	u32Div = (SPI2->CLKDIV & SPI_CLKDIV_DIVIDER_Msk) >> SPI_CLKDIV_DIVIDER_Pos;
	printf("\r\nSPI_GetBusClock : %8d , div : %d\r\n" , SPI_GetBusClock(SPI2) , u32Div);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI2, SPI_SS, SPI_SS_ACTIVE_LOW);

}

void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	printf("%s  : 0x%2X\r\n" , __FUNCTION__ ,res);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
	
			case '1':

				break;	

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
			
				NVIC_SystemReset();
			
				break;		
			
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}



void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetPLLClockFreq : %8d\r\n",CLK_GetPLLClockFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}


void TMR3_IRQHandler(void)
{
	static uint16_t CNT = 0;	
	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);
		tick_counter();

		if ((get_tick() % 10) == 0)
		{
			set_flag(flag_SPI_Transmit_timing , ENABLE);
		}
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
			LED_R ^= 1;
		}
    }
}

void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}


void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable clock source */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
//    CLK_SetCoreClock(FREQ_192MHZ);

	#if 1
    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
//    CLK->PLLCTL = (CLK->PLLCTL & ~(0x000FFFFFUL)) | 0x0000421EUL;
	CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT , FREQ_192MHZ);

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));
	
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2);
	#endif

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PCLK1, MODULE_NoMsk);
    CLK_EnableModuleClock(SPI2_MODULE);

    CLK_EnableModuleClock(PDMA_MODULE);

    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI2 multi-function pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA8MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk| SYS_GPA_MFPH_PA10MFP_Msk| SYS_GPA_MFPH_PA11MFP_Msk);	
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA8MFP_SPI2_MOSI | SYS_GPA_MFPH_PA9MFP_SPI2_MISO | SYS_GPA_MFPH_PA10MFP_SPI2_CLK | SYS_GPA_MFPH_PA11MFP_SPI2_SS;

    /* Enable SPI2 clock pin (PA10) schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN8_Msk | GPIO_SMTEN_SMTEN9_Msk | GPIO_SMTEN_SMTEN10_Msk | GPIO_SMTEN_SMTEN11_Msk);

    /* Enable SPI2 I/O high slew rate */
	GPIO_SetSlewCtl(PA, (BIT8 | BIT9 | BIT10 | BIT11), GPIO_SLEWCTL_FAST);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
	UART0_Init();

	SPI_Master_Init();

	LED_Init();
	TIMER3_Init();
	
    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(100);

		if (is_flag_set(flag_SPI_Transmit_timing))	
//		if (is_flag_set(flag_SPI_Transmit_finish))	
		{
			set_flag(flag_SPI_Transmit_timing , DISABLE);
			set_flag(flag_SPI_Transmit_finish , DISABLE);
			
//			SPI_Master_PDMA_Enable(SPI_RX);
			SPI_Master_PDMA_Enable(SPI_TX);
		}

    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
