/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 19/06/17 3:56p $
 * @brief    MINI51 DS18B20 Driver Sample Code
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *
 *****************************************************************************/
#include <stdio.h>
#include "mini51series.h"

int16_t DS18B20_ReadTemperature(void);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_XTAL, CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART_MODULE, CLK_CLKSEL1_UART_S_XTAL, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(SYS_IPRSTC2_UART_RST_Msk);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    float t;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|        DS18B20 Sample Code          | \n");
    printf("+-------------------------------------+ \n");

    /* Configure P5.2 as Quasi mode  */
    GPIO_SetMode(P5, BIT2, GPIO_PMD_QUASI);

    while (1)
    {
        t = DS18B20_ReadTemperature() * 0.0625;
        printf("\n%f\n", t);
        CLK_SysTickDelay(1000000);//delay 1 sec
    }

}


