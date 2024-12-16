/**************************************************************************//**
 * @file     DS18B20.c
 * @version  V1.00
 * $Date: 19/06/17 3:56p $
 * @brief DS18B20 Driver
 *
 *****************************************************************************/
#include "mini51series.h"
/*---------------------------------------------------------------------------*/
/* Macro definition                                                          */
/*---------------------------------------------------------------------------*/
#define  DQ                      P52            //DQ Pin

#define  CMD_SKIP_ROM            0xCC
#define  CMD_CONVERT_T           0x44
#define  CMD_READ_SCRATCHPAD     0xBE
/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
static void DS18B20_Reset(void);
static void DS18B20_WriteByte(uint8_t u8dat);
static uint8_t DS18B20_ReadByte(void);



/**
  * @brief  Read temperature from DS18B20
  * @param  None
  * @return  i16temp: temperature
  */
int16_t DS18B20_ReadTemperature(void)
{
    uint8_t  u8tempH, u8tempL;
    int16_t  i16temp;

    DS18B20_Reset();
    DS18B20_WriteByte(CMD_SKIP_ROM);
    DS18B20_WriteByte(CMD_CONVERT_T);

    while (!DQ);                             //Waiting for conversion to complete

    DS18B20_Reset();
    DS18B20_WriteByte(CMD_SKIP_ROM);
    DS18B20_WriteByte(CMD_READ_SCRATCHPAD);
    u8tempL = DS18B20_ReadByte();            //Read temperature low byte
    u8tempH = DS18B20_ReadByte();            //Read temperature high byte
    i16temp = (u8tempH << 8) | u8tempL;

    return (i16temp);
}



/**
  * @brief  Reset the DS18B20 and check if the device is present
  * @param  None
  * @return None
  */
static void DS18B20_Reset(void)
{
    uint8_t i;
    i = 1;

    while (i)
    {
        DQ = 0;                              //Send a low level reset signal
        CLK_SysTickDelay(480);               //Delay at least 480us
        DQ = 1;                              //Release data line
        CLK_SysTickDelay(60);                //Waiting for 60us
        i = DQ;                              //Detect the presence of pulses
        CLK_SysTickDelay(420);               //Waiting for the device to release the data line
    }
}


/**
  * @brief  Read 1 byte data from DS18B20
  * @param  None
  * @return u8dat: Read data
  */
static uint8_t DS18B20_ReadByte(void)
{
    uint8_t i;
    uint8_t u8dat = 0;

    for (i = 0; i < 8; i++)
    {
        u8dat >>= 1;
        DQ = 0;                              //Start time slice
        CLK_SysTickDelay(1);                 //Delay waiting
        DQ = 1;                              //Ready to receive
        CLK_SysTickDelay(1);                 //Reception delay

        if (DQ) u8dat |= 0x80;               //Read data

        CLK_SysTickDelay(60);                //Waiting for the end of time
    }

    return u8dat;
}


/**
  * @brief  Write 1 byte of data to the DS18B20
  * @param  u8dat            Write data
  * @return None
  */
static void DS18B20_WriteByte(uint8_t u8dat)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        DQ = 0;                              //Start time slice
        CLK_SysTickDelay(1);                 //Delay waiting

        if (u8dat & 0x01) DQ = 1;            //Send data

        CLK_SysTickDelay(60);                //Waiting for the end of time
        DQ = 1;                              //Restore data line
        CLK_SysTickDelay(1);                 //Recovery delay
        u8dat >>= 1;
    }
}
