/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2017
//***********************************************************************************************************

//***********************************************************************************************************
//  File Function: N76E003 Timer0/1 Mode1 demo code
//***********************************************************************************************************
#include "N76E003.h"
#include "Common.h"
#include "Delay.h"
#include "Timer01.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include <math.h>
//*****************  The Following is in define in Fucntion_define.h  ***************************
//****** Always include Function_define.h call the define you want, detail see main(void) *******
//***********************************************************************************************
#if 0
//#define		TIMER0_MODE0_ENABLE		TMOD&=0x0F
//#define		TIMER0_MODE1_ENABLE		TMOD&=0x0F;TMOD|=0x10
//#define		TIMER0_MODE2_ENABLE		TMOD&=0x0F;TMOD|=0x20
//#define		TIMER0_MODE3_ENABLE		TMOD&=0x0F;TMOD|=0x3F

//#define		TIMER1_MODE0_ENABLE		TMOD&=0xF0
//#define		TIMER1_MODE1_ENABLE		TMOD&=0xF0;TMOD|=0x01
//#define		TIMER1_MODE2_ENABLE		TMOD&=0xF0;TMOD|=0x02
//#define		TIMER1_MODE3_ENABLE		TMOD&=0xF0;TMOD|=0xF3
#endif



#define LED_TX_PIN P00
#define LED_TX_PIN_SET set_P00
#define LED_TX_PIN_CLR clr_P00

#define LED_STS_PIN P01
#define LED_STS_SET set_P01
#define LED_STS_CLR clr_P01

#define LED_LVL_PIN P11
#define LED_LVL_PIN_SET set_P11
#define LED_LVL_PIN_CLR clr_P11


#define RF_TX_PIN P14
#define RF_TX_PIN_SET set_P14
#define RF_TX_PIN_CLR clr_P14

#define TRIG_PIN P17
#define TRIG_PIN_SET set_P17
#define TRIG_PIN_CLR clr_P17
#define ECHO_PIN() P30
#define ECHO_PIN_SET set_P17
#define ECHO_PIN_CLR clr_P17

#define RF_UZ_PIN P04
#define RF_UZ_SET set_P04
#define RF_UZ_CLR clr_P04

#define RF_TM_PIN P15
#define RF_TM_SET set_P15
#define RF_TM_CLR clr_P15


#define DS18B20_PIN() P05
#define DS18B20_PIN_SET set_P05
#define DS18B20_PIN_CLR clr_P05

#define  CMD_SKIP_ROM            0xCC
#define  CMD_CONVERT_T           0x44
#define  CMD_READ_SCRATCHPAD     0xBE

#define sound_velocity 34300  	/* sound velocity in cm per second */
#define period_in_us pow(10,-6)
#define Clock_period 1.085*period_in_us		/* period for clock cycle of 8051*/



#define RELOAD_VALUE_H  (65536-1500)/256
#define RELOAD_VALUE_L  (65536-1500)%256

#define ADC_CHANNEL 0 // ADC channel 0 (P1.2)
#define VDD_VOLTAGE 3300 // The supply voltage in millivolts (e.g., 3.3V)

//========================== VARIABLES =========================================
volatile bit led_on = 0;

volatile unsigned int distance_measurement  = 0;
volatile unsigned int cnt_sec = 0;

float outTemperature = 0.0;
float supplyVoltage = 0.0;
double Bandgap_Voltage = 0.0; 
double Bandgap_Value = 0.0;
double VDD_Voltage = 0.0;

volatile unsigned int batteryValue = 0;
volatile unsigned int temperatureValue = 0;
     
const unsigned char tmr_clbr=194;
const unsigned char poopADDR[3]={20,22}; //

//------------------------------------------------------------
double  Bandgap_Voltage,VDD_Voltage;			//please always use "double" mode for this
unsigned  char xdata ADCdataH[5], ADCdataL[5];
int ADCsumH=0, ADCsumL=0;
unsigned char ADCavgH,ADCavgL;

//========================== BASIC FUNCTIONS ===================================
unsigned char Get_Low_Byte(unsigned int word) {return word - 0x100 * (word / 0x100);}
//------------------------------------------------------------------------------
unsigned char Get_High_Byte(unsigned int word) {return word / 0x100;}
//------------------------------------------------------------------------------
unsigned char set_Bit(char num_val, char num_bit) {return num_val = num_val | (1 << num_bit);}
//------------------------------------------------------------------------------
unsigned char clr_Bit(char num_val, char num_bit) {return  num_val = num_val & ~(1 << num_bit);}
//------------------------------------------------------------------------------
unsigned char tgl_Bit(char num_val, char num_bit) {return num_val = num_val ^ (1 << num_bit);}
//------------------------------------------------------------------------------
unsigned char get_Bit(char num_val, char num_bit) {return num_val = (num_val >> num_bit) & 1; }
//------------------------------------------------------------------------------
unsigned char upd_Bit(char num_val, char num_bit, char num_shift) {
  if (get_Bit(num_val, num_bit)) return (set_Bit(num_val, num_bit+num_shift));
     else return (clr_Bit(num_val, num_bit+num_shift));
}
//------------------------------------------------------------------------------
unsigned char makeIntToHex(unsigned char dec_symbol){
  if (dec_symbol<10) return(dec_symbol+0x30); else
  if (dec_symbol>9 && dec_symbol<16)return(dec_symbol+0x37); else return(0);
}
//========================= MCU FUNCTIONS =====================================
void Init_MCU(){ //Init processes
    clr_EA; // Disable global interrupts during ADC initialization
    
    P00_PushPull_Mode;       //LED TX     
    P01_PushPull_Mode;       //LED STATUS
    P11_PushPull_Mode;       //LED LEVEL   
     
    P04_PushPull_Mode;       //SONAR POWER  
    P15_PushPull_Mode;       //DALLAS POWER
     
    P05_Quasi_Mode;    // Dallas pin
    P14_PushPull_Mode;    // Radio pin config
    P17_Quasi_Mode;       // Set TRIG_PIN (P17) as Quasi-bidirectional mode
    P30_Quasi_Mode;       // Set ECHO_PIN (P30) as Input mode
  
//    ADCM = ADC_CLOCK_P12_DIV_16; // Set ADC clock divider (adjust as needed)
//    ADCCON1 = (ADCCON1 & 0xCF) | 0x00; // Select channel 0 (P1.2) as ADC input
/*     
    TIMER0_MODE1_ENABLE;  // TMOD = 0x01;     // Timer0 mode 1 (16-bit timer)
    set_ET0;              // enable Timer0 interruptTH0 = 0;
    TH0 = 0; 
    TL0 = 0;              // set counter  
    set_EA;               // EA = 1; Enable global interrupt
    set_ET0;              // ET0 = 1;Enable Timer0 interrupt
    set_TR0;             // TR0 = 1;   Start Timer0
 */  
    RH3 = RELOAD_VALUE_H;                       //initial counter values 
    RL3 = RELOAD_VALUE_L;    
    set_ET3;                                    //enable Timer3 interrupt
    set_EA;                                     //enable interrupts
    set_TR3;                                    //Timer3 run

    RF_UZ_CLR;
    RF_TM_CLR; 

    TRIG_PIN_CLR;
    ECHO_PIN_CLR;

}
//-----------------------------------------------------------------------------
void delay_rf(uint8_t ms) {
   uint16_t dly_us;
   while(ms--) {
     dly_us= 510; //410; //340   
     while(dly_us--) {
        _nop_();
     }
  }
}
//-----------------------------------------------------------------------------
void delay_sonar(unsigned int us) {
    while(us--) _nop_();
}
//-----------------------------------------------------------------------------
void delay_us(unsigned int us) {
    while(us--) {
        _nop_();
        _nop_();
        _nop_();
        _nop_();
    }
}
//-----------------------------------------------------------------------------
void delay_ms(unsigned int ms) {
    while(ms--) {
        delay_us(1000);
    }
}
//========================= SONAR FUNCTIONS ===================================
void reset_Echolot(){
     uint16_t cnt_echo=0;
   //  float value_sonar;
     
     TRIG_PIN_SET;           // Generate 10us pulse on TRIG_PIN
     delay_us(10); //delay_sonar(10);
     TRIG_PIN_CLR;
     
     while(!ECHO_PIN());           	/* Waiting for Echo */
     
     
	//TH0 = 0; TL0 = 0; TR0 = 1;  		/* Load registers and timer Starts */
    	while(ECHO_PIN()){
       cnt_echo++; _nop_();
     };    	/* Waiting for Echo goes LOW */
   // 	cnt_echo=0x100*TH0+TL0;            /* Copy Time when echo is received */
   //  TR0 = 0; TH0 = 0; TL0 = 0;   		/* Stop the timer */
	/* calculate distance using timer */
	 // value_sonar =  Clock_period *sound_velocity;
	//distance_measurement = (TL0|(TH0<<8));	/* read timer register for time count */
	distance_measurement = (cnt_echo*34)/200;  /* find distance(in cm) */
     printf ("\n Distance %d =  %e \n", cnt_echo, distance_measurement);   
}
//========================== RADIO FUNCTIONS =======================================
void sendEV1527Code(unsigned long ev1527Code) {
    uint8_t i = 0;
    const unsigned char tmrEVset=4; //350
    LED_TX_PIN_SET;     
    //Send start bit
    RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit
    delay_rf(tmrEVset);
    RF_TX_PIN_CLR;  //RF_TX_PIN = 0; 
    delay_rf(31*tmrEVset);     
    for (i = 0; i < 24; i++) {
        if (ev1527Code & (1UL << i)) {
            RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit   
            delay_rf(3*tmrEVset);
            RF_TX_PIN_CLR;  //RF_TX_PIN = 0; 
            delay_rf(tmrEVset);
        } else {
            RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit 
            delay_rf(tmrEVset);
            RF_TX_PIN_CLR;  //RF_TX_PIN = 0;
            delay_rf(3*tmrEVset);
        }
    }
    LED_TX_PIN_CLR;
}
//--------------------------------------------------------------
void sendEV1527temp() {
    uint8_t i = 0;
    uint8_t j = 0; 
    uint8_t c = 0xAA;
    LED_TX_PIN_SET;     
    //Send start bit
    RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit
    delay_rf(1);
    RF_TX_PIN_CLR;  //RF_TX_PIN = 0; 
    delay_rf(31);     
    for (i=0; i<3; i++){
       if (i==0) c = 0xBA; else c = 0xAA;
       for (j=0;j<8;j++){
            if (get_Bit(c, 7-j)){
                 RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit 
                 delay_rf(3);
                 RF_TX_PIN_CLR;  //RF_TX_PIN = 0; 
                 delay_rf(1);
            } else {
                 RF_TX_PIN_SET;  //RF_TX_PIN = 1; Set the RF transmitter pin HIGH for a '1' bit 
                 delay_rf(1);
                 RF_TX_PIN_CLR;  //RF_TX_PIN = 0; 
                 delay_rf(3);
            }
       }
    }
    RF_TX_PIN_CLR;
    LED_TX_PIN_CLR;
}
//======================== ONE WIRE FUNCTIONS ================================================
bit ds18b20_reset() { //Reset the DS18B20 and check if the device is present
     bit presence = 0;
     uint8_t i_try;
     i_try=0;  presence=1;
     while (presence){
          DS18B20_PIN_CLR; //Set pin 0
          delay_us(480);   //Delay at least 480us
          DS18B20_PIN_SET; //Release data line 
          delay_us(60);    //Waiting for 60us
          presence = DS18B20_PIN();
          delay_us(420);
          i_try++;
          if (i_try>114) return 0;
     }
    return 1; //presence
}
//---------------------------------------------------------------------------------------------
void ds18b20_write_byte(uint8_t dataTemperature) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        DS18B20_PIN_CLR;
        _nop_();
        if (dataTemperature & 0x01) DS18B20_PIN_SET; 
        delay_us(60);
        DS18B20_PIN_SET;
        dataTemperature >>= 1;
    }
}
//---------------------------------------------------------------------------------------------
unsigned char ds18b20_read_byte() { //Write 1 byte of data to the DS18B20
    uint8_t i, dataTemperature = 0;
    for (i = 0; i < 8; i++) {
        dataTemperature >>= 1; 
        DS18B20_PIN_CLR;
        _nop_();
        DS18B20_PIN_SET;
        delay_us(1);
        if (DS18B20_PIN()) {
            dataTemperature |= 0x80;
        }
        delay_us(60);
    }
    return dataTemperature;
}
//---------------------------------------------------------------------------------------------
float read_ds18b20_temperature() {
    unsigned char lsb, msb;
    float temperature;
    LED_LVL_PIN_SET;  
    printf ("\n Start  ds18b20 \n");  
    if (ds18b20_reset()==1){
         ds18b20_write_byte(CMD_SKIP_ROM); // Skip ROM command
         ds18b20_write_byte(CMD_CONVERT_T); // Convert temperature command
    //     delay_ms(750); // DS18B20 conversion time
         while (!DS18B20_PIN());  // DS18B20 conversion time
         printf ("\n Send configuration  ds18b20 \n"); 
    } else {
       printf ("\n Not detected  ds18b20 \n"); 
    }
    

    if (ds18b20_reset()==1){
         ds18b20_write_byte(0xCC); // Skip ROM command
         ds18b20_write_byte(0xBE); // Read Scratchpad command
         lsb = ds18b20_read_byte();
         msb = ds18b20_read_byte();
         temperature = ((msb << 8) | lsb) * 0.0625;
         printf ("\n Temperature = %e \n", temperature); 
    }
    
    LED_LVL_PIN_CLR;   
    return temperature;
}

//======================= ADC Functions ======================================================
void READ_BANDGAP() { 
     UINT8 BandgapHigh,BandgapLow; 
     set_IAPEN; // Enable IAPEN 
     IAPAL = 0x0C; 
     IAPAH = 0x00; 
     IAPCN = 0x04; 
     set_IAPGO; // Trig set IAPGO 
     BandgapHigh = IAPFD; 
     IAPAL = 0x0d;
     IAPAH = 0x00; 
     IAPCN = 0x04; 
     set_IAPGO; // Trig set IAPGO 
     BandgapLow = IAPFD; 
     BandgapLow = BandgapLow&0x0F; 
     clr_IAPEN; // Disable IAPEN 
     Bandgap_Value = (BandgapHigh<<4)+BandgapLow; 
     Bandgap_Voltage = 3072/(0x1000/Bandgap_Value); 
}
//---------------------------------------------------------------------------------------------------------
void ADC_Bypass (void){  // The first three times convert should be bypass
   unsigned char ozc; 
   for (ozc=0;ozc<0x03;ozc++){
     clr_ADCF; 
     set_ADCS; 
     while(ADCF == 0); 
   } 
}
//----------------------------------------------------------------------------------------------------------
void READ_BANDGAP_CUSTOM(){
  UINT8 BandgapHigh,BandgapLow,BandgapMark;
  double Bandgap_Value,Bandgap_Voltage_Temp;
  set_IAPEN;
  IAPCN = READ_UID;
  IAPAL = 0x0d;
  IAPAH = 0x00;
  set_IAPGO;
  BandgapLow = IAPFD;
  BandgapMark = BandgapLow&0xF0;
  if (BandgapMark==0x80){
	BandgapLow = BandgapLow&0x0F;
	IAPAL = 0x0C;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapHigh = IAPFD;
	Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
	Bandgap_Voltage_Temp = Bandgap_Value*3/4;
	Bandgap_Voltage = Bandgap_Voltage_Temp - 33;			//the actually banggap voltage value is similar this value.
  }
  if (BandgapMark==0x00){
	BandgapLow = BandgapLow&0x0F;
	IAPAL = 0x0C;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapHigh = IAPFD;
	Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
	Bandgap_Voltage= Bandgap_Value*3/4;
  }
  if (BandgapMark==0x90){
	IAPAL = 0x0E;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapHigh = IAPFD;
	IAPAL = 0x0F;
	IAPAH = 0x00;
	set_IAPGO;
	BandgapLow = IAPFD;
	BandgapLow = BandgapLow&0x0F;
	Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
	Bandgap_Voltage= Bandgap_Value*3/4;
  }
  clr_IAPEN;
//			printf ("\n BG High = %bX",BandgapHigh); 
//			printf ("\n BG Low = %bX",BandgapLow); 
//			printf ("\n BG ROMMAP = %e",Bandgap_Voltage); 
}
//----------------------------------------------------------------------------
void get_MCU_Voltage(){
    double bgvalue;
    unsigned int i;
    Enable_ADC_BandGap;
    CKDIV = 0x02;	// IMPORTANT!! Modify system clock to 4MHz ,then add the ADC sampling clock base to add the sampling timing.
    for(i=0;i<5;i++){ // All following ADC detect timing is 200uS run under 4MHz.
  	  clr_ADCF;
	  set_ADCS;																
	  while(ADCF == 0);
	  ADCdataH[i] = ADCRH;
	  ADCdataL[i] = ADCRL;
	}		
	CKDIV = 0x00;	// After ADC sampling, modify system clock back to 16MHz to run next code.
	Disable_ADC;
     for(i=2;i<5;i++){// use the last 3 times data to make average 
		ADCsumH = ADCsumH + ADCdataH[i];
		ADCsumL = ADCsumL + ADCdataL[i];
	}				
	ADCavgH = ADCsumH/3;
	ADCavgL = ADCsumL/3;
	bgvalue = (ADCavgH<<4) + ADCavgL;
	VDD_Voltage = (0x1000/bgvalue)*Bandgap_Voltage;
	printf ("\n VDD voltage = %e \n", VDD_Voltage); 
	//Timer0_Delay1ms(500);
	ADCsumH = 0;
	ADCsumL = 0;    
}

//-----------------------------------------------------------------------------------------------------------
void readADCdata(unsigned char adcChannelNumber){
   unsigned int adc_value=0;  
   unsigned char ADCdataH=0;
   unsigned char ADCdataL=0;
   //READ ADC CHANNELS
   switch (adcChannelNumber){   
     case 0:  Enable_ADC_AIN0;  break;   
     case 1:  Enable_ADC_AIN1;  break;
     case 2:  Enable_ADC_AIN2;  break;
     case 3:  Enable_ADC_AIN3;  break;
     case 4:  Enable_ADC_AIN4;  break;   
     case 5:  Enable_ADC_AIN5;  break;  
     case 6:  Enable_ADC_AIN6;  break;
     case 7:  Enable_ADC_AIN7;  break;          
     default: Enable_ADC_AIN0;  break;
   }  
   clr_ADCF;
   set_ADCS;                                
   while(ADCF == 0);
   ADCdataH = ADCRH;
   ADCdataL = ADCRL;
   adc_value=ADCRH << 2 | ADCRL;
   if (adcChannelNumber==0){
      supplyVoltage = (float)adc_value * VDD_VOLTAGE / 1023.0; // Calculate supply voltage in millivolts
      batteryValue=(unsigned int)(1000*supplyVoltage);
   } else {
      temperatureValue=adc_value;
   }
   
   Disable_ADC;  
}
//=============================== LED INFORMATION ===========================================
void ledProc(){
   if (led_on==0){
      led_on=1;
      LED_STS_SET;  
   } else {
      led_on=0;
      LED_STS_CLR; 
   } 
}
//==================================== Power-down Mode Using WKT  ===========================
void sleepPoop(void){
/*
     // Enter power-down mode with wake-up by WKT (Wake-up Timer)
  set_PCON_PD; // Enter power-down mode with WKT wake-up
  set_EWKTEN; // Enable WKT (Wake-up Timer)
  clr_EA; // Disable global interrupts during power-down
  set_WKTF; // Clear the WKT interrupt flag
  clr_WKCON_WKTF; // Clear the WKT flag
  clr_WKCON_WKTR; // Start the WKT (Wake-up Timer)
*/     
}
//================================ UART Mode Usage ===========================================
void SendRsString(void){
  unsigned char poopLOG[22]={'L','=',0,0,0,0,';','T','=',0,0,0,0,';','V','=',0,0,0,0,0x0D,0x0A};  
  uint8_t i, i_high, i_low;
  uint16_t i_temp_h, i_temp_l;
  //Distance
  i_temp_h=distance_measurement/0x100;         i_temp_l=distance_measurement-0x100*i_temp_h;
  i_high=Get_High_Byte(i_temp_h);  i_low=Get_Low_Byte(i_temp_h);      
  poopLOG[2]=makeIntToHex(i_high); poopLOG[3]=makeIntToHex(i_low);  
  i_high=Get_High_Byte(i_temp_l);  i_low=Get_Low_Byte(i_temp_l);      
  poopLOG[4]=makeIntToHex(i_high); poopLOG[5]=makeIntToHex(i_low);     
  //Temperature
  i_temp_h=temperatureValue/0x100; i_temp_l=temperatureValue-0x100*i_temp_h;
  i_high=Get_High_Byte(i_temp_h);  i_low=Get_Low_Byte(i_temp_h);      
  poopLOG[9]=makeIntToHex(i_high); poopLOG[10]=makeIntToHex(i_low);  
  i_high=Get_High_Byte(i_temp_l);  i_low=Get_Low_Byte(i_temp_l);      
  poopLOG[11]=makeIntToHex(i_high); poopLOG[12]=makeIntToHex(i_low);
  //Battery
  i_temp_h=batteryValue/0x100;     i_temp_l=batteryValue-0x100*i_temp_h;
  i_high=Get_High_Byte(i_temp_h);  i_low=Get_Low_Byte(i_temp_h);      
  poopLOG[16]=makeIntToHex(i_high); poopLOG[17]=makeIntToHex(i_low);  
  i_high=Get_High_Byte(i_temp_l);  i_low=Get_Low_Byte(i_temp_l);      
  poopLOG[18]=makeIntToHex(i_high); poopLOG[19]=makeIntToHex(i_low);     
       
       
  for (i = 0; i < 22; i++){
     Send_Data_To_UART0(poopLOG[i]); // delay_us(10);
  }
  delay_ms(1);
}
/************************************************************************************************************
*    TIMER 0 interrupt subroutine
************************************************************************************************************/
//void Timer0_ISR (void) interrupt 1  //interrupt address is 0x000B
void Timer3_ISR (void) interrupt 16 
{
    //TH0 = 0;
    //TL0 = 0;
    clr_TF3; 
    if (cnt_sec<50) cnt_sec++; 
}
/************************************************************************************************************
*    Main function 
************************************************************************************************************/
void main (void){
    uint8_t i=0;
    Init_MCU();
    READ_BANDGAP_CUSTOM();
   //READ_BANDGAP();
    InitialUART0_Timer1(9600); 
    while(1){
      if (cnt_sec>=50){
         cnt_sec=0;   
         ledProc();  
         reset_Echolot();
        // readADCdata(0);
        // readADCdata(1);
         get_MCU_Voltage(); 
         outTemperature = read_ds18b20_temperature();  
         for (i=0; i<8; i++) 
         //  sendEV1527Code(0x808080);
           sendEV1527temp();
//         if (test_key_IN()==0)
//          SendRsString();
      }
      delay_ms(10);
    };
}

