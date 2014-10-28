#include <ioCC2530.h>
//--------------------------------------------------------------------------
// DEFINE MASKS
//--------------------------------------------------------------------------



//------------------------------------------------------------------------------
// MAIN
//------------------------------------------------------------------------------
void main()
{
  CLKCONCMD &= 0x00;//Set system clock to 32MHZ => set bit 6 to 1 => [-1-- ----]
  while(CLKCONSTA&0x40);//waiting for clock to become stable
  //-------------------------------------------------------------------------
  // TEST
  //-------------------------------------------------------------------------
  P1SEL &= ~ 0x01;  
  P1DIR |= 0x01;
  P1_0=1;
  //-------------------------------------------------------------------------
  // CONFIGURE ADC
  //-------------------------------------------------------------------------
  P0SEL |=  0xE0;//SET P0_7, 6, 5 to peripheral [111-  ----]
  P0DIR &= ~0xE0;//SET P0_7, 6, 5 to input [000-  ----]
  APCFG |= 0xE0;// Analog Peripheral I/O enable [111- ----] pg.85, 134
  ADCCON2 |= 0x20;//ADCCON2.SCH Set 256 decimation rate(10bit) [--1- ---] 7 pg. 134
  //ADCCON3 single conversion from a channel
  //ADCCON2.SDIV
  //digital conversion result is available in ADCH and ADCL when ADCCON1.EOC is set to 1.
  ADCCON2 = 0x3F; 
  ADCCON1 = 0x73; 
  while(!(ADCCON1 & 0x80)); 
  v = ADCL; \ 
  v |= (((unsigned int)ADCH) << 8); 
            
}