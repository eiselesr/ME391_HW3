#include <ioCC2530.h>
//--------------------------------------------------------------------------
// DEFINE MASKS
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
// MAIN
//--------------------------------------------------------------------------
void main()
{
  P1SEL &= ~ 0x01;  
  P1DIR |= 0x01;
  P1_0=1;
}