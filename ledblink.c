#include "DSP28x_Project.h" // DSP28x Headerfile
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/wdog.h"
#ifdef _FLASH
memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
(size_t)&RamfuncsLoadSize);
#endif
void main()
{
WDOG_Handle myWDog;
myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
WDOG_disable(myWDog);
CLK_Handle myClk;
PLL_Handle myPll;
myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
CLK_setOscSrc(myClk, CLK_OscSrc_Internal);
PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);
GPIO_Handle myGpio;
myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_1, GPIO_1_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_2, GPIO_2_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_3, GPIO_3_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);
GPIO_setHigh(myGpio, GPIO_Number_0);
GPIO_setHigh(myGpio, GPIO_Number_1);
GPIO_setHigh(myGpio, GPIO_Number_2);
GPIO_setHigh(myGpio, GPIO_Number_3);
while(1)
{
GPIO_setLow(myGpio, GPIO_Number_0);
DELAY_US(1000000);
GPIO_setHigh(myGpio, GPIO_Number_0);
DELAY_US(1000000);
}
}
