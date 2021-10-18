/*************************************************/
/* MCU_GPIO.h		    			        	 */
/*												 */
/* GPIO Define for PE Support Msensor 8051 Board */
/*												 */
/*************************************************/

#define ISCL        P0_0
#define ISDA        P0_1
#define ISCL2       P1_0
#define ISDA2       P1_1

#define DRDY        P0_2

#define RY1_ON	    P0_3=1;
#define RY2_ON		P0_4=1;
#define RY3_ON		P0_5=1;
#define RY4_ON		P0_6=1;

#define RY1_OFF		P0_3=0;
#define RY2_OFF		P0_4=0;
#define RY3_OFF		P0_5=0;
#define RY4_OFF		P0_6=0;

#define LED1_ON     P1_7=0
#define LED1_OFF    P1_7=1
#define LED2_ON     P3_7=0
#define LED2_OFF    P3_7=1
#define LED3_ON     P0_7=0
#define LED3_OFF    P0_7=1
