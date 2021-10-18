/******************************************************************************/
/*                                                                            */
/*       SERIAL.C:  Interrupt Controlled Serial Interface                     */
/*                                                                            */
/******************************************************************************/

#include <AT89X52.h>
#include <INTRINS.H>                 /* special function register 8052       */

#define  OLEN  32                      /* size of serial transmission buffer   */
unsigned char  ostart;                /* transmission buffer start index      */
unsigned char  oend;                  /* transmission buffer end index        */
idata    char  outbuf[OLEN];          /* storage for transmission buffer      */

#define  ILEN  16                      /* size of serial receiving buffer      */
unsigned char  istart;                /* receiving buffer start index         */
unsigned char  iend;                  /* receiving buffer end index           */
idata    char  inbuf[ILEN];           /* storage for receiving buffer         */

bit   CheckCommandFlag;

static bit   sendfull;                /* flag: marks transmit buffer full     */
static bit   sendactive;              /* flag: marks transmitter active       */ 
/******************************************************************************/
/*       delay funtion                                                        */
/******************************************************************************/
void delay(unsigned int end_count)		 //15000 ~0.1sec
{unsigned int start_count;
for(start_count=0; start_count<end_count; start_count++)	 _nop_ ();	
}
/******************************************************************************/
/*       putbuf:  write a character to SBUF or transmission buffer            */
/******************************************************************************/
static void putbuf (unsigned char c)  {
  if (!sendfull)  {                   /* transmit only if buffer not full     */
    ES = 0;                           /* disable serial interrupt             */      
    if (!sendactive)  {  /* if transmitter not active:           */
      sendactive = 1;                 /* transfer the first character direct  */
	  TI = 0; 
      SBUF = c;                       /* to SBUF to start transmission        */
	 
    }
    else  {                           /* otherwize:                           */
      outbuf[oend++ & (OLEN-1)] = c;  /* transfer char to transmission buffer */
      if (((oend ^ ostart) & (OLEN-1)) == 0)  sendfull = 1;
    }                                 /* set flag if buffer is full           */
	ES = 1;                           /* enable serial interrupt              */      
  }
}

/******************************************************************************/
/*       putchar:  interrupt controlled putchar function                      */
/******************************************************************************/
char putchar (unsigned char c)  {
  
  while (sendfull)  {                 /* wait for transmission buffer empty   */

  }
  putbuf (c);                         /* send character                       */
  return (c);                         /* return character: ANSI requirement   */
}

/******************************************************************************/
/*         putline:  write char line to transmission buffer for SBUF          */
/* Example:																	  */
/*		    															      */
/* char ARRAY[]="ABCDE";													  */
/* putline(ARRAY,5);     "5" is variable for datacount (A to E are 5 letters) */
/*																			  */
/******************************************************************************/
void putline (char idata Dataline[], unsigned char DataCount)  {
	 unsigned char index=0;
	 while(DataCount!=0) 
	 {
	 putchar(Dataline[index++]);
	 DataCount--;
	 }
}

/******************************************************************************/
/*       GetSerialCount()                                                     */
/******************************************************************************/
unsigned char GetSerialCount() {
       
	   return (iend);                         
}

/******************************************************************************/
/*      GetCommnadByte()                                                     */
/******************************************************************************/
unsigned char GetCommandByte(unsigned char index) {
       
	   return (inbuf[index]);                         
}

/******************************************************************************/
/*       serial:  serial receiver / transmitter interrupt                     */
/*   require global variable: static bit   CheckCommand                       */
/******************************************************************************/
void serial (void) interrupt 4 using 2  { /* use registerbank 2 for interrupt */

   if (RI)  {                          /* if receiver interrupt               */

    ES=0;	//disabel interrupt

     if (istart != ILEN) 
	  {
      inbuf[istart++] = SBUF; 
	   if (inbuf[istart-1]==0x0A){
	    if (inbuf[istart-2]==0x0D){ 
	       CheckCommandFlag=1;              //Set CheckCommandFlg, if the ending chars are received 
	       iend=istart;
	     istart=0;} }
	  } /* read character */
     else
	  {
	  istart =0;
	  }/* Reset Overflow */	                                                            
     

	RI=0;    /* clear interrupt request flag        */

	ES=1;	 //enable interrupt
	 
  }//Revised Stronger Version
  
  if (TI)  {                         /* if transmitter interrupt              */

    TI = 0;                          /* clear interrupt request flag          */
    if (ostart != oend)  {           /* if characters in buffer and           */
        SBUF = outbuf[ostart++ & (OLEN-1)];        /* transmit character      */
        sendfull = 0;                /* clear 'sendfull' flag                 */ 
    }
    else sendactive = 0;             /* if all transmitted clear 'sendactive' */
  }

} 




