/**
  PWM Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    pwm.c

  @Summary
    This is the generated driver implementation file for the PWM driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for PWM.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.166.0
        Device            :  dsPIC33EV256GM102      
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.41
        MPLAB 	          :  MPLAB X v5.30
*/

/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include "pwm.h"

/**
  Section: Driver Interface
*/

void PWM_Initialize (void)
{
    // PCLKDIV 32; 
    PTCON2 = 0x05;
    // PTPER 65535; 
    PTPER = 0xFFFF;
    // SEVTCMP 0; 
    SEVTCMP = 0x00;
    // MDC 0; 
    MDC = 0x00;
    // CHOPCLK 0; CHPCLKEN disabled; 
    CHOP = 0x00;
    // PWMKEY 0; 
    PWMKEY = 0x00;
    // MDCS Primary; FLTIEN disabled; CAM Edge Aligned; DTC Positive dead time for all Output modes; TRGIEN disabled; XPRES disabled; ITB Master; IUE disabled; CLIEN disabled; DTCP disabled; 
    PWMCON1 = 0x00;
    // MDCS Primary; FLTIEN disabled; CAM Edge Aligned; DTC Positive dead time for all Output modes; TRGIEN disabled; XPRES disabled; ITB Master; IUE disabled; CLIEN disabled; DTCP disabled; 
    PWMCON2 = 0x00;
    // MDCS Primary; FLTIEN disabled; CAM Edge Aligned; DTC Dead-time function is disabled; TRGIEN disabled; XPRES disabled; ITB Master; IUE disabled; CLIEN disabled; DTCP disabled; 
    PWMCON3 = 0x80;
    //FLTDAT PWM1L Low, PWM1H Low; SWAP disabled; OVRENH disabled; PENL disabled; PMOD Complementary Output Mode; OVRENL disabled; OSYNC disabled; POLL disabled; PENH disabled; CLDAT PWM1L Low, PWM1H Low; OVRDAT PWM1L Low, PWM1H Low; POLH disabled; 
    __builtin_write_PWMSFR(&IOCON1, 0x00, &PWMKEY);
    //FLTDAT PWM2L Low, PWM2H Low; SWAP disabled; OVRENH disabled; PENL disabled; PMOD Complementary Output Mode; OVRENL disabled; OSYNC disabled; POLL disabled; PENH disabled; CLDAT PWM2L Low, PWM2H Low; OVRDAT PWM2L Low, PWM2H Low; POLH disabled; 
    __builtin_write_PWMSFR(&IOCON2, 0x00, &PWMKEY);
    //FLTDAT PWM3L Low, PWM3H Low; SWAP disabled; OVRENH disabled; PENL enabled; PMOD Redundant Output Mode; OVRENL disabled; OSYNC disabled; POLL disabled; PENH disabled; CLDAT PWM3L Low, PWM3H Low; OVRDAT PWM3L Low, PWM3H Low; POLH disabled; 
    __builtin_write_PWMSFR(&IOCON3, 0x4400, &PWMKEY);
    //FLTPOL disabled; CLPOL disabled; CLSRC FLT1; CLMOD disabled; FLTMOD PWM1H, PWM1L pins to FLTDAT values- Latched; IFLTMOD disabled; FLTSRC FLT32; 
    __builtin_write_PWMSFR(&FCLCON1, 0xF8, &PWMKEY);
    //FLTPOL disabled; CLPOL disabled; CLSRC FLT1; CLMOD disabled; FLTMOD PWM2H, PWM2L pins to FLTDAT values- Latched; IFLTMOD disabled; FLTSRC FLT32; 
    __builtin_write_PWMSFR(&FCLCON2, 0xF8, &PWMKEY);
    //FLTPOL disabled; CLPOL disabled; CLSRC FLT1; CLMOD disabled; FLTMOD Fault input is disabled; IFLTMOD disabled; FLTSRC CMP1; 
    __builtin_write_PWMSFR(&FCLCON3, 0x43, &PWMKEY);
    // PDC1 0; 
    PDC1 = 0x00;
    // PDC2 0; 
    PDC2 = 0x00;
    // PDC3 6543; 
    PDC3 = 0x198F;
    // PHASE1 0; 
    PHASE1 = 0x00;
    // PHASE2 0; 
    PHASE2 = 0x00;
    // PHASE3 0; 
    PHASE3 = 0x00;
    // DTR1 0; 
    DTR1 = 0x00;
    // DTR2 0; 
    DTR2 = 0x00;
    // DTR3 0; 
    DTR3 = 0x00;
    // ALTDTR1 0; 
    ALTDTR1 = 0x00;
    // ALTDTR2 0; 
    ALTDTR2 = 0x00;
    // ALTDTR3 0; 
    ALTDTR3 = 0x00;
    // TRGCMP 0; 
    TRIG1 = 0x00;
    // TRGCMP 0; 
    TRIG2 = 0x00;
    // TRGCMP 0; 
    TRIG3 = 0x00;
    // TRGDIV 1; TRGSTRT 0; 
    TRGCON1 = 0x00;
    // TRGDIV 1; TRGSTRT 0; 
    TRGCON2 = 0x00;
    // TRGDIV 1; TRGSTRT 0; 
    TRGCON3 = 0x00;
    // PWMCAP 0; 
    PWMCAP1 = 0x00;
    // PWMCAP 0; 
    PWMCAP2 = 0x00;
    // PWMCAP 0; 
    PWMCAP3 = 0x00;
    // BPLL disabled; BPHH disabled; BPLH disabled; BCH disabled; FLTLEBEN disabled; PLR disabled; CLLEBEN disabled; BCL disabled; PLF disabled; PHR disabled; BPHL disabled; PHF disabled; 
    LEBCON1 = 0x00;
    // BPLL disabled; BPHH disabled; BPLH disabled; BCH disabled; FLTLEBEN disabled; PLR disabled; CLLEBEN disabled; BCL disabled; PLF disabled; PHR disabled; BPHL disabled; PHF disabled; 
    LEBCON2 = 0x00;
    // BPLL disabled; BPHH disabled; BPLH disabled; BCH disabled; FLTLEBEN disabled; PLR disabled; CLLEBEN disabled; BCL disabled; PLF disabled; PHR disabled; BPHL disabled; PHF disabled; 
    LEBCON3 = 0x00;
    // LEB 0; 
    LEBDLY1 = 0x00;
    // LEB 0; 
    LEBDLY2 = 0x00;
    // LEB 0; 
    LEBDLY3 = 0x00;
    // CHOPLEN disabled; CHOPHEN disabled; BLANKSEL No state blanking; CHOPSEL No state blanking; 
    AUXCON1 = 0x00;
    // CHOPLEN disabled; CHOPHEN disabled; BLANKSEL No state blanking; CHOPSEL No state blanking; 
    AUXCON2 = 0x00;
    // CHOPLEN disabled; CHOPHEN disabled; BLANKSEL No state blanking; CHOPSEL No state blanking; 
    AUXCON3 = 0x00;

    // SYNCOEN disabled; SEIEN disabled; SESTAT disabled; SEVTPS 1; SYNCSRC SYNCI1; SYNCEN disabled; PTSIDL disabled; PTEN enabled; EIPU disabled; SYNCPOL disabled; 
    PTCON = 0x8000;
}

void __attribute__ ((weak)) PWM_SpecialEvent_CallBack(void)
{
    // Add Application code here
}

void PWM_SpecialEvent_Tasks ( void )
{
    if(IFS3bits.PSEMIF)
    {
        // PWM SpecialEvent callback function 
        PWM_SpecialEvent_CallBack();

        // clear the PWM SpecialEvent interrupt flag
        IFS3bits.PSEMIF = 0;
    }
}

void __attribute__ ((weak)) PWM_Generator1_CallBack(void)
{
    // Add Application code here
}

void PWM_Generator1_Tasks ( void )
{
	if(IFS3bits.PSEMIF)
	{
		// PWM Generator1 callback function 
		PWM_Generator1_CallBack();
		
		// clear the PWM Generator1 interrupt flag
		IFS5bits.PWM1IF = 0;
	}
}

void __attribute__ ((weak)) PWM_Generator2_CallBack(void)
{
    // Add Application code here
}

void PWM_Generator2_Tasks ( void )
{
	if(IFS3bits.PSEMIF)
	{
		// PWM Generator2 callback function 
		PWM_Generator2_CallBack();
		
		// clear the PWM Generator2 interrupt flag
		IFS5bits.PWM2IF = 0;
	}
}

void __attribute__ ((weak)) PWM_Generator3_CallBack(void)
{
    // Add Application code here
}

void PWM_Generator3_Tasks ( void )
{
	if(IFS3bits.PSEMIF)
	{
		// PWM Generator3 callback function 
		PWM_Generator3_CallBack();
		
		// clear the PWM Generator3 interrupt flag
		IFS6bits.PWM3IF = 0;
	}
}


/**
 End of File
*/
