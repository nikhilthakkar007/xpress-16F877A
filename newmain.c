/*
 * File:   PIC_GPIO_PWM.c
 * Author: Aswinth
 *
 * Created on 17 October, 2018, 11:59 AM
 */

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 20000000
#define PWM_Frequency 0.05 // in KHz (50Hz)

//TIMER0    8-bit  with 64-bit Prescalar
//$$RegValue = 256-((Delay * Fosc)/(Prescalar*4))  delay in sec and Fosc in hz  ->Substitute value of Delay for calculating RegValue

int POT_val; //variable to store value from ADC
int count; //timer variable 
int T_TOTAL = (1/PWM_Frequency)/10; //calculate Total Time from frequency (in milli sec)) //2msec
int T_ON=0; //value of on time
int Duty_cycle; //Duty cycle value

void ADC_Initialize() //Prepare the ADC module 
{
  ADCON0 = 0b01000001; //ADC ON and Fosc/16 is selected
  ADCON1 = 0b11000000; // Internal reference voltage is selected
}

unsigned int ADC_Read(unsigned char channel) //Read from ADC
{
  ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
  ADCON0 |= channel<<3; //Setting the required Bits
  __delay_ms(2); //Acquisition time to charge hold capacitor
  GO_nDONE = 1; //Initializes A/D Conversion
  while(GO_nDONE); //Wait for A/D Conversion to complete
  return ((ADRESH<<8)+ADRESL); //Returns Result
}

void interrupt timer_isr()
{  
    if(TMR0IF==1) // Timer flag has been triggered due to timer overflow -> set to overflow for every 0.1ms
    {
        TMR0 = 248;     //Load the timer Value
        TMR0IF=0;       // Clear timer interrupt flag
        count++; //Count increments for every 0.1ms -> count/10 will give value of count in ms 
    } 
    
    if (count <= (T_ON) )
        RD1=1;
    else
        RD1=0;
    
    if (count >= (T_TOTAL*10) )
        count=0;
}

void main()
{    
/*****Port Configuration for Timer ******/
    OPTION_REG = 0b00000101;  // Timer0 with external freq and 64 as prescalar // Also Enables PULL UPs
    TMR0=248;       // Load the time value for 0.0001s; delayValue can be between 0-256 only
    TMR0IE=1;       //Enable timer interrupt bit in PIE1 register
    GIE=1;          //Enable Global Interrupt
    PEIE=1;         //Enable the Peripheral Interrupt
    /***********______***********/   
 
    /*****Port Configuration for I/O ******/
    TRISD = 0x00; //Instruct the MCU that all pins on PORT D are output 
    PORTD=0x00; //Initialize all pins to 0
    /***********______***********/   
    
    ADC_Initialize();

    while(1)
    {
       POT_val = (ADC_Read(0)); //Read the value of POT using ADC
       
       Duty_cycle = (POT_val * 0.0976); //Map 0 to 1024 to 0 to 100
       
       T_ON = ((Duty_cycle * T_TOTAL)*10 / 100); //Calculate On Time using formulae unit in milli seconds 

       __delay_ms(100);     
       
    }
    
}