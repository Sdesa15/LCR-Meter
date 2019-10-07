// LCR Meter
// Supritha Desa

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include "tm4c123gh6pm.h"
#include <stdio.h>
#include <stdlib.h>
#include "hw_types.h"
#include "hw_nvic.h"

#define GREEN_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define RED_LED          (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define MAX_CHARS         80
#define MEAS_LR_PE2      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define HIGHSIDE_R_PD3   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define MEAS_C_PE3       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define LOWSIDE_R_PD2    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define INTEGRATE_PE1    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))



char str[MAX_CHARS];
char str1[20],str2[20],str3[20],str4[20],str5[20];
char cmd_str[MAX_CHARS];
uint8_t argc;
uint8_t pos[5];
char type[5];

uint32_t time = 0;
void Comp0_isr();
float ct;
char c[100];
float dut2volt=0.0,dut1volt=0.0;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;


    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOD|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOB;

    //Port configurations for MEAS_LR, MEAS_C, INTEGRATE pins(port E2,E3,E1 respectively)
    GPIO_PORTE_DIR_R |= 0x0E;
    GPIO_PORTE_DR2R_R |=0x0E;
    GPIO_PORTE_DEN_R |=0x0E;

    //Port configurations for HIGHSIDE_R and LOWSIDE_R pins(port D2,D3)
    GPIO_PORTD_DIR_R |=0x0C;
    GPIO_PORTD_DR2R_R|= 0x0C;
    GPIO_PORTD_DEN_R |= 0x0C;

    // Configure RED Led and Green LED
    GPIO_PORTF_DIR_R |= 0x0A;  // make bit 1 an outputs
    GPIO_PORTF_DR2R_R |= 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x0A;  // enable LED
    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module



    //configuring ADC0 and ADC1 module for DUT2 and Dut1, port pin PB4,PB5 and using AIN10, AIN11
    SYSCTL_RCGCADC_R =0X03;                          // turn on ADC module 0 clocking
    GPIO_PORTB_AFSEL_R =0x30;                        // select alternative functions for AN10 and AN11
    GPIO_PORTB_DEN_R &= ~0x30;                       // turn off digital operation on pin PB4 and PB5
    GPIO_PORTB_AMSEL_R= 0x30;                        // turn on analog operation on pin PB4
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0X0A;                            // set first sample to AN10
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation


    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 0X0B;                            // set first sample to AN11
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_TAV_R = 0;                               // zero counter for first period

    NVIC_EN3_R &= ~(1 << (INT_WTIMER5A-16-96));      // turn-off interrupt 120 (WTIMER5A)
    //Comparator Configuration
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;

    GPIO_PORTC_DEN_R &=~(0x80);       //TURN OFF DIGTITAL DATA
    GPIO_PORTC_AMSEL_R |= 0x80;      //TURN ON ANALOG DATA
    GPIO_PORTC_AFSEL_R |= 0x80;   //TURN ON ALTERNATE FUCNTION ON PORT C
    //COMPARATOR CONFIGURATION
    COMP_ACREFCTL_R =0x000020F; // set reference to 2.469 v
    COMP_ACCTL0_R = 0x0000040A; //Rising edge and comparator output invert
    COMP_ACINTEN_R|=0x01;
}




// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
 while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

//isCommand function compares whether user entered string and argument matches with the desired string and argcount
int isCommand(char *comp_str, uint8_t argcount)
{
    uint8_t c;
    // convert the user string to lower cases before comparison
    for (c=0;str[c]!= NULL;c++)
    {
        cmd_str[c]=str[c];
        cmd_str[c]= tolower(cmd_str[c]);
    }

    // compare the string
    if(strcmp(comp_str,cmd_str)==0)
    {
        //check if the argument count matches
        if(argcount<=argc-1)
            {
            return 1;
            }
    }

  return 0;

}

//get value function returns the number
int get_value(uint8_t Val)
{
    char*num1_str;
    char* num2_str;
    if (type[Val]=='a')
      {
        num1_str = (&str[pos[Val]]) ;
        return atoi(*num1_str);
      }
      else
      {
          num2_str = (&str[pos[Val]]) ;
          return atoi(*num2_str);
      }

}

//Get string function extracts the string from the required position in the user  entered string
char* get_string(uint8_t Val)
{
    if (type[Val]=='a')
    {
      return(&str[pos[Val]]);
    }
    else
    {
        return "error";
    }

}

int readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

int readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}

void test_step2(char *test_str)
{
    putsUart0("\r\n");
    putsUart0(str); //to test step 2, display the str back to console
}

//Calculate voltage across DUT2 and DUT1 and display measured voltage
void Voltage_DUT2_DUT1()
{
    float Voltage=0.0, Voltage1=0.0,Voltage2=0.0;
    double DUT1raw=0.0,DUT2raw=0.0;
    GREEN_LED=1;
    waitMicrosecond(500000);
    GREEN_LED=0;
    DUT1raw = readAdc1Ss3(); //read raw value
    Voltage1 = (DUT1raw / 4096.0)* 3.3;  //calculate voltage

    DUT2raw = readAdc0Ss3(); //read raw value
    Voltage2 = (DUT2raw / 4096.0) * 3.3;  //calculate voltage
    Voltage =Voltage2-Voltage1;


    //display raw ADC value and voltage FOR DUT1
    putsUart0("\r\n Raw DUT2 Value:");
    sprintf(str1, "%lf", DUT2raw);
    putsUart0(str1);

    putsUart0("\r\n Dut2 Voltage:");
    sprintf(str2, "%f", Voltage2);
    putsUart0(str2);
    //display raw ADC value and voltage FOR DUT2
    putsUart0("\r\n Raw DUT1 Value:");
    sprintf(str3, "%lf", DUT1raw);
    putsUart0(str3);

    putsUart0("\r\n Dut1 Voltage:");
    sprintf(str4, "%f", Voltage1);
    putsUart0(str4);

    // The Difference Voltage
    putsUart0("\r\n Difference Voltage:");
    sprintf(str5, "%f", Voltage);
    putsUart0(str5);

}
//Resistance calculation function
void measure_resistance()
{

    char resistance[100];
    float r ;
    //Deintegrate Commands
    MEAS_LR_PE2=0;
    INTEGRATE_PE1=1;
    LOWSIDE_R_PD2=1;
    HIGHSIDE_R_PD3=0;
    MEAS_C_PE3=0;
    waitMicrosecond(4000000);
    WTIMER5_TAV_R = 0;
    //integrate commands
    LOWSIDE_R_PD2=0;
    MEAS_LR_PE2=1;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN; //ENABLING TIMER
    NVIC_EN0_R |= (1 << (INT_COMP0-16));
    waitMicrosecond(3000000);
   //y= (51967/1000)x+747 equation from two point calculator
    r = (time-747);
    r= r/51.967;

    sprintf(resistance, "%lf", r);
    putsUart0("\n Resistance:");
    putsUart0(resistance);
    putsUart0("ohms\n");
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;


}
//Comparator ISR function
void Comp0_Isr()
{

 time= WTIMER5_TAV_R;
 WTIMER5_TAV_R=0;
 COMP_ACMIS_R=0x01;
 NVIC_EN0_R &= ~(1 << (INT_COMP0-16));

}

//capacitance calculation function
void measure_capacitance()
{
     //char t[100];

     MEAS_LR_PE2=0;
     INTEGRATE_PE1=0;
     MEAS_C_PE3=1;
     LOWSIDE_R_PD2=1;
     HIGHSIDE_R_PD3=0;

     waitMicrosecond(15000000);

     LOWSIDE_R_PD2=0;
     HIGHSIDE_R_PD3=1;

     WTIMER5_TAV_R = 0;
     WTIMER5_CTL_R |= TIMER_CTL_TAEN;

     NVIC_EN0_R |= (1 << (INT_COMP0-16));
     waitMicrosecond(15000000);

     if(time >=2000 && time <57000)
     {
       ct=  (time-1895)*0.000001 ;
       ct=ct/4.41;

     }
     else
     {
     ct = (time+16971);
     ct= ct/5879750;
     }
     sprintf(c, "%f", ct);
     putsUart0("\n capacitance:");
     putsUart0(c);
     putsUart0("MicroFarad\n");
     /*sprintf(t, "%u", time);
     putsUart0("\r \n capacitance:");
     putsUart0(t);*/
     WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;


}

void measure_inductance()
{

    char I[100];
    char tp[100];
    float ind=0.0;
    memset(I,'\0',sizeof(I));

    MEAS_LR_PE2=0;
    INTEGRATE_PE1=0;
    MEAS_C_PE3=1;
    LOWSIDE_R_PD2=1;
    HIGHSIDE_R_PD3=0;
    waitMicrosecond(2000000);

    HIGHSIDE_R_PD3=0;
    MEAS_C_PE3=0;
    INTEGRATE_PE1=0;
    MEAS_LR_PE2=1;
    LOWSIDE_R_PD2=1;

    WTIMER5_TAV_R = 0;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN0_R |= (1 << (INT_COMP0-16));
    waitMicrosecond(50000);

     if(time >=20 && time <=80)
     {
         ind =time *1.1;
     }
     else if(time >=90 && time <= 160 )
     {
         ind =time/1.2;
     }
     else if(time>=500 && time<=620)
     {

         ind = time/1.5;
     }
     else
     {
         ind = time;
     }

     sprintf(I, "%f", ind);
     putsUart0("\r\nInductance:");
     putsUart0(I);
     putsUart0("microhenry\n");
     sprintf(tp, "%u", time);
     putsUart0("\r\n time value:");
     putsUart0(tp);
     WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
     //To ensure voltages across dut1 and dut2 are 0.02mv
     MEAS_LR_PE2=0;
     INTEGRATE_PE1=0;
     MEAS_C_PE3=1;
     LOWSIDE_R_PD2=1;
     HIGHSIDE_R_PD3=0;


}
// Measure ESR
void measure_esr()
{

    char effsr[100];
    double raw1=0.0,raw2=0.0;
    double esr =0.0;


    MEAS_LR_PE2=0;
    INTEGRATE_PE1=0;
    MEAS_C_PE3=1;
    LOWSIDE_R_PD2=1;
    HIGHSIDE_R_PD3=0;
    waitMicrosecond(2000000);

    HIGHSIDE_R_PD3=0;
    MEAS_C_PE3=0;
    INTEGRATE_PE1=0;
    MEAS_LR_PE2=1;
    LOWSIDE_R_PD2=1;

    raw1 = readAdc0Ss3(); //read raw value
    dut2volt = (raw1 / 4096.0)* 3.3;  //calculate voltage
    raw2 = readAdc1Ss3(); //read raw value
    dut1volt = (raw2 / 4096.0)* 3.3;  //calculate voltage

    /* using voltage divider formula vout=vin(33/(33+esr)), which can be rearranged as esr=((vin-vout)*33/vout)*/
     esr = (33 *(dut1volt-dut2volt))/dut2volt;
     sprintf(effsr, "%f", esr);
     putsUart0(" \n ESR:");
     putsUart0(effsr);
     putsUart0(" in ohms \n:");
     //To ensure voltages across dut1 and dut2 are 0.02mv
     MEAS_LR_PE2=0;
     INTEGRATE_PE1=0;
     MEAS_C_PE3=1;
     LOWSIDE_R_PD2=1;
     HIGHSIDE_R_PD3=0;

}

//perform reset function
void Reset()
{

    /*Reset is performed using Application Interrupt reset control
      to write to this function VECTKEY field must be written or the write is
    ignored, system reset request is selected to reset core and all on chip peripherals.
    used macros for hardware access */

    HWREG(NVIC_APINT)=NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ;


}

/*Function to select command to perform desired operation.
To Compare Strings without Case Sensitivity, strcasecmp is used*/
void select_command()
{

    // if command is measure
    if(isCommand("measure", 1))
    {
        char *vstr = get_string(1);
        //if the desired operation is to measure voltage

        if (!strcasecmp(vstr, "VOLTAGE"))
        {
            Voltage_DUT2_DUT1();
        }
        //if the desired operation is to measure resistance
        else if (!strcasecmp(vstr, "R"))
        {
            measure_resistance();
        }
        //if the desired operation is to measure capacitance
        else if (!strcasecmp(vstr, "C"))
        {
            measure_capacitance();

        }
        //if the desired operation is to measure inductance
        else if (!strcasecmp(vstr, "I"))
        {

            measure_inductance();

        }
        //if the desired operation is to measure esr
        else if (!strcasecmp(vstr, "esr"))
        {

            measure_esr();

       }
    }
    //if the desired operation is to perform reset
    else if(isCommand("reset",0))

    {
        Reset() ;

    }
    //if the desired operation is to turn particular port pins ON/OFF
    else if (isCommand("measc", 1))
    {
       char *str1 = get_string(1);
        if(!strcasecmp(str1, "on"))
        {
            //to ensure both measlr and measc are not ON,turning off the other when one is ON
            MEAS_LR_PE2 = 0;
            MEAS_C_PE3 = 1;
            // Turn GREEN_LED for 500msec
             GREEN_LED = 1;
             waitMicrosecond(500000);
             GREEN_LED = 0;
        }
        else
        {
            MEAS_LR_PE2 = 0;
            MEAS_C_PE3 = 0;
        }
    }
    else if (isCommand("measlr", 1))
     {
           char *str1 = get_string(1);
            if(!strcasecmp(str1, "on"))
            {
                //to ensure both measlr and measc are not ON,turning off the other when one is ON
                MEAS_C_PE3 = 0;
                MEAS_LR_PE2 = 1;
                // Turn RED_LED for 500msec
                 RED_LED = 1;
                 waitMicrosecond(500000);
                 RED_LED = 0;


            }
            else
            {
                MEAS_C_PE3 = 0;
                MEAS_LR_PE2 = 0;
            }
       }
    else if (isCommand("highsider", 1))
     {
           char *str1 = get_string(1);
            if(!strcasecmp(str1, "on"))
            {

                HIGHSIDE_R_PD3 = 1;

            }
            else
            {
                HIGHSIDE_R_PD3 = 0;
            }
       }

    else if (isCommand("lowsider", 1))
     {
           char *str1 = get_string(1);
            if (!strcasecmp(str1, "on"))
            {


                LOWSIDE_R_PD2 = 1;

            }
            else
            {
                LOWSIDE_R_PD2 = 0;
            }
       }
    else if (isCommand("inthigh", 1))
       {
             char *str1 = get_string(1);
              if(!strcasecmp(str1, "on"))
              {
                  INTEGRATE_PE1 = 1;
                  HIGHSIDE_R_PD3 = 1;
              }
              else
              {
                  INTEGRATE_PE1 = 0;
                  HIGHSIDE_R_PD3 = 0;
              }
         }
    else if (isCommand("intlow", 1))
       {
             char *str1 = get_string(1);
              if(!strcasecmp(str1, "on"))
              {

                  INTEGRATE_PE1 = 1;
                  LOWSIDE_R_PD2 = 1;
              }
              else
              {
                  INTEGRATE_PE1 = 0;
                  LOWSIDE_R_PD2 = 0;
              }
         }
    else if (isCommand("integrate", 1))
         {
               char *str1 = get_string(1);
                if(!strcasecmp(str1, "on"))
                {

                    INTEGRATE_PE1=1;


                }
                else
                {
                    INTEGRATE_PE1 = 0;

                }
           }
    //if it is none of the above operation/command ,print invalid command
    else
        {
        putsUart0("\r\n Invalid Command:");
        }


}

int main(void)
{
    uint8_t i,k;
    // Initialize hardware
    initHw();

    // Turn RED_LED for 500msec
    RED_LED = 1;
    waitMicrosecond(500000);
    RED_LED = 0;

    while(1)
    {

          uint8_t count = 0;
          pos[5] =0;
          type[5]=0;
          argc=0;
          memset(cmd_str,'\0',sizeof(cmd_str));
          //Get string from terminal
          putsUart0("\n");
          putsUart0("\rEnter the command:\r\n");


          while(count<MAX_CHARS)
          {

             char l_ch = getcUart0();
             if(l_ch == 8)   // check if the entered character is backspace
             {
                if(count>0)
                {
                   count--;
                 }
             }
             else if(l_ch >=32)  // check if entered character is greater than or equal to space
             {
                str[count++]=l_ch;
             }
             else if(l_ch==13)  //check if entered character is equal to carriage return
             {
                str[count]=0x00;
                break;

             }
             else
             {

             }

          }
          //print error if the length of the string is greater than max chars
          if(strlen(str) > MAX_CHARS)
          {
                   // Turn GREEN_LED for 500msec
                   GREEN_LED = 1;
                   waitMicrosecond(500000);
                   GREEN_LED = 0;

                   putsUart0("\r Error :Too many characters \n");
          }

         for(i=0;i<=strlen(str);i++)
         {

             if((str[i]>=65 && str[i] <=90)||(str[i]>=97 && str[i]<=122))  //Analyze global string if its a character or number
             {
                type[argc]='a';                                             //Assign type as alphabet
                pos[argc]=i;                                                 //store the position
                argc=argc+1;                                                //Increment the argument count
                while((str[i+1]>=65 && str[i+1] <=90)||(str[i+1]>=97 && str[i+1]<=122)) // move the loop till the end of the alphabet/character
                {
                   i=i+1;
                }

             }

              if((str[i]>=48)&& (str[i] <=57))   //Analyze global string if its a character or number
              {
                     type[argc]='n';                     //Assign type as number
                     pos[argc]=i;                        //store the position
                     argc=argc+1;                         //Increment the argument count
                     while((str[i+1]>=48)&& (str[i+1] <=57)) // move the loop till the end of the number
                     {
                         i=i+1;
                     }
               }
          }
          for(k=0;k<i;k++)
          {
             if(((str[k]>=48 && str[k]<=57) || (str[k]>=65 && str[k]<=90) || (str[k]>=97 && str[k]<=122)))
             {
                str[k]=str[k];
             }
             else
             {
                     str[k]=NULL;
              }

          }
          //Select command to perform desired operation
          select_command();

       }
     }
