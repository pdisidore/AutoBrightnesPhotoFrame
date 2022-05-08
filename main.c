#include <msp430.h>
#include "GrLib/grlib/grlib.h"          // TI’s Graphics Library (grlib)
#include "LcdDriver/lcd_driver.h"       // Driver codes for LCD
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define LED_red BIT0          //Position of Red LED 1
#define LED_green BIT7
#define S1 BIT1
#define S2 BIT2
#define LCD_BackLight BIT6 //S6
#define S6 BIT6
//#define Up 0
//#define Max 1
//#define Down 2
//#define Min 3
volatile unsigned int k = 0;
volatile unsigned int input = 2;    //input 1: S1; input 0: S2; input 2: others;
enum GateStates {On = 0, Off = 1} state;
/*
 * GPIO Initialization
 */
void config_LCD(){
    PJSEL0 = BIT4 | BIT5;       //LFXT
    // Initialize LCD segments
    LCDCPCTL0 = 0xFFFF;
    LCDCPCTL1 = 0xFC3F;
    LCDCPCTL2 = 0x0FFF;
    // Configure LFXT 32kHz crystal
    CSCTL0_H = CSKEY >> 8;
    CSCTL4 &= ~LFXTOFF;         // Enable LFXT
    do {
        CSCTL5 &= ~LFXTOFFG; // Clear LFXT fault flag
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG); // Test oscillator fault flag
    CSCTL0_H = 0; // Lock CS registers
    // Initialize LCD_C
    // ACLK, Divider = 1, Pre-divider = 16; 4-pin MUX
    LCDCCTL0 = LCDDIV__1 | LCDPRE__16 | LCD4MUX | LCDLP;
    // VLCD generated internally,
    // V2-V4 generated internally, v5 to ground
    // Set VLCD voltage to 2.60v
    // Enable charge pump and select internal reference for it
    LCDCVCTL = VLCD_1 | VLCDREF_0 | LCDCPEN;
    LCDCCPCTL = LCDCPCLKSYNC; // Clock synchronization enabled
    LCDCMEMCTL = LCDCLRM; // Clear LCD memory
    //Turn LCD on
    LCDCCTL0 |= LCDON;
    return;
}


//We setup I2C in this function
void I2C_Setup(void){
    UCB1CTLW0 |= UCSWRST;//Dataset says only configure while paused

    //Configure P4.0 and P4.1 to I2C mode
    P4SEL1 |= BIT0;
    P4SEL1 |= BIT1;
    P4SEL0 &= ~BIT0;
    P4SEL0 &= ~BIT1;

    //UCMODE_3: I2C
    //UCMST:    Master Select
    //UCSSEL_3: USCI 0 Clock Source: 3, SMCLK
    UCB1CTLW0 |= UCMODE_3 | UCMST | UCSSEL_3;

    UCB1BRW = 8;    //Baud rate generator is 8; I believe the SMCLK clk frequency is 131KHz

    UCB1CTLW0 &= ~UCSWRST;//Unpause/begin operation
}


// Master reads 2 bytes from device via I2C
// i2c_address: Device addr in I2C
// i2c_reg:     reg addr in the device
// data:        data read from the device reg
int i2c_read_word(unsigned char i2c_address, unsigned char i2c_reg, unsigned int * data){
    unsigned char byte1, byte2;
    // Master writes frame 1
    UCB1I2CSA = i2c_address;    // Setup the device address in I2c
    UCB1IFG &= ~UCTXIFG0;       // Clear TX flag so we don't get confused below
    UCB1CTLW0 |= UCTR;          // Master set to write mode W (writes)
    UCB1CTLW0 |= UCTXSTT;       // Initiate the start signal

    while ((UCB1IFG & UCTXIFG0) ==0) {}
    UCB1TXBUF = i2c_reg;        // set the reg address

    while((UCB1CTLW0 & UCTXSTT)!=0) {}  //wait for transmit complete
    if(( UCB1IFG & UCNACKIFG )!=0) return -1; // error if NACK

    UCB1CTLW0 &= ~UCTR;         // Master set the R/W signal to R (reads)
    UCB1CTLW0 |= UCTXSTT;       // Initiate a Start Signal


    // Begin to read 1st Byte
    while ( (UCB1IFG & UCRXIFG0) == 0) {}
    byte1 = UCB1RXBUF;

    // Read the 2nd Byte
    while((UCB1CTLW0 & UCTXSTT)!=0) {}
    UCB1CTLW0 |= UCTXSTP;       // It really appears that you request another byte by setting the STOP bit

    while ( (UCB1IFG & UCRXIFG0) == 0) {}
    byte2 = UCB1RXBUF;

    while ( (UCB1CTLW0 & UCTXSTP) != 0) {}
    *data = ( (byte1 << 8) | (byte2 & 0xFF) );  // Received 2 bytes and put them in the data
    return 0;
}

// Master writes 2 bytes from a device via I2C
// i2c_address: Device addr in I2C
// i2c_reg:     reg addr in the device
// data:        data writing to the device reg
int i2c_write_word(unsigned char i2c_address, unsigned char i2c_reg, unsigned int data){
    unsigned char byte1, byte2;

    //Put data into 2 bytes
    byte1 = (data >> 8) & 0xFF;
    byte2 = data & 0xFF;

    UCB1I2CSA = i2c_address;        // Setup the device address in I2c
    UCB1CTLW0 |= UCTR;              // Master to W (writes)
    UCB1CTLW0 |= UCTXSTT;           // Initiate a Start Signal

    while ((UCB1IFG & UCTXIFG0) ==0) {}
    UCB1TXBUF = i2c_reg; // send the reg address
    while((UCB1CTLW0 & UCTXSTT)!=0) {}
    // Master writes the 1st Byte
    UCB1TXBUF = byte1;
    while ( (UCB1IFG & UCTXIFG0) == 0) {}
    // Master writes the 2nd Byte
    UCB1TXBUF = byte2;
    while ( (UCB1IFG & UCTXIFG0) == 0) {}
    UCB1CTLW0 |= UCTXSTP;
    while ( (UCB1CTLW0 & UCTXSTP) != 0) {} //Waiting to avoid race condition

    return 0;
}

/// Configuration of Joystick ///
void config_ADC_JOYSTICK(){
    // Enable Pins
    // Horizontal: P9.2 --> A10 to Digital
    P9SEL1 |= BIT2;
    P9SEL0 |= BIT2;
    // Vertical: P8.7 --> A4 to Digital
    P8SEL1 |= BIT7;
    P8SEL0 |= BIT7;

    ADC12CTL0 |= ADC12ON; //ADC enable
    ADC12CTL0 &= ~ADC12ENC; //Turn off ENC (Enable Conversion) for configuration

    //Setup ADC12CTL0
    ADC12CTL0|=ADC12ON|ADC12SHT1_10; //ADC12SHT0:control the interval of the sampling timer (the number of cycles)

    //Setup ADC12CTL1
    // ADC12SHS: configure ADC12SC bit as the trigger signal
    // ADC12SHP ADC12 Sample/Hold Pulse Mode
    // ADC12DIV ADC12 Clock Divider Select: 7
    // ADC12SSEL ADC12 Clock Source Select: 0
    ADC12CTL1= ADC12SHS_0|ADC12SHP|ADC12DIV_7|ADC12SSEL_0;
    ADC12CTL1|=ADC12CONSEQ_1;

    //Setup ADC12MCTL0
    // ADC12VRSEL: VR+=AVCC, VR-=AVSS
    // ADC12INCH: Channel A10
    ADC12MCTL0|= ADC12INCH_10|ADC12VRSEL_0;
    // ADC12INCH: Channel A4
    ADC12MCTL1|=ADC12INCH_4|ADC12VRSEL_0|ADC12EOS;

    ADC12CTL0 |= ADC12ENC; //Turn on ENC at the end
    return;
}
/*
 * We configure ACLK to 32 KHz
 * The default value of ACLK is 39KHz
 * */
void config_ACLK_to_32KHz(){
    //Perform the LFXIN and LFXOUT functionality
    PJSEL1 &= ~BIT4;
    PJSEL0 |= BIT4;
    CSCTL0 = CSKEY;             // Unlock CS registers
    do{
        CSCTL5 &= ~LFXTOFFG;    // Local fault flag
        SFRIFG1 &= ~OFIFG;      // Global fault flag
    } while((CSCTL5 & LFXTOFFG) != 0);
    CSCTL0_H = 0;               // Lock CS registers
    return;
}

/**
 * main.c
 */
int main(void)
{
    volatile unsigned int i;
    signed int imageIndex = -1;          //starts at -1 so that when we increase with our first joystick move, the index will be 0
    char lcd_str[30];

    extern tImage doge4BPP_UNCOMP;
    extern tImage BeStrong4BPP_UNCOMP;
    extern tImage GrowlingDoge4BPP_UNCOMP;
    extern tImage FatFaceDoge4BPP_UNCOMP;
    extern tImage BusinessDoge4BPP_UNCOMP;
    extern tImage ItWillGet4BPP_UNCOMP;
    extern tImage LifeIsHard4BPP_UNCOMP;

    tImage images[7] = {doge4BPP_UNCOMP, BusinessDoge4BPP_UNCOMP, GrowlingDoge4BPP_UNCOMP, FatFaceDoge4BPP_UNCOMP, BeStrong4BPP_UNCOMP,
                        ItWillGet4BPP_UNCOMP, LifeIsHard4BPP_UNCOMP};

        WDTCTL = WDTPW | WDTHOLD;   //Stop Watchdog Timer
        PM5CTL0 &= ~LOCKLPM5;       //Disable GPIO power-on default high impedance mode

        config_ACLK_to_32KHz();
        _enable_interrupts();
//        I2C_Setup();
//        i2c_write_word(0x44, 0x01, 0b1000011000000100);
        config_ADC_JOYSTICK();                                  //config and enable statements

        P1DIR |= LED_red;            //Set the direction of P1 as output
        P1OUT &= ~LED_red;           //Turn off the red LED
        P9DIR |= LED_green;            //Set the direction of P9 as output
        P9OUT &= ~LED_green;           //Turn off the green LED

        // LCD Config: specify SMCLK to 8 MHz in SPI clock
        CSCTL0 = CSKEY;                 // Unlock CS registers
        CSCTL3 &= ~(BIT4|BIT5|BIT6);    // DIVS=0
        CSCTL0_H = 0;                   // Relock the CS registers


        // Graphic Library Config
        Graphics_Context g_sContext;        // defines a drawing context to be use to draw onto the screen
        Crystalfontz128x128_Init();         // Initializes the display driver
        Crystalfontz128x128_SetOrientation(0);  // Set the LCD orientation
        Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128); // Initialize a drawing context
        Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK); // Set the background color to black
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE); // Set the foreground color to white
        GrContextFontSet(&g_sContext, &g_sFontFixed6x8); //Sets the font to be used
        Graphics_clearDisplay(&g_sContext); // force a clear screen


        // Test Code Start:
        Graphics_drawStringCentered(&g_sContext, "Hello User!", AUTO_STRING_LENGTH, 64, 15, OPAQUE_TEXT);
        sprintf(lcd_str, "This is my adjustable");
        Graphics_drawStringCentered(&g_sContext, (int8_t)lcd_str, AUTO_STRING_LENGTH, 64, 30, OPAQUE_TEXT);
        sprintf(lcd_str, "brightness display.");
        Graphics_drawStringCentered(&g_sContext, (int8_t)lcd_str, AUTO_STRING_LENGTH, 64, 40, OPAQUE_TEXT);

        sprintf(lcd_str, "Left Button(P1.1)");
        Graphics_drawStringCentered(&g_sContext,(int8_t) lcd_str, AUTO_STRING_LENGTH, 64, 60, OPAQUE_TEXT);
        sprintf(lcd_str, "turns on auto-bright." );
        Graphics_drawStringCentered(&g_sContext, (int8_t)lcd_str, AUTO_STRING_LENGTH, 64, 70, OPAQUE_TEXT);

        sprintf(lcd_str, "Right Button(P1.2)");
        Graphics_drawStringCentered(&g_sContext,(int8_t) lcd_str, AUTO_STRING_LENGTH, 64, 85, OPAQUE_TEXT);
        sprintf(lcd_str, "turns off auto-bright." );
        Graphics_drawStringCentered(&g_sContext, (int8_t)lcd_str, AUTO_STRING_LENGTH, 64, 95, OPAQUE_TEXT);

        //Graphics_drawImage(&g_sContext, &(images[0]) , 0, 0);

        P2DIR |= LCD_BackLight; //P2.6(LED RED/LCD BACKLIGHT) output

        P2SEL1 &= ~ S6;
        P2SEL0 |= S6;            //setting up the PWM pg103 of MSP430FR698x(1), MSP430FR598x(1) datasheet

        P1DIR &= ~S1;
        P1IFG &=~ S1;
        P1IE |= S1;
        P1REN |= S1;
        P1IES |= S1;
        P1OUT |= S1;

        P1DIR &= ~S2;
        P1IFG &=~ S2;
        P1IE |= S2;
        P1REN |= S2;
        P1IES |= S2;
        P1OUT |= S2;


        TB0CCTL0 = CCIE;  //CCR0 interrupt enable
        TB0CCTL5 = OUTMOD_7; //CCR5 OUTPUT MODE RESET/SET
        TB0CCR5 = 10;        //Initialize to low light

        //TBSSEL 1 = ACLK
        //ID 0 = /1, 1 = /2 , 2 = /4, 3 = /8
        //MC  0 = Stop, 1 = UP, 2 = Continuous , 3 = up/down
        //TBCLR clears
        //TBIE interrupt enable
        TB0CTL = TBSSEL_1 | ID_0 | MC_1 | TBCLR /*| TBIE*/ ;     // Error with TBIE
        TB0CCR0 = (100);    //counter to compare TB0CCR5 to


        unsigned int data = 0;
        unsigned int deviceID = 0;
        I2C_Setup();
        i2c_write_word(0x44, 0x01, 0b1000011000000100);
        i2c_read_word(0x44, 0x7E, &data);
        i2c_read_word(0x44, 0x7F, &deviceID);

        char buffData[5];
        char buffDeviceID[5];

        sprintf(buffData, "%d", data);
        sprintf(buffDeviceID, "%d", deviceID);   //testing I2C

        unsigned int light = 0;
        state = On;
        P1OUT |= LED_red;        // shows on state at start-up
        P2OUT |= LCD_BackLight;  // screen is on

        for(;;){
            int i = 0;

//            i2c_read_word(0x44,0x00,&data);
//            sprintf(light, "%d", data);          // convert data to an int number and store in light

            ADC12CTL0 |= ADC12SC;
            while((ADC12CTL1 & ADC12BUSY)==0);
            if(ADC12MEM0>2300){    //Joystick Right or low light    (Light needs debugging)
                if(imageIndex<6){
                    imageIndex++;
                    Graphics_drawImage(&g_sContext, &(images[imageIndex]) , 0, 0);
                    while(ADC12MEM0>2300){
                        ADC12CTL0 |= ADC12SC;
                        while((ADC12CTL1 & ADC12BUSY)==0);
                    }
                }
            }
            if(ADC12MEM0<1750){   //Joystick Left or high light
                if(imageIndex>0){
                    imageIndex--;
                    Graphics_drawImage(&g_sContext, &(images[imageIndex]) , 0, 0);
                    while(ADC12MEM0<1750){
                        ADC12CTL0 |= ADC12SC;
                        while((ADC12CTL1 & ADC12BUSY)==0);
                    }
                }
            }
            switch(state){
                case On:
                    for(i = 0;i<32000; i++){}
                    i2c_read_word(0x44, 0x00, &light);
                    if(light<40){
                        TB0CCR5 = 1;
                    }
                    else if(40<light && light<100){
                        TB0CCR5 = 20;
                    }
                    else if((100<light && light<600)){
                        TB0CCR5 = 50;
                    }
                    else if((light>600)){
                        TB0CCR5 = 100;
                    }
                    break;
                case Off:
                    for(i = 0;i<5000; i++){}
                    if(ADC12MEM1>2100){              //Joystick Up
                        if(TB0CCR5 +1 <100){
                            TB0CCR5 += 1;            //Brightness up
                        }
                    }
                    if(ADC12MEM1<1650){              //Joystick Down
                        if(TB0CCR5 -1 >1){
                            TB0CCR5 -= 1;            //Brightness down
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    return 0;
}

#pragma vector = TIMER0_B0_VECTOR        //Timer TB0CTL is Timer0_B7 Control
__interrupt void ISR_Brightness_B0(){
    TB0CTL &= ~TBIFG;
}

#pragma vector = PORT1_VECTOR           //Link the following ISR to the interrupt Vector in the vector table
__interrupt void ISR_Lab5_A1(){
    switch(state){
        case On:
            if((P1IN & S1)==0){
                state = On;                 // stay in current state
                P1OUT |= LED_red;           //Turn on the red Led to display auto mode is on
            }
            else if((P1IN & S2)==0){
                state = Off;
                P1OUT &= ~LED_red;           //Turn off the red Led to display auto mode is off
            }
            P1IFG &=~ S1;
            P1IFG &=~ S2;
            break;
        case Off:
            if((P1IN & S1)==0){
                state = On;                 //go to on current state
                P1OUT |= LED_red;           //Turn on the red Led to display auto mode is on
            }
            else if((P1IN & S2)==0){
                state = Off;
                P1OUT &= ~LED_red;           //Turn off the red Led to display auto mode is off
            }
            P1IFG &=~ S1;
            P1IFG &=~ S2;
            break;
        default:
            state = On;
            break;
    }

}
