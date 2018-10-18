 #include <msp430.h>

volatile int byte = 0;

extern void UARTSetup();
extern void LEDSetup();
extern void TimerASetup();
extern void resetColor();

void UARTSetup(void)
{

    P4SEL |= (BIT4+BIT5);
    UCA1CTL1 |= UCSWRST; //State machine reset + small clock initialization     UCA1CTL1 |= UCSSEL_2;
    UCA1BR0 = 6;                //9600 baud *Determined by TI calculator(http://processors.wiki.ti.com/index.php/USCI_UART_Baud_Rate_Gen_Mode_Selection)
    UCA1BR1 = 0;            //9600 baud *Determined by TI calculator(http://processors.wiki.ti.com/index.php/USCI_UART_Baud_Rate_Gen_Mode_Selection)
    UCA1MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;
    UCA1CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
    UCA1IE |= UCRXIE; // Enable USCI_A0 RX interrupt
}

void LEDSetup(void)
{
    P1DIR |= (BIT2 + BIT3 + BIT4);  //Set outputs for LEDs
    P1SEL |= (BIT2 + BIT3 + BIT4);
}

void TimerASetup(void)
{
     TA0CTL = TASSEL_2 + MC_1;   // ACLK with 8 divider set to up mode

     TA0CCR0 = 0xFF; // PWM Period
     TA0CCR1 = 255; //red led
     TA0CCR2 = 255;   //green led
     TA0CCR3 = 255;   //blue led

     TA0CCTL1 = OUTMOD_2;               // CCR1 reset/set
     TA0CCTL2 = OUTMOD_2;               // CCR1 reset/set
     TA0CCTL3 = OUTMOD_2;
}

void resetColor()
{
    TA0CCR1 = 0x00;
    TA0CCR2 = 0x00;
    TA0CCR3 = 0x00;
}

volatile int buffer = 0;

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{

    switch(__even_in_range(UCA1IV,4))
    {
      case 0:break;                             // Vector 0 - no interrupt
      case 2: // Vector 2 - RXIFG
          switch(byte)    {
                  case 0:
                      while(!(UCA1IFG & UCTXIFG));
                      buffer = UCA1RXBUF;
                      UCA1TXBUF = UCA1RXBUF - 3;  //Takes 3 bytes off the code for LEDs and leaves the rest
                      __no_operation();           //Pauses the clock for a moment
                      break;
                  case 1:
                      TA0CCR1 = (UCA1RXBUF);      //Sets RED section
                      break;
                  case 2:
                      TA0CCR2 = (UCA1RXBUF);      //Sets Green section
                      break;
                  case 3:
                      TA0CCR3 = (UCA1RXBUF);      //Sets Blue section
                      break;
                  default:
                      while(!(UCA1IFG & UCTXIFG));
                      //UCA1TXBUF = UCA1RXBUF - 3;  //Takes 3 bytes off the code for LEDs and leaves the rest
                      UCA1TXBUF = UCA1RXBUF;      //Transmit bytes to next board
                      break;
    }
        if(byte != buffer - 1)
        {
        byte += 1;
        }
        else if (byte == buffer - 1)
        {
        //At the end of the message reset count in anticipation for the next message
        byte = 0;
        }

    break;

    case 4:break;
    default: break;
    }
    // Increment/Reset byte counter

}

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT
  LEDSetup();
  TimerASetup();
  UARTSetup();

                               // do not load, trap CPU!!
  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  __no_operation();
}


