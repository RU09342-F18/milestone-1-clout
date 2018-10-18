/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F552x Demo - USCI_A0, SPI 3-Wire Master Incremented Data
//
//   Description: SPI master talks to SPI slave using 3-wire mode. Incrementing
//   data is sent by the master starting at 0x01. Received data is expected to
//   be same as the previous transmission.  USCI RX ISR is used to handle
//   communication with the CPU, normally in LPM0. If high, P1.0 indicates
//   valid data reception.  Because all execution after LPM0 is in ISRs,
//   initialization waits for DCO to stabilize against ACLK.
//   ACLK = ~32.768kHz, MCLK = SMCLK = DCO ~ 1048kHz.  BRCLK = SMCLK/2
//
//   Use with SPI Slave Data Echo code example.  If slave is in debug mode, P1.1
//   slave reset signal conflicts with slave's JTAG; to work around, use IAR's
//   "Release JTAG on Go" on slave device.  If breakpoints are set in
//   slave RX ISR, master must stopped also to avoid overrunning slave
//   RXBUF.
//
//                   MSP430F552x
//                 -----------------
//             /|\|                 |
//              | |                 |
//              --|RST          P1.0|-> LED
//                |                 |
//                |             P3.3|-> Data Out (UCA0SIMO)
//                |                 |
//                |             P3.4|<- Data In (UCA0SOMI)
//                |                 |
//  Slave reset <-|P1.1         P2.7|-> Serial Clock Out (UCA0CLK)
//
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************

#include <msp430.h>

void LEDSetup();
void timerSetup();
void resetColor();
void UARTSetup();

volatile int byte = 0;
volatile int buffer = 0;

void LEDSetup(void)
{
    // Red LED
    P1DIR |= BIT2;                  // Sets the Red LED to the output direction
    P1SEL |= BIT2;                  // Allows BIT2 to be the output of TA0.1

    // Green LED
    P1DIR |= BIT3;                  // Sets the Green LED to the output direction
    P1SEL |= BIT3;                  // Allows BIT3 to be the output of TA0.2

    // Blue LED
    P1DIR |= BIT4;                  // Sets the Blue LED to the output direction
    P1SEL |= BIT4;                  // Allows BIT4 to be the output of TA0.3
}

void timerSetup(void)
{
    TA0CTL = TASSEL_2 + MC_1;           // SMCLK set to UP mode
    TA0CCR0 = 255;                      // PWM Period

    // Red LED
    TA0CCR1 = 0;                        // PWM of Red LED
    TA0CCTL1 = OUTMOD_2;                // Toggle/Reset

    // Green LED
    TA0CCR2 = 0;                        // PWM of Green LED
    TA0CCTL2 = OUTMOD_2;                // Toggle/Reset

    // Blue LED
    TA0CCR3 = 0;                        //PWM of Blue LED
    TA0CCTL3 = OUTMOD_2;                // Toggle/Reset
}

void UARTSetup(void)
{
    P4SEL |= (BIT4+BIT5);                   // Allows BIT4 to become the TXD output and BIT5 to become the RXD input
    UCA1CTL1 |= UCSWRST;                    // State Machine Reset + Small Clock Initialization
    UCA1CTL1 |= UCSSEL_2;                   // Sets USCI Clock Source to SMCLK
    UCA1BR0 = 6;                            // 9600 Baud Rate
    UCA1BR1 = 0;                            // 9600 Baud Rate
    UCA1MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;
    UCA1CTL1 &= ~UCSWRST;                   // Initialize USCI State Machine
    UCA1IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt

}


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;           // Stop Watchdog Timer
    LEDSetup();
    timerSetup();
    UARTSetup();

    __bis_SR_register(LPM0_bits + GIE);   // Low Power Mode with Global Interrupt Enabled
    __no_operation();
}

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
      case 0:break;                             // No Interrupt

      case 2: // Receiver Interrupt

          switch(byte)
              {

              case 0:

                  while(!(UCA1IFG & UCTXIFG));
                  buffer = UCA1RXBUF;
                  UCA1TXBUF = UCA1RXBUF - 3;    // Removes three bytes from the transmitted signals and transfers the rest to the next device
                  __no_operation();             // Stops the clock
                  break;

              case 1:

                  TA0CCR1 = (UCA1RXBUF);         // Sets PWM of Red LED
                  break;

              case 2:

                  TA0CCR2 = (UCA1RXBUF);      // Sets PWM of Green LED
                  break;

              case 3:

                  TA0CCR3 = (UCA1RXBUF);      // Sets PWM of Blue LED
                  break;

              default:
                  while(!(UCA1IFG & UCTXIFG)); // While there is no transmit interrupt and no USCI_A! interrupt
                  UCA1TXBUF = UCA1RXBUF;      // Sends the remaining bytes to the next board
                  break;
              }

              if(byte != buffer - 1)
              {
                  byte += 1;
              }

              else if (byte == buffer - 1)
              {
                            // Reset Count
                   byte = 0;
              }

        break;

        case 4:break;

        default: break;

      }
}

