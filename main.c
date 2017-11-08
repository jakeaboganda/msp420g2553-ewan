#include "msp430g2553.h"
#include <stdlib.h>
#include <stdio.h>
#include "circular_buffer.h"

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

#define enableTx() {UC0IE |= UCA0TXIE;}
#define enableRx() {UC0IE |= UCA0RXIE;}
#define disableTx() {UC0IE &= ~UCA0TXIE;}
#define disableRx() {UC0IE &= ~UCA0RXIE;}

//static volatile uint8_t value = 0;
#define ADC_CHANNELS 1
static volatile uint16_t samples[ADC_CHANNELS];

// maximum values
#define ADC_RESOLUTION  10
#define ADC_FLOOR       0
#define ADC_MAX         (((0x1 << ADC_RESOLUTION) - 1) - ADC_FLOOR)

// convert to millivolts
#define ADC_MILLIVOLT_MAX ((uint16_t)3500)
#define ADC_VALUE_TO_MILLIVOLTS(adc_value) (uint16_t)(ADC_MILLIVOLT_MAX * (((float)(adc_value)) / ADC_MAX))
#define ADC_LOGIC_LOW_MINIMUM_DIFFERENCE    80  // millivolts
#define ADC_LOGIC_HIGH_MAXIMUM_DIFFERENCE   20  // millivolts

#define start_sampling() { \
    ADC10CTL0 &= ~ENC; \
    while (ADC10CTL1 & BUSY); \
    ADC10SA = (uint16_t)samples; \
    ADC10CTL0 |= ENC | ADC10SC; \
}

static circular_buffer_t uart_buffer = CIRCULAR_BUFFER_INIT;

void output_string(const uint8_t *bytes, uint16_t length)
{
    uint16_t i;
    for(i = 0; i < length; i++)
    {
        circularBuffer_pushByte(&uart_buffer, bytes[i]);
    }
}

void output(const uint8_t byte)
{
    circularBuffer_pushByte(&uart_buffer, byte);
    enableTx();
}

void output16(const uint16_t data16)
{
    output((data16 >> 0) & 0xFF);
    output((data16 >> 8) & 0xFF);
}

static void transmit_to_uart(void)
{
    while (!(IFG2 & UCA0TXIFG));
    if(!circularBuffer_isEmpty(&uart_buffer))
    {
        UCA0TXBUF = circularBuffer_popByte(&uart_buffer);
    }
    else
    {
        disableTx();
    }
}

static void configure_adc(void)
{
   /* Configure ADC Channel */
   while (ADC10CTL1 & BUSY);

   ADC10CTL1 = INCH_4 + ADC10DIV_3 + ADC10SSEL_0 + CONSEQ_3 + SHS_0; // Multi-channel repeated conversion starting from channel 4
   ADC10CTL0 = SREF_5 + ADC10SHT_2 + MSC + ADC10ON + ADC10IE;
   ADC10AE0 = BIT3 + BIT4; // http://www.ti.com/lit/ds/symlink/msp430g2553.pdf ; page 45
   CAPD &= ~(BIT3 + BIT4); // http://www.ti.com/lit/ds/symlink/msp430g2553.pdf ; page 45
   ADC10DTC1 = ADC_CHANNELS;
}

static void configure_hardware(void)
{
    /* Configure the hardware */
    WDTCTL = WDTPW + WDTHOLD;   // Stop WDT
    //DCOCTL = 0;                 // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;      // Set range // Set DC0
    DCOCTL = CALDCO_16MHZ;
    BCSCTL2 &= ~(DIVS_3);       // SMCLK = DCO = 1MHz // from ADC
    P2DIR |= 0xFF;              // All P2.x outputs
    P2OUT &= 0x00;              // All P2.x reset
    P3DIR |= 0xFF;              // All P3.x outputs
    P3OUT &= 0x00;              // All P3.x reset
    P1SEL |= RXD + TXD ;        // P1.1 = RXD, P1.2=TXD
    ///P1SEL |= BIT3 + BIT4;       // ADC Input pin P1.3 = VREF-, P1.4 = VREF+ //Refer to: http://www.ti.com/lit/ds/symlink/msp430g2553.pdf page45
    P1SEL2 |= RXD + TXD ;       // P1.1 = RXD, P1.2=TXD
    //P1SEL2 |= BIT3 + BIT4;      // ADC Input pin P1.3 = VREF-, P1.4 = VREF+ //Refer to: http://www.ti.com/lit/ds/symlink/msp430g2553.pdf page45
    P1DIR |= RXLED + TXLED;
    P1OUT &= 0x00;
    UCA0CTL1 |= UCSSEL_2;       // SMCLK
    UCA0BR0 = 0x8A;             // 16MHz 115200
    UCA0BR1 = 0x00;             // 16MHz 115200
    UCA0MCTL = 7 << 3;          // Modulation UCBRSx = 7
    UCA0CTL1 &= ~UCSWRST;       // Initialize USCI state machine

    enableRx();
}

// bits buffer
#define BITS_BUFFER_SIZE     64
static uint16_t bits_buffer[BITS_BUFFER_SIZE];
static volatile uint8_t bits_buffer_end_index = BITS_BUFFER_SIZE - 1;
static volatile uint16_t bits_buffer_end_mask = 0;
static volatile uint8_t bits_buffer_end_count = 0;
static volatile uint8_t bits_buffer_start_index = BITS_BUFFER_SIZE - 1;
static volatile uint8_t bits_buffer_start_shift = 32;
static volatile uint8_t bits_buffer_start_count = 0;
static volatile bool bits_buffer_not_full = true;

#define BUS_IDLE_HIGH_COUNT 40//100
#define BUS_START_BIT_MINIMUM_LOW_COUNT      8//80
#define BUS_LOGIC_BIT_MINIMUM_LOW_COUNT      2//5
#define BUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT     5//15
static volatile bool bus_is_busy = true;

//#define OUTPUT_BYTE

static void read_adc()
{
    start_sampling();
    __bis_SR_register(CPUOFF + GIE);

    static volatile bool is_high = false;
    static volatile bool not_frame = true;
    static uint16_t sample_count = 0;
    static uint8_t readerBitsBufferStartShift = 16;
    static bool process = false;

    // get the voltage difference in mV
#define volt_diff (ADC_VALUE_TO_MILLIVOLTS(samples[0]))

    //output('-');
    //output16(samples[0]);
    //output16(ADC_VALUE_TO_MILLIVOLTS(samples[BUS_PLUS]));
    //output16(ADC_VALUE_TO_MILLIVOLTS(samples[BUS_MINUS]));
    //output16(samples[0]);
#if 1
    if(volt_diff > ADC_LOGIC_LOW_MINIMUM_DIFFERENCE) // logic low
    {
#ifdef OUTPUT_BYTE
#else
        output(0);
#endif
        if(is_high)
        {
            //output('H');
            //output(sample_count);
            sample_count++;
            if(sample_count == BUS_IDLE_HIGH_COUNT)
            {
                //output('-');
                bus_is_busy = false;
                not_frame = true;
            }
        }
        else
        {
            is_high = true;
            while(bits_buffer_not_full)
            {
                if(bits_buffer_end_mask == 0)
                {
                    if(++bits_buffer_end_index == BITS_BUFFER_SIZE)
                    {
                        bits_buffer_end_index = 0;
                    }

                    if(bits_buffer_end_index == bits_buffer_start_index)
                    {
                        // overrun
                        bits_buffer_not_full = false;
                        not_frame = true;
                        break;
                    }

                    bits_buffer[bits_buffer_end_index] = 0;
                    bits_buffer_end_mask = 0x1;
                }

                if(sample_count > BUS_START_BIT_MINIMUM_LOW_COUNT)
                {
                    //received++
                    bits_buffer[bits_buffer_end_index] |= (bits_buffer_end_mask << 1);
                    not_frame = false;
                }
                else if(not_frame)
                {
                    break;
                }
                else if(sample_count < BUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT)
                {
                    if(sample_count < BUS_LOGIC_BIT_MINIMUM_LOW_COUNT)
                    {
                        break;
                    }
                    bits_buffer[bits_buffer_end_index] |= bits_buffer_end_mask;
                }

                bits_buffer_end_mask <<=2;
#if 1 //jake
                if(bits_buffer_end_mask == 0)
                {
#ifdef OUTPUT_BYTE
#else
#endif
                }
#endif
                ++bits_buffer_end_count;
                break;
            }

            sample_count = 1;
        }
    }
    else
    {
#ifdef OUTPUT_BYTE
#else
        output(1);
#endif
        bus_is_busy = true;

        if(is_high)
        {
            sample_count = 1;
            is_high = false;
        }
        else
        {
            ++sample_count;
        }
    }
#endif
}


static void transmit_bits_buffer()
{
    do
    {
#if 0
        if(bits_buffer_start_index != bits_buffer_end_index)
        {
            output('[');
            output(bits_buffer[bits_buffer_start_index]);
            if(++bits_buffer_start_index == BITS_BUFFER_SIZE)
            {
                bits_buffer_start_index = 0;
            }
        }

#endif
    } while(0);
}

int main(void)
{
    configure_hardware();
    configure_adc();

    __enable_interrupt();

   while (1)
   {
       //
       __delay_cycles(1000); // Wait for ADC Ref to settle
       read_adc();
       transmit_bits_buffer();
       transmit_to_uart();
   }
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
   __bic_SR_register_on_exit(CPUOFF); // Return to active mode
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    P1OUT |= TXLED;
    if(!circularBuffer_isEmpty(&uart_buffer))
    {
        UCA0TXBUF = circularBuffer_popByte(&uart_buffer);
    }
    else
    {
        disableTx();
    }
    P1OUT &= ~TXLED;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
#if 0
   P1OUT |= RXLED;
   enableTx();
   UCA0TXBUF = UCA0RXBUF;
   P1OUT &= ~RXLED;
#else
   // control LED
   //P1OUT |= RXLED;
   //enableTx();
   //UCA0TXBUF = sizeof(samples) / sizeof(samples[0]);
   UCA0TXBUF = sizeof(int);
   __bic_SR_register_on_exit(CPUOFF);
   //P1OUT &= ~RXLED;
#endif
}
