#include "msp430g2553.h"
#include <stdlib.h>
#include <stdio.h>
#include "circular_buffer.h"

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

#ifdef ENABLE_UART
#define enableTx() {UC0IE |= UCA0TXIE;}
#define enableRx() {UC0IE |= UCA0RXIE;}
#define disableTx() {UC0IE &= ~UCA0TXIE;}
#define disableRx() {UC0IE &= ~UCA0RXIE;}
#else
#define enableTx()
#define enableRx()
#define disableTx()
#define disableRx()
#endif

//static volatile uint8_t value = 0;
#define ADC_CHANNELS 2
static int samples[ADC_CHANNELS];

// maximum values
#define ADC_RESOLUTION  10
#define ADC_FLOOR       0
#define ADC_MAX         (((0x1 << ADC_RESOLUTION) - 1) - ADC_FLOOR)

// convert to millivolts
#define ADC_MILLIVOLT_MAX ((uint16_t)3500)
#define ADC_VALUE_TO_MILLIVOLTS(adc_value)  (uint16_t)(((float)(adc_value) / ADC_MAX) * ADC_MILLIVOLT_MAX)
#define ADC_LOGIC_LOW_MINIMUM_DIFFERENCE    300  // millivolts
#define ADC_LOGIC_HIGH_MAXIMUM_DIFFERENCE   20  // millivolts

#define ADC_LOGIC_LOW_MINIMUM_DIFFERENCE_ADC    200//88// ADC COUNT 88 = 300 * 1023 / 3500
#if 0
#define start_sampling() { \
    ADC10CTL0 &= ~ENC; \
    while (ADC10CTL1 & BUSY); \
    ADC10SA = (int)samples; \
    ADC10CTL0 |= ENC | ADC10SC; \
}
#else
#define start_sampling() { \
    ADC10CTL0 &= ~ENC; \
    ADC10SA = (int)samples; \
    ADC10CTL0 |= ENC | ADC10SC; \
}
#endif

static circular_buffer_t uart_buffer = CIRCULAR_BUFFER_INIT;
uint8_t *output_string_p;
#define output_string(string, length) { \
    output_string_p = (uint8_t *)string;\
    while(length--) circularBuffer_pushByte(&uart_buffer, *(output_string_p++)); \
}

#define output_raw(byte) {circularBuffer_pushByte(&uart_buffer, (uint8_t)(byte));}
#define shift_byte(bits, byte_to_shift) ((uint8_t)((byte_to_shift >> bits) & 0xFF))
#define output(d8) {output_raw(d8);}
#if 0
#define output16(d16) { \
    output_raw(((uint8_t*)(&d16))[0]); \
    output_raw(((uint8_t*)(&d16))[1]); \
}
#else
#define output16(d16) { \
        output_raw(((d16) >> 0)&0xff); \
        output_raw(((d16) >> 8)&0xff); \
}
#endif
#define output32(d32) { \
        output_raw(((d32) >> 0)&0xff); \
        output_raw(((d32) >> 8)&0xff); \
        output_raw(((d32) >> 16)&0xff); \
        output_raw(((d32) >> 24)&0xff); \
}

static void configure_adc(void)
{
   while (ADC10CTL1 & BUSY);        // Wait until ADC can be properly modified

   ADC10CTL1 = INCH_5               // BIT 4 and BIT 5
           + ADC10DIV_1             // ADC Clock divider = 5
           + ADC10SSEL_3            // Select SMCLK as the source clock
           + CONSEQ_1               // Sequence of channels
           + SHS_0;                 // Sample and hold source is ADC10SC

   ADC10CTL0 = SREF_0               // VR+ = VCC and VR- = VSS
           + ADC10SHT_0             // Use 4 ADC10CLKs
           + MSC                    // First rising edge triggers sampling timer, succeeding conversions are done automatically
           + ADC10ON                // Enable ADC10
           + ADC10IE;               // Interrupt enable
   ADC10AE0 = BIT4 + BIT5;          // Enable analog mode for Pin1.4 and P1.5
   ADC10DTC1 = ADC_CHANNELS;        // NUmber of ADC Channels
}

static void configure_hardware(void)
{
    /* Configure the hardware */
    WDTCTL = WDTPW + WDTHOLD;       // Stop Watchdog Timer
    DCOCTL = 0;                     // Clear DCOx and MODx
    BCSCTL2 = 0;                    // MCLK = DCOCLK; MCLK DIV = 1
    BCSCTL1 = CALBC1_16MHZ;         // Set range // Set DC0
    DCOCTL = CALDCO_16MHZ;          // Set DCO
    //BCSCTL1 |= XT2OFF + DIVA_0;     // XT2OFF -- Disable XT2CLK ; DIVA_0 -- divide by 1
    BCSCTL2 = 0;
    //BCSCTL3 = XT2S_2 |              // 3 ~ 16MHz
    //        LFXT1S_2 |              // If XTS = 0, XT1 = VLOCLK ; If XTS = 1, XT1 = 3 - 16-MHz crystal or resonator
    //        XCAP_1;                 // ~6 pF

    // Power conservation
    P2DIR |= 0xFF;                  // All P2.x outputs
    P2OUT &= 0x00;                  // All P2.x reset
    P3DIR |= 0xFF;                  // All P3.x outputs
    P3OUT &= 0x00;                  // All P3.x reset

#ifdef ENABLE_UART
    // UART Pin Directions
    P1SEL |= RXD + TXD ;            // P1.1 = RXD, P1.2=TXD
    P1SEL |= BIT3;                  // P1.3 = ADC10CLK
    //P1SEL |= BIT3 + BIT4;         // ADC Input pin P1.3 = VREF-, P1.4 = VREF+ //Refer to: http://www.ti.com/lit/ds/symlink/msp430g2553.pdf page45
    P1SEL2 |= RXD + TXD ;           // P1.1 = RXD, P1.2=TXD
    //P1SEL2 |= BIT3 + BIT4;        // ADC Input pin P1.3 = VREF-, P1.4 = VREF+ //Refer to: http://www.ti.com/lit/ds/symlink/msp430g2553.pdf page45
    P1SEL2 &= ~BIT3;                // P1.3 = ADC10CLK
    P1DIR |= RXLED + TXLED;
    P1DIR |= BIT3;                  // P1.3 = ADC10CLK
    P1OUT &= 0x00;
#else
    P1SEL |= BIT3;                  // P1.3 = ADC10CLK
    P1SEL2 |= RXD + TXD ;           // P1.1 = RXD, P1.2=TXD
    P1SEL2 &= ~BIT3;                // P1.3 = ADC10CLK
    P1DIR |= RXLED + TXLED;
    P1DIR |= BIT3;                  // P1.3 = ADC10CLK
    P1OUT &= 0x00;
#endif

#ifdef ENABLE_UART
    // UART Initialization
    UCA0CTL1 |= UCSSEL_2;           // SMCLK
    UCA0BR0 = 0x8A;                 // 16MHz 115200 baud
    UCA0BR1 = 0x00;                 // 16MHz 115200 baud
    UCA0MCTL = 7 << 3;              // Modulation UCBRSx = 7
    UCA0CTL1 &= ~UCSWRST;           // Initialize USCI state machine
#endif
}

static uint32_t bits_buffer;
static volatile uint32_t bits_buffer_end_mask = 0;

#define bits_buffer_length ((bits_buffer_start_index < bits_buffer_end_index) ? (bits_buffer_end_index - bits_buffer_start_index) : (BITS_BUFFER_SIZE - bits_buffer_start_index + bits_buffer_end_index))

#define BUS_IDLE_HIGH_COUNT 30//100
#define BUS_START_BIT_MINIMUM_LOW_COUNT      30//80
#define BUS_LOGIC_BIT_MINIMUM_LOW_COUNT      2//5
#define BUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT     4//15
static volatile bool bus_is_busy = true;

//#define OUTPUT_BITS_RAW
int main(void)
{
    configure_hardware();
    configure_adc();
    enableRx();

    __enable_interrupt();

   while (1)
   {
       start_sampling();
       __bis_SR_register(CPUOFF + GIE);
       //__bis_SR_register(GIE);
       //analyze_bits();
       enableTx();
   }
}

// We are going to use the TX LED to signify a logic "1" or logic "0"
#define logic1() {P1OUT |= TXLED;}
#define logic0() {P1OUT &= ~TXLED;}

#define volt_diff ((samples[0] > samples[1]) ? (samples[0] - samples[1]) : (samples[1] - samples[0]))
//#define volt_diff (samples[1] - samples[0])
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
#if 1
    if(volt_diff > ADC_LOGIC_LOW_MINIMUM_DIFFERENCE_ADC)
    {
        logic0();
    } else
    {
        logic1();
    }
#else
    P1OUT |= TXLED;
    static volatile bool is_high = false;
    static volatile bool not_frame = true;
    static uint8_t sample_count = 0;
    do
    {
        if(volt_diff > ADC_LOGIC_LOW_MINIMUM_DIFFERENCE_ADC)
        {
            if(is_high)
            {
                sample_count++;
                if(sample_count == BUS_IDLE_HIGH_COUNT)
                {
                    bus_is_busy = false;
                    not_frame = true;
                }
            }
            else
            {
                is_high = true;
                do
                {
                    if(bits_buffer_end_mask == 0)
                    {
                        output32(bits_buffer);
                        bits_buffer_end_mask = 0x1;
                        bits_buffer = 0;
                    }

                    if(sample_count > BUS_START_BIT_MINIMUM_LOW_COUNT)
                    {
#ifdef OUTPUT_BITS_RAW
                        output(2);
#endif
                        bits_buffer |= (bits_buffer_end_mask << 1);
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
#ifdef OUTPUT_BITS_RAW
                        output(1);
#endif
                        bits_buffer |= bits_buffer_end_mask;
                    }
                    else
                    {
#ifdef OUTPUT_BITS_RAW
                        output(0);
#endif
                    }

                    bits_buffer_end_mask <<=2;
                } while(0);

                sample_count = 1;
            }
        }
        else
        {
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
    } while(0);
    P1OUT &= ~TXLED;
#endif
   __bic_SR_register_on_exit(CPUOFF); // Return to active mode
}

#ifdef ENABLE_UART
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    if(!circularBuffer_isEmpty(&uart_buffer))
    {
        UCA0TXBUF = circularBuffer_popByte(&uart_buffer);
    }
    else
    {
        disableTx();
    }
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
   UCA0TXBUF = 'j';//sizeof(int);
   __bic_SR_register_on_exit(CPUOFF);
   //P1OUT &= ~RXLED;
#endif
}
#endif
