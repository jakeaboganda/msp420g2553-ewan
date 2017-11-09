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
#define ADC_CHANNELS 2
static volatile uint16_t samples[ADC_CHANNELS];

// maximum values
#define ADC_RESOLUTION  10
#define ADC_FLOOR       0
#define ADC_MAX         (((0x1 << ADC_RESOLUTION) - 1) - ADC_FLOOR)

// convert to millivolts
#define ADC_MILLIVOLT_MAX ((uint16_t)3500)
#define ADC_VALUE_TO_MILLIVOLTS(adc_value) (uint16_t)(ADC_MILLIVOLT_MAX / ADC_MAX * (((float)(adc_value))))
#define ADC_LOGIC_LOW_MINIMUM_DIFFERENCE    120  // millivolts
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

   ADC10CTL1 = INCH_4/*4*/      // BIT 3 and BIT4
           + ADC10DIV_0         // ADC Clock divider = 1
           + ADC10SSEL_0/*2*/   // Select the ADC10OSC as the source clock
           + CONSEQ_3           // Repeat sequence of channels
           + SHS_0;             // Sample and hold source is ADC10SC
   ADC10CTL0 = SREF_0/*5*/      // VR+ = VCC and VR- = VSS
           + ADC10SHT_0/*2*/    // Use four ADC10CLKs
           + MSC                // First rising edge triggers sampling timer, succeeding conversions are done automatically
           + ADC10ON            // Enable ADC10
           + ADC10IE;           // Interrupt enable
   ADC10AE0 = BIT3 + BIT4; // http://www.ti.com/lit/ds/symlink/msp430g2553.pdf ; page 45
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
    //P1SEL |= BIT3 + BIT4;       // ADC Input pin P1.3 = VREF-, P1.4 = VREF+ //Refer to: http://www.ti.com/lit/ds/symlink/msp430g2553.pdf page45
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
#define BITS_BUFFER_SIZE     32
static uint16_t bits_buffer[BITS_BUFFER_SIZE];
static volatile uint8_t bits_buffer_end_index = BITS_BUFFER_SIZE - 1;
static volatile uint16_t bits_buffer_end_mask = 0;
static volatile uint8_t bits_buffer_end_count = 0;
static volatile uint8_t bits_buffer_start_index = BITS_BUFFER_SIZE - 1;
static volatile uint8_t bits_buffer_start_shift = 16;
static volatile uint8_t bits_buffer_start_count = 0;
static volatile bool bits_buffer_not_full = true;

#define BUS_IDLE_HIGH_COUNT 100//100
#define BUS_START_BIT_MINIMUM_LOW_COUNT      100//80
#define BUS_LOGIC_BIT_MINIMUM_LOW_COUNT      5//5
#define BUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT     15//15
static volatile bool bus_is_busy = true;



// maximum length of data field in bytes
#define IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE      256
#define IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH               32 // Mode 1


enum {
    IEBUS_BIT_LOGIC_LOW,
    IEBUS_BIT_LOGIC_HIGH,
    IEBUS_BIT_START
};

// frame buffer
typedef struct {
    uint8_t valid;
    uint8_t notBroadcast;
    uint8_t masterAddress[2];
    uint8_t slaveAddress[2];
    uint8_t control;
    uint8_t dataLength;
    uint8_t data[IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH];
} READER_FRAME_BUFFER;

enum {
    READER_FRAME_BUFFER_INDEX_BROADCAST_BIT,
    READER_FRAME_BUFFER_INDEX_MASTER_ADDRESS_MSB,
    READER_FRAME_BUFFER_INDEX_MASTER_ADDRESS_LSB,
    READER_FRAME_BUFFER_INDEX_SLAVE_ADDRESS_MSB,
    READER_FRAME_BUFFER_INDEX_SLAVE_ADDRESS_LSB,
    READER_FRAME_BUFFER_INDEX_CONTROL,
    READER_FRAME_BUFFER_INDEX_MESSAGE_LENGTH,
    READER_FRAME_BUFFER_INDEX_DATA
};

static READER_FRAME_BUFFER readerFrameBuffer;
//static const READER_FRAME_BUFFER *lastReaderFrameBuffer = &readerFrameBuffer[READER_FRAME_BUFFER_ITEMS - 1];
//static READER_FRAME_BUFFER *readerFrameBufferEnd = readerFrameBuffer;
//static READER_FRAME_BUFFER *readerFrameBufferStart = readerFrameBuffer;

// IEBus address
typedef int16_t IEBUS_ADDRESS;
#define INVALID_IEBUS_ADDRESS   (IEBUS_ADDRESS)-1

// IEBus control
enum {
    IEBUS_CONTROL_SLAVE_STATUS_READ             = 0x0,  // reads slave status (SSR)
    IEBUS_CONTROL_DATA_READ_AND_LOCK            = 0x3,  // reads and locks data
    IEBUS_CONTROL_LOCK_LSB_ADDRESS_READ         = 0x4,  // reads lock address (lower 8 bits)
    IEBUS_CONTROL_LOCK_MSB_ADDRESS_READ         = 0x5,  // reads lock address (higher 4 bits)
    IEBUS_CONTROL_SLAVE_STATUS_READ_AND_UNLOCK  = 0x6,  // reads and unlocks slave status (SSR)
    IEBUS_CONTROL_DATA_READ                     = 0x7,  // reads data
    IEBUS_CONTROL_COMMAND_WRITE_AND_LOCK        = 0xa,  // writes and locks command
    IEBUS_CONTROL_DATA_WRITE_AND_LOCK           = 0xb,  // writes and locks data
    IEBUS_CONTROL_COMMAND_WRITE                 = 0xe,  // writes command
    IEBUS_CONTROL_DATA_WRITE                    = 0xf   // writes data
}; typedef uint8_t IEBUS_CONTROL;

// IEBus frame

// frame fields
enum {
    IEBUS_FRAME_FIELD_BROADCAST_BIT,
    IEBUS_FRAME_FIELD_MASTER_ADDRESS,
    IEBUS_FRAME_FIELD_SLAVE_ADDRESS,
    IEBUS_FRAME_FIELD_CONTROL,
    IEBUS_FRAME_FIELD_MESSAGE_LENGTH,
    IEBUS_FRAME_FIELD_DATA
};

// maximum number of bits in a frame
#define IEBUS_FRAME_MAXIMUM_NUMBER_OF_BITS          (45 + (10 * IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH))

// number of bits (excluding parity and acknowledge) for each field
#define IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT        1
#define IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS       12
#define IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS        12
#define IEBUS_FRAME_FIELD_BITS_CONTROL              4
#define IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH       8
#define IEBUS_FRAME_FIELD_BITS_DATA                 8

// frame field information
typedef struct {
    uint8_t id;
    bool hasParityBit;
    bool hasAcknowledgeBit;
    uint8_t dataBits;
    uint8_t totalBits;
    bool isTwoBytes;
    uint16_t bitMask;
} IEBUS_FRAME_FIELD;

#define IEBUS_FRAME_FIELD_INITIALIZER(id, hasParityBit, hasAcknowledgeBit, numberOfBits) \
    [id] = { \
            id, \
            hasParityBit, \
            hasAcknowledgeBit, \
            numberOfBits, \
            numberOfBits + hasParityBit + hasAcknowledgeBit, \
            (numberOfBits > 8), \
            0x1 << (numberOfBits - 1) \
    }

static const IEBUS_FRAME_FIELD iebus_frame_field[] = {
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_BROADCAST_BIT, false, false, IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MASTER_ADDRESS, true, false, IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_SLAVE_ADDRESS, true, true, IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_CONTROL, true, true, IEBUS_FRAME_FIELD_BITS_CONTROL),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MESSAGE_LENGTH, true, true, IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_DATA, true, true, IEBUS_FRAME_FIELD_BITS_DATA)
};

void analyze_bits(bool loop)
{
#define finishFrame(reason) \
        { \
            processing_frame = false; \
        }

    static bool processing_frame = false; // indicates that a frame is being processed
    static bool check_acknowledgement; // ordinary (not broadcast) communication should be acknowledged
    static uint8_t remaining_field_bits; // remaining bits for the current field
    static const IEBUS_FRAME_FIELD *current_field; // current field information
    static uint16_t field_bit_mask; // data bit mask for the current field
    static uint16_t field_data_buffer; // data buffer for the current field
    static uint16_t frame_high_bit_count; // frame high bit count (excluding the broadcast and acknowledge bits) for parity check
    static uint16_t remaining_bytes; // number of remaining bytes to receive for the current frame
    static uint8_t *data;

    do
    {
        if(bits_buffer_not_full)
        {
            if(bits_buffer_end_count != bits_buffer_start_count)
            {
                if(bits_buffer_start_shift < 14)
                {
                    bits_buffer_start_shift += 2;
                }
                else
                {
                    bits_buffer_start_shift = 0;

                    if(++bits_buffer_start_index == BITS_BUFFER_SIZE)
                    {
                        bits_buffer_start_index = 0;
                    }
                }

                uint8_t bit = (bits_buffer[bits_buffer_start_index] >> bits_buffer_start_shift) & 0x3;

                ++bits_buffer_start_count;

                if(bit == IEBUS_BIT_START)
                {
                    // the previous frame didn't finish
                    if(processing_frame)
                    {
                        //output('x');
                        //output('x');
                        finishFrame(incomplete);
                    }

                    // prepare buffer
                    data = &readerFrameBuffer.notBroadcast;

                    // after the start bit is the broadcast bit
                    current_field = iebus_frame_field;
                    remaining_field_bits = current_field->totalBits;
                    field_bit_mask = current_field->bitMask;
                    field_data_buffer = 0;

                    // processing_frame the frame
                    processing_frame = true;
                }
                else if(processing_frame)
                {
                    output(bit);
                    // save the bit
                    if(bit == IEBUS_BIT_LOGIC_HIGH)
                    {
                        ++frame_high_bit_count; // track the number of high bits for parity checking later
                        field_data_buffer |= field_bit_mask;
                    }

                    field_bit_mask >>= 1;

                    // last bit of field
                    if(--remaining_field_bits == 0)
                    {
                        if(current_field->hasAcknowledgeBit)
                        {
                            // ignore frames that are not acknowledged by slave during ordinary communication
                            if(bit == IEBUS_BIT_LOGIC_HIGH)
                            {
                                --frame_high_bit_count;

                                if(check_acknowledgement)
                                {
                                    finishFrame(notAcknowledged);
                                    continue;
                                }
                            }
                        }

                        if(current_field->hasParityBit)
                        {
                            // data is invalid if high bit count is odd
                            if((frame_high_bit_count & 0x1))
                            {
                                finishFrame(parityError);

                                continue;
                            }
                        }

                        // save the field (up to two bytes)
                        if(current_field->isTwoBytes)
                        {
                            *data = field_data_buffer >> 8; // MSB
                            ++data;
                        }

                        *data = field_data_buffer; // LSB
                        ++data;

                        if(current_field->id == IEBUS_FRAME_FIELD_DATA)
                        {
                            // last byte of frame
                            if(--remaining_bytes == 0)
                            {
                                readerFrameBuffer.valid = true;

                                finishFrame(valid);

                                output('*');
                                continue;
                            }
                        }
                        else
                        {
                            if(current_field->id == IEBUS_FRAME_FIELD_BROADCAST_BIT)
                            {
                                check_acknowledgement = (bit == IEBUS_BIT_LOGIC_HIGH);
                                frame_high_bit_count = 0;
                            }
                            else if(current_field->id == IEBUS_FRAME_FIELD_MESSAGE_LENGTH)
                            {
                                // 0 means maximum
                                if(field_data_buffer == 0)
                                {
                                    field_data_buffer = IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE;
                                }

                                // ignore multi-frame data for now; should be okay since all relevant data are single frame
                                if(field_data_buffer > IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH)
                                {
                                    finishFrame(tooLong);
                                    continue;
                                }

                                remaining_bytes = field_data_buffer;
                            }

                            ++current_field;
                        }

                        // prepare to processing_frame the next field
                        remaining_field_bits = current_field->totalBits;
                        field_bit_mask = current_field->bitMask;
                        field_data_buffer = 0;
                    }
                }
            }
        }
        else
        {
            bits_buffer_end_index = BITS_BUFFER_SIZE - 1;
            bits_buffer_end_mask = 0;
            bits_buffer_end_count = 0;
            bits_buffer_start_index = BITS_BUFFER_SIZE - 1;
            bits_buffer_start_shift = 16;
            bits_buffer_start_count = 0;
            bits_buffer_not_full = true;
        }
    }
    while(loop);
}
#if 0
static void analyze_bits_buffer()
{
#if 1
    analyze_bits(false);
#else
    if(bits_buffer_end_mask == 0)
    {
        while(bits_buffer_start_index != bits_buffer_end_index)
        {
#ifdef OUTPUT_BYTE
            output('[');
            output(bits_buffer[bits_buffer_start_index]);
#else
            //output('[');
            output16(bits_buffer[bits_buffer_start_index]);
#endif
            if(++bits_buffer_start_index == BITS_BUFFER_SIZE)
            {
                bits_buffer_start_index = 0;
            }
        }
    }
#endif
}
#endif
static void read_adc()
{
    start_sampling();
    __bis_SR_register(CPUOFF + GIE);

    static volatile bool is_high = false;
    static volatile bool not_frame = true;
    static uint16_t sample_count = 0;
    //static bool process = false;

    //output('-');
#define BUS_PLUS (ADC_VALUE_TO_MILLIVOLTS(samples[0]))
#define BUS_MINUS (ADC_VALUE_TO_MILLIVOLTS(samples[1]))
#define volt_diff ((BUS_PLUS > BUS_MINUS) ? BUS_PLUS - BUS_MINUS : BUS_MINUS - BUS_PLUS)
    //output16(ADC_VALUE_TO_MILLIVOLTS(samples[0]));
    //output16(volt_diff);
#if 1
    if(volt_diff > ADC_LOGIC_LOW_MINIMUM_DIFFERENCE)
    {
        //output(0); //logic "0"
        if(is_high)
        {
            sample_count++;
            if(sample_count == BUS_IDLE_HIGH_COUNT)
            {
                bus_is_busy = false;
                not_frame = true;
                output('-');
#if 1//jake
                bits_buffer_start_index = bits_buffer_end_index = 0;
#endif
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
                        output('!');
                        output('!');
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
                    //output('@');
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
                ++bits_buffer_end_count;
#if 0
                if(bits_buffer_end_mask == 0)
                {
                    //analyze_bits_buffer();
                }
                //output(bits_buffer_end_mask);
#endif
                break;
            }

            sample_count = 1;
        }
    }
    else
    {
#ifdef OUTPUT_BYTE
#else
        //output(1); //logic "1"
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

int main(void)
{
    configure_hardware();
    configure_adc();

    __enable_interrupt();

   while (1)
   {
       //__delay_cycles(1000); // Wait for ADC Ref to settle
       read_adc();
       analyze_bits(false);
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
