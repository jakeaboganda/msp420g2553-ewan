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

#if 0 // enable for writer
#define WRITER_CLOCK_RESOLUTION     1000000
#define WRITER_CLOCK_FREQUENCY      TM4C1294_CLOCK_FREQUENCY_SYSTEM
#define WRITER_USEC_TO_COUNT(usec)    (uint32_t)(((float)WRITER_CLOCK_FREQUENCY / WRITER_CLOCK_RESOLUTION) * usec + 0.5)
#endif

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
   ADC10CTL1 = INCH_4 + ADC10DIV_3 + CONSEQ_3 + SHS_0; // Multi-channel repeated conversion starting from channel 4
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
    BCSCTL1 = CALBC1_1MHZ;      // Set range // Set DC0
    DCOCTL = CALDCO_1MHZ;
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
    UCA0BR0 = 0x08;             // 1MHz 115200
    UCA0BR1 = 0x00;             // 1MHz 115200
    UCA0MCTL = UCBRS2 + UCBRS0; // Modulation UCBRSx = 5
    UCA0CTL1 &= ~UCSWRST;       // Initialize USCI state machine

    enableRx();
}

// bits buffer
#define BITS_BUFFER_SIZE     64
static uint8_t bits_buffer[BITS_BUFFER_SIZE];
static volatile uint8_t bits_buffer_end_index = BITS_BUFFER_SIZE - 1;
static volatile uint8_t bits_buffer_end_mask = 0;
static volatile uint8_t bits_buffer_end_count = 0;
static volatile uint8_t bits_buffer_start_index = BITS_BUFFER_SIZE - 1;
static volatile uint8_t bits_buffer_start_shift = 32;
static volatile uint8_t bits_buffer_start_count = 0;
static volatile bool bits_buffer_not_full = true;

#define BUS_IDLE_HIGH_COUNT 100
#define BUS_START_BIT_MINIMUM_LOW_COUNT 32//80
#define BUS_LOGIC_BIT_MINIMUM_LOW_COUNT      2//5
#define BUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT     6//15
static volatile bool bus_is_busy = true;

static void read_adc()
{
    start_sampling();
    __bis_SR_register(CPUOFF + GIE);
    //output('-');
    //output16(samples[0]);
    //output16(samples[1]);
    //output16(ADC_VALUE_TO_MILLIVOLTS(samples[0]));
    //output16(ADC_VALUE_TO_MILLIVOLTS(samples[1]));

    static volatile bool is_high = false;
    static volatile bool not_frame = true;
    static uint16_t sample_count = 0;

    // get the voltage difference in mV
#if 0
    uint16_t volt_diff = ADC_VALUE_TO_MILLIVOLTS(samples[BUS_PLUS]) -
            ADC_VALUE_TO_MILLIVOLTS(samples[BUS_MINUS]);
#else
#define volt_diff (ADC_VALUE_TO_MILLIVOLTS(samples[0]))
#endif

    //output('-');
    //output16(samples[0]);
    //output16(ADC_VALUE_TO_MILLIVOLTS(samples[BUS_PLUS]));
    //output16(ADC_VALUE_TO_MILLIVOLTS(samples[BUS_MINUS]));
    //output16(samples[0]);
#if 1
    if(volt_diff > ADC_LOGIC_LOW_MINIMUM_DIFFERENCE)
    {
        //output(0);
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
                        output('X');
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
                ++bits_buffer_end_count;
                break;
            }
        }

        sample_count = 1;
    }
    else
    {
        //output(1);
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
#define IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH 32
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

#define READER_FRAME_BUFFER_ITEMS   5
static READER_FRAME_BUFFER readerFrameBuffer[READER_FRAME_BUFFER_ITEMS];
static const READER_FRAME_BUFFER *lastReaderFrameBuffer = &readerFrameBuffer[READER_FRAME_BUFFER_ITEMS - 1];
static READER_FRAME_BUFFER *readerFrameBufferEnd = readerFrameBuffer;
static READER_FRAME_BUFFER *readerFrameBufferStart = readerFrameBuffer;

enum {
    READER_IEBUS_BIT_LOGIC_LOW,
    READER_IEBUS_BIT_LOGIC_HIGH,
    READER_IEBUS_BIT_START
};

// frame fields
enum {
    IEBUS_FRAME_FIELD_BROADCAST_BIT,
    IEBUS_FRAME_FIELD_MASTER_ADDRESS,
    IEBUS_FRAME_FIELD_SLAVE_ADDRESS,
    IEBUS_FRAME_FIELD_CONTROL,
    IEBUS_FRAME_FIELD_MESSAGE_LENGTH,
    IEBUS_FRAME_FIELD_DATA
};
typedef struct {
    uint8_t id;
    bool hasParityBit;
    bool hasAcknowledgeBit;
    uint8_t dataBits;
    uint8_t totalBits;
    bool isTwoBytes;
    uint16_t bitMask;
} IEBUS_FRAME_FIELD;

// maximum number of bits in a frame
#define IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE      256
#define IEBUS_FRAME_MAXIMUM_NUMBER_OF_BITS          (45 + (10 * IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH))

// number of bits (excluding parity and acknowledge) for each field
#define IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT        1
#define IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS       12
#define IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS        12
#define IEBUS_FRAME_FIELD_BITS_CONTROL              4
#define IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH       8
#define IEBUS_FRAME_FIELD_BITS_DATA                 8

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

static const IEBUS_FRAME_FIELD iebusFrameField[] = {
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_BROADCAST_BIT, false, false, IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MASTER_ADDRESS, true, false, IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_SLAVE_ADDRESS, true, true, IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_CONTROL, true, true, IEBUS_FRAME_FIELD_BITS_CONTROL),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MESSAGE_LENGTH, true, true, IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_DATA, true, true, IEBUS_FRAME_FIELD_BITS_DATA)
};


static void transmit_bits_buffer()
{
    static bool process = false;
#define finishFrame(x) {process = false;}
    static uint8_t *data;
    static bool checkAcknowledgement; // ordinary (not broadcast) communication should be acknowledged
    static uint8_t remainingFieldBits; // remaining bits for the current field
    static const IEBUS_FRAME_FIELD *currentField; // current field information
    static uint16_t fieldBitMask; // data bit mask for the current field
    static uint16_t fieldDataBuffer; // data buffer for the current field
    static uint16_t frameHighBitCount; // frame high bit count (excluding the broadcast and acknowledge bits) for parity check
    static int remainingBytes; // number of remaining bytes to receive for the current frame

    do
    {

        if(bits_buffer_start_index != bits_buffer_end_index)
        {
#if 0
            output('-');
            output(bits_buffer[bits_buffer_start_index]);
            if(++bits_buffer_start_index == BITS_BUFFER_SIZE)
            {
                bits_buffer_start_index = 0;
            }
#else
            //output('N');
            //output(bits_buffer_end_count);
            //output(bits_buffer_start_count);
            //output('O');
            if(bits_buffer_end_count != bits_buffer_start_count)
            {
                if(bits_buffer_start_shift < 30)
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

                //output(bits_buffer[bits_buffer_start_index]);

                int bit = (bits_buffer[bits_buffer_start_index] >> bits_buffer_start_shift) & 0x3;

                ++bits_buffer_start_count;

                if(bit == READER_IEBUS_BIT_START)
                {
                    // the previous frame didn't finish
                    if(process)
                    {
                        finishFrame(incomplete);
                    }

                    // prepare buffer
                    data = &readerFrameBufferEnd->notBroadcast;

                    // after the start bit is the broadcast bit
                    currentField = iebusFrameField;
                    remainingFieldBits = currentField->totalBits;
                    fieldBitMask = currentField->bitMask;
                    fieldDataBuffer = 0;

                    // process the frame
                    process = true;
                }
                else if(process)
                {
                    // save the bit
                    if(bit == READER_IEBUS_BIT_LOGIC_HIGH)
                    {
                        ++frameHighBitCount; // track the number of high bits for parity checking later
                        fieldDataBuffer |= fieldBitMask;
                    }

                    fieldBitMask >>= 1;

                    // last bit of field
                    if(--remainingFieldBits == 0)
                    {
                        if(currentField->hasAcknowledgeBit)
                        {
                            // ignore frames that are not acknowledged by slave during ordinary communication
                            if(bit == READER_IEBUS_BIT_LOGIC_HIGH)
                            {
                                --frameHighBitCount;

                                if(checkAcknowledgement)
                                {
                                    finishFrame(notAcknowledged);
                                    continue;
                                }
                            }
                        }

                        if(currentField->hasParityBit)
                        {
                            // data is invalid if high bit count is odd
                            if((frameHighBitCount & 0x1))
                            {
                                finishFrame(parityError);

                                continue;
                            }
                        }

                        // save the field (up to two bytes)
                        if(currentField->isTwoBytes)
                        {
                            *data = fieldDataBuffer >> 8; // MSB
                            ++data;
                        }

                        *data = fieldDataBuffer; // LSB
                        ++data;

                        if(currentField->id == IEBUS_FRAME_FIELD_DATA)
                        {
                            // last byte of frame
                            if(--remainingBytes == 0)
                            {
                                //vmIebusIsActive = true;
                                //lastBusActivityTime = clock_get();

                                //mutex_lock(&readerFrameBufferMutex);

                                readerFrameBufferEnd->valid = true;
                                readerFrameBufferEnd = (readerFrameBufferEnd == lastReaderFrameBuffer) ? readerFrameBuffer : (readerFrameBufferEnd + 1);

                                if(readerFrameBufferEnd == readerFrameBufferStart)
                                {
                                    //++vmIebusStatistics.readerFrameBuffer.overrun;
                                    readerFrameBufferStart->valid = false;
                                    readerFrameBufferStart = (readerFrameBufferStart == lastReaderFrameBuffer) ? readerFrameBuffer : (readerFrameBufferStart + 1);
                                }
                                else
                                {
#if 0
                                    //++vmIebusStatistics.readerFrameBuffer.currentUsage;

                                    if(vmIebusStatistics.readerFrameBuffer.currentUsage > vmIebusStatistics.readerFrameBuffer.maximumUsage)
                                    {
                                        //vmIebusStatistics.readerFrameBuffer.maximumUsage = vmIebusStatistics.readerFrameBuffer.currentUsage;
                                    }
#endif
                                }

                               // mutex_unlock(&readerFrameBufferMutex);

                                finishFrame(valid);
                                continue;
                            }
                        }
                        else
                        {
                            if(currentField->id == IEBUS_FRAME_FIELD_BROADCAST_BIT)
                            {
                                checkAcknowledgement = (bit == READER_IEBUS_BIT_LOGIC_HIGH);
                                frameHighBitCount = 0;
                            }
                            else if(currentField->id == IEBUS_FRAME_FIELD_MESSAGE_LENGTH)
                            {
                                // 0 means maximum
                                if(fieldDataBuffer == 0)
                                {
                                    fieldDataBuffer = IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE;
                                }

                                // ignore multi-frame data for now; should be okay since all relevant data are single frame
                                if(fieldDataBuffer > IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH)
                                {
                                    finishFrame(tooLong);
                                    continue;
                                }

                                remainingBytes = fieldDataBuffer;
                            }

                            ++currentField;
                        }

                        // prepare to process the next field
                        remainingFieldBits = currentField->totalBits;
                        fieldBitMask = currentField->bitMask;
                        fieldDataBuffer = 0;
                    }
                }
            }
#endif
        }

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
