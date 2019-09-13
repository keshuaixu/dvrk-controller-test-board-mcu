#include <mbed.h>
#include <stdint.h>
#include <ctype.h>
DigitalInOut OW(PA_7);
AnalogIn heatsink_temp(PB_0);
AnalogOut pot2(PA_5);
AnalogOut pot1(PA_4);
DigitalOut enc_a(PB_7);
DigitalOut enc_b(PB_6);
DigitalIn motor1_fb(PA_0, PullUp);
DigitalOut led(LED1);

DigitalInOut inst_loopback(PB_1, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut slave_clutch(PA_8, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut st_adap(PA_11, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut sj2_reln(PB_4, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut sj2_rel(PA_1, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut arm_present(PA_6, PIN_OUTPUT, OpenDrainNoPull, 1);

InterruptIn OneWire(PA_7);
Serial pc(USBTX, USBRX);
Ticker enc_ticker;
//   {Fami, <---, ---, ---, ID--, ---,  -->,7CRC}
uint8_t ROM_ID[8] = {0x0B, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x0B, 0x10};
#define mem_size 64 * 32
uint8_t memory[mem_size] = {0xFF};
uint8_t mem_start[] = "Dedicated to taking surgical precision and technique beyond the limits of the human hand. Copyright Intuitive Surgical, Inc. 1997.";
uint8_t mem_0x9e[] = {0x04, 0xfb};
uint8_t mem_0xa4[] = {0x00, 0x06, 0x1a, 0x86};
uint8_t mem_0x160[] = "LARGE NEEDLE DRIVER.";
uint8_t OW_buffer; // buffer for data received on 1-wire line

volatile uint32_t encoder_count = 0;
const uint16_t pot1_test_voltage = 0x4000 * 4.5 / 3.3; //0x5000 in qla disp
double pot_voltage_coefficient = 1.0;

void increment_encoder()
{
    encoder_count += 1;
    enc_a = (encoder_count / 2) % 2;
    enc_b = ((encoder_count - 1) / 2) % 2;

    inst_loopback = enc_a;
    slave_clutch = enc_a;
    st_adap = enc_a;
    sj2_reln = enc_a;
    sj2_rel = enc_a;
    arm_present = enc_a;
}

double adc_to_deg_c(double adc)
{
    return adc * 3.3 / 0.01;
}

uint8_t crc8(unsigned char *pDst, unsigned char length)
{
    unsigned char i, j, aa, check, flag;
    check = 0;
    for (i = 0; i < length; i++)
    {
        aa = *pDst++;
        for (j = 0; j < 8; j++)
        {
            flag = (check ^ aa) & 1;
            check >>= 1;
            if (flag)
            {
                check ^= 0x0c;
                check |= 0x80;
            }
            aa >>= 1;
        }
    }
    return check;
}

// 1-wire slave write bit          Master------>Slaver
void OW_write_byte(uint8_t value) //121us
{
    for (uint8_t i = 0; i < 8; i++)
    {
        while (OW)
            ; // Wait for bit start
        //wait_us(2);
        if ((value & 0x01) == 0) // if bit is zero
        {
            OW.output();
            OW.mode(OpenDrain);
            OW = 0;
            wait_us(10); // 30
            //OW.output(); OW.mode(OpenDrain); OW=1;
            OW.input();
        }
        else // write 1
        {
            OW.output();
            OW.mode(OpenDrain);
            OW = 1;
            wait_us(10);
            OW.output();
            OW.mode(OpenDrain);
            OW = 1;
            OW.input();
        }
        value >>= 1;
    }
}

uint8_t OW_read_byte() // Slave--->Master  90us(0--80+10..; 1--8+90+2)
{
    uint8_t res = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        // __disable_irq();
        while (OW)
            ;        // Wait for bit start
        wait_us(14); // 9 15 Delay to sample bit value
        res >>= 1;
        if (OW)
        {
            res |= 0x80;
        }
        wait_us(70);
        // __enable_irq();
    }
    return (res);
}
//==============================================================================
void memcpy(uint8_t *destination, uint8_t *source, const uint16_t bytes)
{
    for (uint16_t counter = 0; counter < bytes; counter++)
    {
        destination[counter] = source[counter];
    }
}
void readMemory(uint8_t *destination, uint16_t length, uint16_t position)
{
    memcpy(destination, &memory[position], length);
}

//write string & array to Stm32 memory offset position
void writeMemory(uint8_t *source, uint16_t length, uint16_t position)
{
    if (position >= mem_size)
        return;
    memcpy(&memory[position], source, (position + length >= mem_size) ? (mem_size - position) : length);
}
void fillMemory()
{
    writeMemory(mem_start, sizeof(mem_start), 0x00);
    writeMemory(mem_0x9e, sizeof(mem_0x9e), 0x9e);
    writeMemory(mem_0xa4, sizeof(mem_0xa4), 0xa4);
    writeMemory(mem_0x160, sizeof(mem_0x160), 0x160);
}
void clearMemory()
{
    for (uint16_t counter = 0; counter < mem_size; counter++)
    {
        memory[counter] = 0xFF;
    }
}
// Print current memory content to console
void printMemory()
{
    for (int i = 0; i < 64; i++)
    {
        pc.printf("\r\n--PAGE %02d--------------", i, "\r\n");
        for (int j = 0; j < 32; j++)
        {
            if (isprint(memory[i * 32 + j]))
                pc.printf("%4C", memory[i * 32 + j]);
            else
                pc.printf("%4X", memory[i * 32 + j]);
        }
    }
    pc.printf("\r\n");
}

//**************************************************************
// detect reset pulse (returns 1 if pulse detected)
uint8_t OW_reset_pulse(void)
{
    uint8_t del_count = 50;
    while (del_count--)
    { // this loop takes 13.6 us
        wait_us(10);
        if (OW)
        {
            break;
        }
    }
    //  printf("Reset time = %f us\r\n.",(50-del_count)*13.6);
    //  Here OW=1
    if (del_count > 40)
    {
        return 0; // too short
    }
    return 1; // reset pulse detected
}

void OW_presence_pulse()
{
    OW.output();
    OW.mode(OpenDrain);
    OW = 0;
    wait_us(300); // tPD+tPDL=
    OW.output();
    OW.mode(OpenDrain);
    OW = 1;       //generate presence pulse
    wait_us(500); // 480 is testified before
}
void Duty() // F0----Memory Command
{
    uint16_t TA1, TA2, tgaddr;  // Target address
    OW_buffer = OW_read_byte(); // Read Memory command F0
    TA1 = OW_read_byte();       // Memory address (low byte)
    TA2 = OW_read_byte();       // Memory address (high byte)
    //OW_write_byte(memory[TA2<<8 | TA1]);
    for (int i = 0; i < 64 * 32; i++)
        OW_write_byte(memory[i]);
}
void ISR1()
{
    if (OW_reset_pulse()) //  while(OW_reset_pulse())  if reset detected
    {
        __disable_irq();
        wait_us(30);                // 15~60us wait for end of timeslot
        OW_presence_pulse();        // generate presence pulse
        OW_buffer = OW_read_byte(); //Slave receive Rom command from Master
        //printf("read out ROM command is 0X%2X.\r\n",OW_buffer);
        for (uint8_t i = 0; i < 8; i++)
            OW_write_byte(ROM_ID[i]);
        Duty();
        __enable_irq();
        return;
    }
}

int main()
{
    pc.printf("\n\r****** dVRK controller Testing (including 1-wire instrument******\r\n");
    clearMemory();
    fillMemory();
    printMemory(); // 64 pages*32 Bytes, 2K Byte memory size
    enc_ticker.attach(&increment_encoder, 0.2);
    OneWire.fall(&ISR1); // InterruptIn    OneWire(PA_7);
    OW.mode(OpenDrain);
    // ROM_ID[7] =crc8(ROM_ID,7);  pc.printf("Family code ROM_ID is 0X%X .\r\n",ROM_ID[7]);
    while (1)
    {
        led = motor1_fb;
        pot_voltage_coefficient = motor1_fb ? 1.0 : 0.5;
        pot1.write_u16(pot1_test_voltage * pot_voltage_coefficient);
        pot2.write_u16(pot1_test_voltage * 2 * pot_voltage_coefficient);
    }
}
