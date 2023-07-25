#include <mbed.h>

AnalogIn heatsink_temp(PB_0);
AnalogOut pot1(PA_4);
AnalogOut pot2(PA_5);

DigitalOut enc_a(PB_7);
DigitalOut enc_b(PB_6);

DigitalIn motor1_fb(PA_0, PullUp);

DigitalOut led(LED1);
DigitalOut beep(PB_5);

DigitalInOut inst_loopback(PB_1, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut slave_clutch(PA_8, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut st_adap(PA_11, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut sj2_reln(PB_4, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut sj2_rel(PA_1, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut arm_present(PA_6, PIN_OUTPUT, OpenDrainNoPull, 1);

Ticker enc_ticker;
Ticker beep_ticker;
volatile uint32_t encoder_count = 0;

const uint16_t pot1_test_voltage = 0x4000 * 4.5 / 3.3; // 0x4000 in qladisp 
double pot_voltage_coefficient = 1.0;

volatile bool beep_enable = 0;
volatile bool digin_state = 0;

void increment_encoder() {
  encoder_count += 1;
  enc_a = (encoder_count / 2) % 2;
  enc_b = ((encoder_count - 1) / 2) % 2; 

  if (encoder_count % 10 == 0) {
    digin_state = !digin_state;
  }
  inst_loopback = digin_state;
  slave_clutch = digin_state;
  st_adap = digin_state;
  sj2_reln = digin_state;
  sj2_rel = digin_state;
  arm_present = digin_state;
}

void beep_func() {
    beep = beep_enable ? !beep : 0;
}

double adc_to_deg_c(double adc) {
  return adc * 3.3 / 0.01;
}

int main() {
  enc_ticker.attach(&increment_encoder, 0.1);
//   beep_ticker.attach(&beep_func, 1.0/8e3);


  while(1) {
    led = motor1_fb;
    pot_voltage_coefficient = motor1_fb ? 1.0 : 0.5;
    pot1.write_u16(pot1_test_voltage * pot_voltage_coefficient); 
    pot2.write_u16(pot1_test_voltage * 2 * pot_voltage_coefficient);
    // beep_enable = (adc_to_deg_c(heatsink_temp) > 50.0) ? 1 : 0;
  }
}