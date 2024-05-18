
#ifndef STSPIN32G4_H_
#define STSPIN32G4_H_

void spg4_init(void);
void spg4_set_pwm(float dc_a, float dc_b, float dc_c);
float get_vbus(void);
void adc_isr(void);
void spg4_adc_init(void);
void ml_handler(void);

#endif /* STSPIN32G4_H_ */
