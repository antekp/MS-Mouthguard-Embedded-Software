#include <zephyr/drivers/adc.h>

int8_t IsADCReady(const struct adc_dt_spec *adc_channel);
int8_t ADCChannelSetup(const struct adc_dt_spec *adc_channel);