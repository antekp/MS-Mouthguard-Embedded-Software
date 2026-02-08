#include <zephyr/drivers/adc.h>

int8_t IsADCReady(const struct adc_dt_spec *adc_channel);
int8_t ADCChannelSetup(const struct adc_dt_spec *adc_channel);
int8_t ADCRawToMilivolts(const struct adc_dt_spec *adc_channel, int *val_mv);
void StartingADC(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence);
void PrintRawAndMilivoltsADCValue(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence);
int8_t ADCSequenceInit(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence);
int8_t ReadADCValue(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence);