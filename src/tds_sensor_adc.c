#include <zephyr/sys/printk.h>
#include "tds_sensor_adc.h"

int8_t IsADCReady(const struct adc_dt_spec *adc_channel)
{
    if(!adc_is_ready_dt(&adc_channel))
    {
        printk("ADC controller devivce  not ready");
        return -1;
    }
    return 0;
}
int8_t ADCChannelSetup(const struct adc_dt_spec *adc_channel)
{
    
    uint8_t err = adc_channel_setup_dt(&adc_channel);
    if(err < 0)
    {
        printk("ADC controller devivce  not ready");
        return err;
    }
    return 0;
}
int8_t ADCSequenceInit(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence)
{
    uint8_t err = adc_sequence_init_dt(&adc_channel, &sequence);
    if(err < 0)
    {
        printk("Could not initalize sequnce");
        return err;
    }
    return 0;
}

int8_t ReadADCValue(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence)
{
    uint8_t err = adc_read(adc_channel->dev, &sequence);
    if (err < 0) {
        printk("Could not read value");
	}
    return 0;
}