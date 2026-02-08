#include <zephyr/sys/printk.h>
#include "tds_sensor_adc.h"




void StartingADC(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence)
{
    if(IsADCReady(&adc_channel) != 0)
    {
        return;
    }
    if(ADCChannelSetup(&adc_channel) != 0)
    {
        return;
    }
    if(ADCSequenceInit(&adc_channel, &sequence) != 0)
    {
        return;
    }
}

void PrintRawAndMilivoltsADCValue(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence, int *val_mv)
{
    int val_mv = (int)(*sequence).buffer; 
    ReadADCValue(&adc_channel, &sequence);
    printk("ADC Reading: channel[%d] raw: %d\r\n", adc_channel->channel_id, val_mv);

    if(ADCRawToMilivolts(&adc_channel, &val_mv) != 0)
    {
        return;
    }
}

int8_t IsADCReady(const struct adc_dt_spec *adc_channel)
{
    if(!adc_is_ready_dt(&adc_channel))
    {
        printk("ADC controller devivce  not ready\r\n");
        return -1;
    }
    return 0;
}
int8_t ADCChannelSetup(const struct adc_dt_spec *adc_channel)
{
    
    uint8_t err = adc_channel_setup_dt(&adc_channel);
    if(err < 0)
    {
        printk("ADC controller devivce  not ready\r\n");
        return err;
    }
    return 0;
}
int8_t ADCSequenceInit(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence)
{
    uint8_t err = adc_sequence_init_dt(&adc_channel, &sequence);
    if(err < 0)
    {
        printk("Could not initalize sequnce\r\n");
        return err;
    }
    return 0;
}

int8_t ReadADCValue(const struct adc_dt_spec *adc_channel, struct adc_sequence *sequence)
{
    uint8_t err = adc_read(adc_channel->dev, &sequence);
    if (err < 0) {
        printk("Could not read value\r\n");
	}
    return 0;
}
int8_t ADCRawToMilivolts(const struct adc_dt_spec *adc_channel, int *val_mv)
{
    int8_t err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    if(err < 0)
    {
        printk("Value in mV is not avaiable\r\n");
        return err;
    }
    else
    {
        printk("ADC Reading: channel[%d] %d[mV]: %d\n\r",adc_channel->channel_id, *val_mv);
        return 0;
    }
}