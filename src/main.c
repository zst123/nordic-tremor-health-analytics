/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

/********************************************************/
/* GPIO LED */
/********************************************************/

#include <drivers/gpio.h>

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0_DEVICE DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN  DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS  DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

/* The devicetree node identifier for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1_DEVICE DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN  DT_GPIO_PIN(LED1_NODE, gpios)
#define LED1_FLAGS  DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

struct device *dev_led0;
struct device *dev_led1;

void my_led_init() {
  dev_led0 = (struct device *) device_get_binding(LED0_DEVICE);
  if (dev_led0 != NULL) {
    gpio_pin_configure(dev_led0, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
  }

  dev_led1 = (struct device *) device_get_binding(LED1_DEVICE);
  if (dev_led1 != NULL) {
    gpio_pin_configure(dev_led1, LED1_PIN, GPIO_OUTPUT_ACTIVE | LED1_FLAGS);
  }
}


/********************************************************/
/* ADC */
/********************************************************/

#include <drivers/adc.h>
#include <hal/nrf_saadc.h>

#define ADC_DEVICE_NAME       DT_LABEL(DT_INST(0, nordic_nrf_saadc))
#define ADC_RESOLUTION        12
#define ADC_REFERENCE         ADC_REF_INTERNAL // 0.6V reference
#define ADC_GAIN              ADC_GAIN_1_6 // 3.6V max input
#define ADC_ACQUISITION_TIME  NRF_SAADC_ACQTIME_3US // 3us // 10usec acq time
#define ADC_BUFFER_SIZE       2

static int16_t adc_sample_buffer[ADC_BUFFER_SIZE];

#define ADC_1ST_CHANNEL_ID    0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0
// #define ADC_2ND_CHANNEL_ID    2
// #define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2
// #define ADC_3RD_CHANNEL_ID    3
// #define ADC_3RD_CHANNEL_INPUT NRF_SAADC_INPUT_AIN3

struct adc_channel_cfg channel_cfg = {
  .gain = ADC_GAIN,
  .reference = ADC_REFERENCE,
  .acquisition_time = ADC_ACQUISITION_TIME,
  /* channel ID will be overwritten below */
  .channel_id = 0,
  .differential = 0,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
  .input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};

struct adc_sequence sequence = {
  /* individual channels will be added below */
  .channels    = 0,
  .buffer      = adc_sample_buffer,
  /* buffer size in bytes, not number of samples */
  .buffer_size = sizeof(adc_sample_buffer),
  .resolution  = ADC_RESOLUTION,
};

struct device *dev_adc0;

inline const struct device *get_adc_device(void) {
  //return device_get_binding(ADC_DEVICE_NAME);
  return dev_adc0;
}

static void my_adc_init(void) {
  dev_adc0 = device_get_binding(ADC_DEVICE_NAME);
  if (!device_is_ready(dev_adc0)) {
    printk("ADC device not found\n");
    return;
  }

  // Configure channels individually prior to sampling
  channel_cfg.channel_id = ADC_1ST_CHANNEL_ID;
  adc_channel_setup(dev_adc0, &channel_cfg);

  // Add bit of channel to sequence
  sequence.channels |= BIT(ADC_1ST_CHANNEL_ID);
}

int32_t my_adc_read(void) {
  const struct device *dev_adc = get_adc_device();

  // Read sequence of channels (fails if not supported by MCU)
  __disable_irq();
  int err = adc_read(dev_adc, &sequence);
  __enable_irq();

  if (err != 0) {
    printk("ADC reading failed with error %d.\n", err);
    return -1;
  }

  return adc_sample_buffer[0];
}

void my_adc_test(void) {
  const struct device *dev_adc = get_adc_device();
  int32_t adc_vref = adc_ref_internal(dev_adc);

  // Read sequence of channels (fails if not supported by MCU)
  int err = adc_read(dev_adc, &sequence);
  if (err != 0) {
    printk("ADC reading failed with error %d.\n", err);
    return;
  }

  printk("ADC reading:");
  for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    int32_t raw_value = adc_sample_buffer[i];
    printk(" %d", raw_value);
    if (adc_vref > 0) {
      /*
       * Convert raw reading to millivolts if driver
       * supports reading of ADC reference voltage
       */
      int32_t mv_value = raw_value;

      adc_raw_to_millivolts(adc_vref, ADC_GAIN,
              ADC_RESOLUTION, &mv_value);
      printk(" = %d mV", mv_value);
    }
    printk("    ");
  }
  printk("\n");
}


/********************************************************/
/* FFT */
/********************************************************/
#include "arm_math.h"
#include "arm_const_structs.h"

#define FFT_INPUT_SIZE 1024
#define FFT_OUTPUT_SIZE 512

// FFT variables
int32_t ADC_Values[FFT_INPUT_SIZE/2];

float32_t FFT_Input[FFT_INPUT_SIZE]; // Real and imaginary parts
float32_t FFT_Output[FFT_OUTPUT_SIZE];
float32_t FFT_maxValue = 0;          // Max FFT value is stored here
uint32_t  FFT_maxIndex = 0;          // Index in Output array where max value is

uint32_t  FFT_intFlag = 0;
uint32_t  FFT_doBitReverse = 1;

void FFT_Process() {
  // Copy ADC values into the input array
  // Note that the arm_cfft_f32() function
  // will directly modify the input array.
  for (int i = 0; i < FFT_INPUT_SIZE; i += 2) {
    // Convert to float upon 1.0f
    const uint16_t adcMaxValue = (1U << ADC_RESOLUTION) - 1U;
    float32_t adcPercentage = (ADC_Values[i/2] * 1.0f / adcMaxValue);
  
    // Insert into FFT_Input array
    FFT_Input[i] = adcPercentage - 0.5f; // Real part is copied in (removed DC offset)
    FFT_Input[i+1] = 0; // Imaginary part is always zero
  }

  /* Process the data through the CFFT/CIFFT module */
  arm_cfft_f32(&arm_cfft_sR_f32_len512, FFT_Input, FFT_intFlag, FFT_doBitReverse);

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  arm_cmplx_mag_f32(FFT_Input, FFT_Output, FFT_OUTPUT_SIZE);

  /* Calculates maxValue and returns corresponding BIN value */
  arm_max_f32(&FFT_Output[0], FFT_OUTPUT_SIZE/2, &FFT_maxValue, &FFT_maxIndex);
}

void FFT_PrintOutputValues() {
  for (int i = 0; i < FFT_OUTPUT_SIZE; i++) {
    printk("%d,%d\n", i, (int) (FFT_Output[i] * 10000));
  }
}

void FFT_PrintInputValues() {
  for (int i = 0; i < FFT_INPUT_SIZE; i++) {
    printk("%d,%d\n", i, (int) (FFT_Input[i] * 10000));
  }
}


/********************************************************/
/* TIMER */
/********************************************************/

#include <drivers/timer/nrf_rtc_timer.h>
#include <hal/nrf_timer.h>
#include <irq.h>

int16_t timer_count = 0;
bool timer_trigger = false;

ISR_DIRECT_DECLARE(timer0_isr_wrapper) {
  nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
  
  // Handler event
  gpio_pin_toggle(dev_led0, LED0_PIN);

  timer_trigger = true;

  timer_count += 1;
  if (timer_count*2 >= FFT_INPUT_SIZE) {
    timer_count = 0;
  }

  if ((timer_count % 64) == 0) {
   gpio_pin_toggle(dev_led1, LED1_PIN);
  }

  //my_adc_test();
  return 0;
}

static void my_timer0_init(void) {
  nrf_timer_mode_set(NRF_TIMER0, NRF_TIMER_MODE_TIMER);
  nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_16);
  nrf_timer_frequency_set(NRF_TIMER0, NRF_TIMER_FREQ_16MHz);
  nrf_timer_cc_set(NRF_TIMER0, 0, 31250); // 16 MHz / 512 Hz = 31250 counts
  
  nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
  nrf_timer_shorts_enable(NRF_TIMER0, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

  IRQ_DIRECT_CONNECT(TIMER0_IRQn, 2,
                     timer0_isr_wrapper,
                     COND_CODE_1(CONFIG_ZERO_LATENCY_IRQS, (IRQ_ZERO_LATENCY), (0))
                    );
  irq_enable(TIMER0_IRQn);
}

static void my_timer0_start(void) {
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);
}

static void my_timer0_stop(void) {
  nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
}

/********************************************************/
/* I2C OLED */
/********************************************************/

#include <drivers/i2c.h>
#include "hackster_logo.h"

#define I2C_DEV DT_LABEL(DT_NODELABEL(i2c1))

#define SSD1306_STREAM (device_get_binding(I2C_DEV))

struct i2c_msg I2C_msgs[1];
uint8_t I2C_payload[64];
uint8_t I2C_len = 0;
uint8_t I2C_addr = 0;

void I2C_Start(const struct device *i2c_dev) {
  // Check if device binding is valid
  if (!i2c_dev) {
    printk("I2C: Device driver not found.\n");
    return;
  }

  // Configure to 400kHz Fast Mode
  if (i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_FAST))) {
    printk("I2C config failed\n");
    return;
  }

  // printk("I2C: Initialised successfully.\n");
  I2C_len = 0;
}

void I2C_SetAddress(const struct device *i2c_dev, uint8_t addr) {
  I2C_addr = addr;
  I2C_len = 0;
}

void I2C_Write(const struct device *i2c_dev, uint8_t data) {
  /* Fill up buffer with single byte */
  I2C_payload[I2C_len] = data;
  I2C_len += 1;

  I2C_msgs[0].buf = I2C_payload;
  I2C_msgs[0].len = I2C_len;
  I2C_msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
}

int I2C_Stop(const struct device *i2c_dev) {
  return i2c_transfer(i2c_dev, &I2C_msgs[0], 1, I2C_addr);
}

#include "SSD1306OLED.h"

void my_oled_init(void) {
  // Initialize the SSD1306 OLED with 7-bit address of 0x3C
  SSD1306_Begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);

  // Clear the display
  SSD1306_ClearDisplay();
  SSD1306_Display();
}

/********************************************************/
/* Main App */
/********************************************************/

void main(void) {
  // Initialize devices
  my_led_init();
 
  my_adc_init();

  my_timer0_init();
  my_timer0_start();

  my_oled_init();

  // Test OLED
  SSD1306_DrawBMP(0, 0, (uint8_t *) &hackster_logo[0], hackster_logo_width, hackster_logo_height);
  
  SSD1306_DrawText(5, 32, "  Tremor  ", 2);
  SSD1306_DrawText(5, 32 + 8*2, "  XXX Hz  ", 2);
  //SSD1306_DrawText(5, 32, "0123456789", 2);
  //SSD1306_FillCircle(128-10, 64-10, 8, true);

  SSD1306_Display();

  while (1) {
    while (timer_trigger == false);
    
    // Insert into array
    ADC_Values[timer_count] = my_adc_read();

    if (timer_count == (FFT_INPUT_SIZE/2 - 1)) {

      FFT_Process();
      printk("FFT---- \n");
      //FFT_PrintOutputValues();
      //FFT_PrintInputValues();

      printk("max %d -> %f\n", FFT_maxIndex, FFT_maxValue);

      if (1 < FFT_maxValue && FFT_maxValue < 4000) {
        char line2[20];
        snprintf(line2, 20,     "  %3d Hz  ", FFT_maxIndex);
        SSD1306_DrawText(5, 32, "  Tremor  ", 2);
        SSD1306_DrawText(5, 32 + 8*2, line2, 2);
        SSD1306_Display();
      }

      timer_count = 0;
    }

    //gpio_pin_toggle(dev_led0, LED0_PIN);
    //k_msleep(100);
    
    
    //my_adc_read();
  }
}

