#include "function.h"
// peripheeral
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"

//common
#include "led.h"
#include "buzzer.h"
#include "flash.h"

#define OFF_VALUE 0
#define LEFT_VALUE 1
#define RIGHT_VALUE 2
#define FINISH_CONVERT 3

static float batt_calc_const = 0.0f;
static uint8_t ctr_irled = 0;

///////////////////////////////////////////////////////////////////////
// machine_init
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
void machine_init( void )
{
  setbuf( stdout, NULL );
  setIrledPwm( IRLED_OFF );
  failSafe_flag = 0;
  buzzerSetMonophonic( NORMAL, 100 );
  HAL_Delay( 101 );
  certainLedOut( LED_OFF );
  fullColorLedOut( LED_OFF );
  MPU6500_init();
  buzzerSetMonophonic( NORMAL, 100 );
  HAL_Delay( 100 );
  HAL_TIM_Encoder_Start( &htim1, TIM_CHANNEL_ALL ); // encoder
  HAL_TIM_Encoder_Start( &htim8, TIM_CHANNEL_ALL ); // encoder
  batt_calc_const = 3.3f / 4096.0f * ( 1000.0f + 390.0f ) / 390.0f;
  mode_counter = 0;
  mode_distance = 0.0f;
  HAL_Delay( 300 );
  HAL_GPIO_WritePin( stby_GPIO_Port, stby_Pin, GPIO_PIN_SET );

  //MPU6500_z_axis_offset_calc_start();
}

///////////////////////////////////////////////////////////////////////
// batt voltage calculation
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
float battMonitor( int16_t data )
{
  float batt_voltage;
  //batt_voltage = 3.3 * batt_analog / 1024.0;
  batt_voltage = (float)( batt_calc_const * data );
  return batt_voltage;
}

///////////////////////////////////////////////////////////////////////
// able ir led
// [argument] ired 1:paluse emit ON 0:OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void setIrledPwm( uint8_t able )
{
	ctr_irled = able;
}


void setSensorConstant( t_sensor *sen, int16_t reference, int16_t threshold )
{
  sen->reference = reference;
  sen->threshold = threshold;
}

void update_sensor_data( void )
{

  sen_front.now = ( sen_fl.now + sen_fr.now ) / 2;

  sen_fr.diff = ( sen_fr_log.now - sen_fr_log.before_3ms );
  if ( sen_fr.diff < 0 ) sen_fr.diff = -1 * sen_fr.diff;
  sen_fr.diff_1ms = ( sen_fr_log.now - sen_fr_log.before_1ms );
  if ( sen_fr.diff_1ms < 0 ) sen_fr.diff_1ms = -1 * sen_fr.diff_1ms;

  sen_r.diff = ( sen_r_log.now - sen_r_log.before_3ms );
  if ( sen_r.diff < 0 ) sen_r.diff = -1 * sen_r.diff;
  sen_r.diff_1ms = ( sen_r_log.now - sen_r_log.before_1ms );
  if ( sen_r.diff_1ms < 0 ) sen_r.diff_1ms = -1 * sen_r.diff_1ms;

  sen_l.diff = ( sen_l_log.now - sen_l_log.before_3ms );
  if ( sen_l.diff < 0 ) sen_l.diff = -1 * sen_l.diff;
  sen_l.diff_1ms = ( sen_l_log.now - sen_l_log.before_1ms );
  if ( sen_l.diff_1ms < 0 ) sen_l.diff_1ms = -1 * sen_l.diff_1ms;

  sen_fl.diff = ( sen_fl_log.now - sen_fl_log.before_3ms );
  if ( sen_fl.diff < 0 ) sen_fl.diff = -1 * sen_fl.diff;
  sen_fl.diff_1ms = ( sen_fl_log.now - sen_fl_log.before_1ms );
  if ( sen_fl.diff_1ms < 0 ) sen_fl.diff_1ms = -1 * sen_fl.diff_1ms;

  if ( sen_front.now < sen_front.threshold ){
    sen_front.is_wall = 0;
  } else {
    sen_front.is_wall = 1;
  }

  if ( sen_l.now < sen_l.threshold ){
    sen_l.is_wall = 0;
  } else {
    sen_l.is_wall = 1;
  }

  if ( sen_r.now < sen_r.threshold ){
    sen_r.is_wall = 0;
  } else {
    sen_r.is_wall = 1;
  }
}

void adcStart( void )
{
  setIrledPwm( IRLED_ON );
  adc_counter = 0;
  HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
}

void adcEnd( void )
{
  setIrledPwm( IRLED_OFF );
}

void adcCheckConvert( void )
{
  if ( adc_counter == FINISH_CONVERT && ctr_irled == 1 ){
    update_sensor_data();
    adc_counter = 0;
    HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
  }
}

// DMA の変換式を記載
void getADSensor( int16_t *adcount )
{
  volatile int i;
  switch( *adcount ) {
    case OFF_VALUE:
      HAL_ADC_Stop_DMA( &hadc1 );
      ADCOffData[0] = ADCBuff[0];
      ADCOffData[1] = ADCBuff[1];
      ADCOffData[2] = ADCBuff[2];
      ADCOffData[3] = ADCBuff[3];

      HAL_GPIO_WritePin( sensor_paluseL_GPIO_Port, sensor_paluseL_Pin, GPIO_PIN_SET );
      for( i = 0; i < 50; i++ ){

      }

      *adcount = LEFT_VALUE;

      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
      break;

    case LEFT_VALUE:
      HAL_GPIO_WritePin( sensor_paluseL_GPIO_Port, sensor_paluseL_Pin, GPIO_PIN_RESET );
      HAL_ADC_Stop_DMA( &hadc1 );

      ADCOntData[2] = ADCBuff[2];
      ADCOntData[3] = ADCBuff[3];

      sen_l.now = ADCOntData[2] - ADCOffData[2];
      sen_l_log.before_5ms = sen_l_log.before_4ms;
      sen_l_log.before_4ms = sen_l_log.before_3ms;
      sen_l_log.before_3ms = sen_l_log.before_2ms;
      sen_l_log.before_2ms = sen_l_log.before_1ms;
      sen_l_log.before_1ms = sen_l_log.now;
      sen_l_log.now = sen_l.now;

      sen_fl.now = ADCOntData[3] - ADCOffData[3];
      sen_fl_log.before_5ms = sen_fl_log.before_4ms;
      sen_fl_log.before_4ms = sen_fl_log.before_3ms;
      sen_fl_log.before_3ms = sen_fl_log.before_2ms;
      sen_fl_log.before_2ms = sen_fl_log.before_1ms;
      sen_fl_log.before_1ms = sen_fl_log.now;
      sen_fl_log.now = sen_fl.now;

      HAL_GPIO_WritePin( sensor_paluseR_GPIO_Port, sensor_paluseR_Pin, GPIO_PIN_SET );
      for( i = 0; i < 400; i++ ){

      }

      *adcount = RIGHT_VALUE;

      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
      break;

    case RIGHT_VALUE:
      HAL_GPIO_WritePin( sensor_paluseR_GPIO_Port, sensor_paluseR_Pin, GPIO_PIN_RESET );
      HAL_ADC_Stop_DMA( &hadc1 );

      ADCOntData[0] = ADCBuff[0];
      ADCOntData[1] = ADCBuff[1];

      sen_fr.now = ADCOntData[0] - ADCOffData[0];
      sen_fr_log.before_5ms = sen_fr_log.before_4ms;
      sen_fr_log.before_4ms = sen_fr_log.before_3ms;
      sen_fr_log.before_3ms = sen_fr_log.before_2ms;
      sen_fr_log.before_2ms = sen_fr_log.before_1ms;
      sen_fr_log.before_1ms = sen_fr_log.now;
      sen_fr_log.now = sen_fr.now;

      sen_r.now = ADCOntData[1] - ADCOffData[1];
      sen_r_log.before_5ms = sen_r_log.before_4ms;
      sen_r_log.before_4ms = sen_r_log.before_3ms;
      sen_r_log.before_3ms = sen_r_log.before_2ms;
      sen_r_log.before_2ms = sen_r_log.before_1ms;
      sen_r_log.before_1ms = sen_r_log.now;
      sen_r_log.now = sen_r.now;

      *adcount = FINISH_CONVERT;
      for( i = 0; i < 100; i++ ){

      }
      break;

    default:
      break;
  }
}