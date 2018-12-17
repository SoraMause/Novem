#include "logger.h"

#include "variable.h"

#include <stdio.h>

t_log_data logger;

int8_t log_flag = 0;

static int16_t log_count = 0;

void log_init( void )
{
  log_count = 0;
  for ( int i = 0; i < 2048; i++ ){
    logger.sensor_left[i] = 0;
    logger.sensor_right[i] = 0;
    logger.distance[i] = 0.0f;
    logger.real_distance[i] = 0.0f;
  }
}

void setLog( void )
{
  if ( log_flag == 1 ){
    if ( log_count < 2048 ){
      logger.sensor_left[log_count] = sen_fl_log.now;
      logger.sensor_right[log_count] = sen_fr_log.now;
      logger.distance[log_count] = rotation_ideal.velocity;
      logger.real_distance[log_count] = rotation_real.velocity;
      log_count++;
    } else {
      log_flag = 0;
    }
  }

}

void showLog( void )
{
  printf( "ideal dis, real dis, sen_l, sen_r \r\n" );
  for ( int i  = 0; i < log_count; i++ ){
    printf( "%f,%f,%d,%d\r\n",logger.distance[i],logger.real_distance[i], logger.sensor_left[i], logger.sensor_right[i] );
  }
}

void setLogFlag( int8_t _flag )
{
  log_flag = _flag;
}