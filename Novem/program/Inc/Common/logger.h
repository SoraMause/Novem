#ifndef __LOGGER_H
#define __LOGGER_H

#include <stdint.h>

typedef struct {
  float distance[2048];
  float real_distance[2048];
  int16_t sensor_left[2048];
  int16_t sensor_right[2048];
}t_log_data;

extern t_log_data logger;

void log_init( void );
void setLog( void );
void showLog( void );
void setLogFlag( int8_t _flag );

#endif /* __LOGGER_H */