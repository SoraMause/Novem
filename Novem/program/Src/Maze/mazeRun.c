#include "search.h"

#include "maze.h"
#include "agent.h"

#include "tim.h"

#include "variable.h"

#include "mode.h"

#include "motion.h"
#include "run.h"
#include "timer.h"
#include "buzzer.h"
#include "led.h"

void adachiSearchRun( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_position *pos, uint8_t maze_scale )
{
  int8_t next_dir = front;
  adjFront( translation->accel, translation->velocity );
  mazeUpdatePosition( front, pos );
  while( pos->x != gx || pos->y != gy ){
    addWall( pos, wall );
    mazeUpdateMap( gx, gy, wall, maze_scale );
    next_dir = getNextDir( pos->direction,pos->x, pos->y, wall, maze_scale );

    switch( next_dir ){
      case front:
        fullColorLedOut( LED_BLUE );
        mazeUpdatePosition( front, pos );
        straightOneBlock( translation->velocity );
        break;

      case left:
        fullColorLedOut( LED_GREEN );
        mazeUpdatePosition( left, pos );
        slaromLeft( translation->velocity );
        break;

      case right:
        fullColorLedOut( LED_MAGENTA );
        mazeUpdatePosition( right, pos );
        slaromRight( translation->velocity );
        break;

      case rear:
        fullColorLedOut( LED_RED );
        mazeUpdatePosition( rear, pos );
        straightHalfBlockStop( translation->accel, translation->velocity );
        pivoTurn180( rotation->accel, rotation->velocity );
        adjBack();
        adjFront( translation->accel, translation->velocity );
        break;
    }
  }
    
  addWall( pos, wall );
  straightHalfBlockStop( translation->accel, translation->velocity );
  waitMotion( 300 );
  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
}

void adachiFastRun( t_normal_param *translation, t_normal_param *rotation )
{
  while( motion_last < motion_end ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SLAROM_LEFT:
        slaromLeft( translation->velocity );
        break;

      case SLAROM_RIGHT:
        slaromRight( translation->velocity );
        break;

      case FRONTPD_DELAY:
        waitMotion( 300 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        sidewall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;
    } // end switch 
    motion_last++;
  }

  waitMotion( 300 );

  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
}

void adachiFastRunDiagonal( t_normal_param *translation, t_normal_param *rotation )
{
  #if 0
  setControlFlag( 0 );
  funControl( FUN_ON );
  HAL_Delay( 1000 );
  setControlFlag( 1 );
  #endif

  while( motion_last < motion_end ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      // 中心から90度
      case CENRTER_SLAROM_LEFT:
        slaromCenterLeft();
        break;

      case CENRTER_SLAROM_RIGHT:
        slaromCenterRight();
        break;

      // 中心から180度
      case SLAROM_LEFT_180:
        slaromCenterLeft180();
        break;

      case SLAROM_RIGHT_180:
        slaromCenterRight180();
        break;

      // 中心から45度
      case DIA_CENTER_LEFT:
        slaromCenterLeft45();
        break;

      case DIA_CENTER_RIGHT:
        slaromCenterRight45();
        break;

      // 中心から135度
      case DIA_CENTER_LEFT_135:
        slaromCenterLeft135();
        break;

      case DIA_CENTER_RIGHT_135:
        slaromCenterRight135();
        break;

      // 斜め90度 ( V90 )
      case DIA_LEFT_TURN:
        slaromLeftV90();
        break;

      case DIA_RIGHT_TURN:
        slaromRightV90();
        break;

      // 斜めから復帰
      case RETURN_DIA_LEFT:
        slaromReturnDiaLeft45();
        break;

      case RETURN_DIA_RIGHT:
        slaromReturnDiaRight45();
        break;

      // 斜めから135度ターン復帰
      case RETURN_DIA_LEFT_135:
        slaromReturnDiaLeft135();
        break;

      case RETURN_DIA_RIGHT_135:
        slaromReturnDiaRight135();
        break;

      case FRONTPD_DELAY:
        // 前壁制御有効にする
        waitMotion( 300 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;
    } // end switch 
    motion_last++;
  }

  funControl( FUN_OFF );
  waitMotion( 300 );

  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
}