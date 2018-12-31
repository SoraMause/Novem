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

#define SEARCH_MAX_TIME 150000

void adachiSearchRun( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale )
{

  int8_t next_dir = front;

  // もし、ゴール座標が探索ならマシンの動作時間を0にする。 
  if ( gx != 0 && gy != 0 ){
    cnt_act = 0;
  }

  mazeUpdatePosition( front, pos );
  adjFront( translation->accel, translation->velocity );
  while( pos->x != gx || pos->y != gy ){
    addWall( pos, wall );
    addWall( pos, bit ); 
    mazeUpdateMap( gx, gy, wall, maze_scale );
    next_dir = getNextDir( pos->direction,pos->x, pos->y, wall, maze_scale );

    switch( next_dir ){
      case front:
        mazeUpdatePosition( front, pos );
        straightOneBlock( translation->velocity );
        break;

      case left:
        mazeUpdatePosition( left, pos );
        slaromLeft( translation->velocity );
        //straightHalfBlockStop( translation->accel, translation->velocity );
        //pivoTurnLeft( rotation->accel, rotation->velocity );
        //straightHalfBlockStart( translation->accel, translation->velocity );
        break;

      case right:
        mazeUpdatePosition( right, pos );
        slaromRight( translation->velocity );
        //straightHalfBlockStop( translation->accel, translation->velocity );
        //pivoTurnRight( rotation->accel, rotation->velocity );
        //straightHalfBlockStart( translation->accel, translation->velocity );
        break;

      case rear:

        straightHalfBlockStop( translation->accel, translation->velocity );
        mazeUpdatePosition( rear, pos );
        pivoTurn180( rotation->accel, rotation->velocity );
        adjBack();
        adjFront( translation->accel, translation->velocity );
        break;

      case pivo_rear:
        straightHalfBlockStop( translation->accel, translation->velocity );
        mazeUpdatePosition( rear, pos );
        pivoTurn180( rotation->accel, rotation->velocity );
        straightHalfBlockStart( translation->accel, translation->velocity );
        break;
    }

    // 探索時間が2分30秒以上たっていた場合打ち切り。
    if( cnt_act > SEARCH_MAX_TIME ) break;
  }
    
  addWall( pos, wall );
  addWall( pos, bit ); 
  straightHalfBlockStop( translation->accel, translation->velocity );
  waitMotion( 300 );
  pivoTurn180( rotation->accel, rotation->velocity );
  adjBack();
  mypos.direction = (mypos.direction + 2) % 4;
  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
}

void adachiSearchRunKnown( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale )
{
  int8_t next_dir = front;
  int8_t block = 0;

  // もし、ゴール座標が探索ならマシンの動作時間を0にする。 
  if ( gx != 0 && gy != 0 ){
    cnt_act = 0;
  }

  mazeUpdatePosition( front, pos );
  adjFront( translation->accel, translation->velocity );
  while( pos->x != gx || pos->y != gy ){
    addWall( pos, wall );
    addWall( pos, bit ); 
    mazeUpdateMap( gx, gy, wall, maze_scale );
    next_dir = getNextDirKnown( pos->direction,pos->x, pos->y, wall, bit, maze_scale );

    if ( next_dir == front ){
        mazeUpdatePosition( front, pos );
        straightOneBlock( translation->velocity );
    } else if ( next_dir == left ){
        mazeUpdatePosition( left, pos );
        slaromLeft( translation->velocity );
    } else if ( next_dir == right ){
        mazeUpdatePosition( right, pos );
        slaromRight( translation->velocity );
    } else if ( next_dir == rear ){
      straightHalfBlockStop( translation->accel, translation->velocity );
      mazeUpdatePosition( rear, pos );
      pivoTurn180( rotation->accel, rotation->velocity );
      adjBack();
      adjFront( translation->accel, translation->velocity );
    } else if ( next_dir == pivo_rear ){
      straightHalfBlockStop( translation->accel, translation->velocity );
      mazeUpdatePosition( rear, pos );
      pivoTurn180( rotation->accel, rotation->velocity );
      straightHalfBlockStart( translation->accel, translation->velocity );
    } else if ( next_dir > 10 ){
        sidewall_control_flag = 1;  // 壁制御有効
        block = next_dir - 10;
        mazeUpdatePosition( next_dir, pos );
        runStraight( translation->accel, ONE_BLOCK_DISTANCE * block, translation->velocity, 
                    translation->velocity + 300.0f, translation->velocity );
    }

    // 探索時間が2分30秒以上たっていた場合打ち切り。
    if( cnt_act > SEARCH_MAX_TIME ) break;

  }
    
  addWall( pos, wall );
  addWall( pos, bit ); 
  straightHalfBlockStop( translation->accel, translation->velocity );
  waitMotion( 300 );
  pivoTurn180( rotation->accel, rotation->velocity );
  adjBack();
  mypos.direction = (mypos.direction + 2) % 4;
  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
  buzzerSetMonophonic( NORMAL, 200 );
  waitMotion( 300 );
}

void adachiFastRun( t_normal_param *translation, t_normal_param *rotation )
{
  while( motion_queue[motion_last] != 0 ){
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

  while( motion_queue[motion_last] != 0 ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        //dirwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        dirwall_control_flag = 0;
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
        waitMotion( 100 );
        break;

      case DELAY:
        waitMotion( 100 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;

      default:
        break;
    } // end switch 
    motion_last++;
  }

  buzzerSetMonophonic( NORMAL, 200 );
  setControlFlag( 0 );
  waitMotion( 300 );
}