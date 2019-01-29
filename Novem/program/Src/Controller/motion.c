#include "motion.h"

#include "run.h"

#include "maze.h"

void setSlaromOffset( t_slarom_parameter *slarom, float left_in, float left_out, float right_in, 
                      float right_out, float ang_accel, float max_ang_vel )
{
  slarom->left.in = left_in;
  slarom->left.out = left_in;
  slarom->right.in = right_in;
  slarom->right.out = right_out;
  slarom->angular_accel = ang_accel;
  slarom->max_angular_velocity = max_ang_vel;
}

void setNormalRunParam( t_normal_param *param, float accel, float max_vel )
{
  param->accel = accel;
  param->velocity = max_vel;
}

void adjFront( float accel, float run_vel )
{ 
  wall_out_flag = 1;          // 壁切れを読むことを許可
  sidewall_control_flag = 1;  // 壁制御有効
  setStraight( ADJ_FRONT_DISTANCE, accel, run_vel, 0.0f, run_vel );
  waitStraight();
}

void adjBack( void )
{
  setStraight( -55.0f, 1000, 100.0f, 0.0f, 0.0f );
  waitStraight();
}

void straightOneBlock( float run_vel )
{
  wall_out_flag = 2;          // 壁切れを読むことを許可
  sidewall_control_flag = 1;  // 壁制御有効
  setStraight( ONE_BLOCK_DISTANCE, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
}

void straightHalfBlockStop( float accel , float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  frontwall_control_flag = 1;
  setStraight( HALF_BLOCK_DISTANCE, accel, run_vel, run_vel, 0.0f );
  waitStraight(); 
  waitMotion( 300 );
}

void straightHalfBlockStart( float accel , float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  frontwall_control_flag = 1;
  setStraight( HALF_BLOCK_DISTANCE, accel, run_vel, 0.0f, run_vel );
  waitStraight(); 
}

void pivoTurnLeft( float accel, float run_vel )
{
  setRotation( 90.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void pivoTurnRight( float accel, float run_vel )
{
  setRotation( -90.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void pivoTurn180( float accel, float run_vel )
{
  setRotation( 180.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void slaromLeft( float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.left.in, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
  setRotation( 90.0f, slarom500.angular_accel, slarom500.max_angular_velocity, run_vel );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.left.out, 0.0f, run_vel, run_vel, run_vel );
  //waitStraight();
  waitSlaromOut();
}

void slaromRight( float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.right.in, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
  setRotation( -90.0f, slarom500.angular_accel, slarom500.max_angular_velocity, run_vel );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.right.out, 0.0f, run_vel, run_vel, run_vel );
  //waitStraight();
  waitSlaromOut();
}

// to do 最短のやつを作りまくる
void runStraight( float accel , float distance, float start_vel, float run_vel, float end_vel )
{
  setStraight( distance, accel, run_vel, start_vel, end_vel );
  waitStraight(); 
}

// 中心から90度
void slaromCenterLeft( float accel )
{

}

void slaromCenterRight( float accel )
{

}

// 中心から180度
void slaromCenterLeft180( float accel )
{

}

void slaromCenterRight180( float accel )
{

}

// 中心から45度
void slaromCenterLeft45( float accel )
{

}

void slaromCenterRight45( float accel )
{

}

// 中心から135度
void slaromCenterLeft135( float accel )
{

}

void slaromCenterRight135( float accel )
{

}

// 斜め90度 ( V90 )
void slaromLeftV90( void )
{

}

void slaromRightV90( void )
{

}

// 斜めから復帰
void slaromReturnDiaLeft45( void )
{

}

void slaromReturnDiaRight45( void )
{

}

// 斜めから135度ターン復帰
void slaromReturnDiaLeft135( void )
{

}

void slaromReturnDiaRight135( void )
{

}