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
  setStraight( 90.0f, accel, run_vel, run_vel, 0.0f );
  waitStraight(); 
  waitMotion( 300 );
}

void straightHalfBlockStart( float accel , float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  frontwall_control_flag = 1;
  setStraight( 90.0f, accel, run_vel, 0.0f, run_vel );
  waitStraight(); 
  waitMotion( 300 );
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
  setRotation( -180.0f, accel, run_vel, 0.0f );
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
void slaromCenterLeft( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 75.5f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( 90.0f, 6740.0f, 540.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 75.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

void slaromCenterRight( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 75.5f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( -90.0f, 6740.0f, 540.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 75.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

// 中心から180度
void slaromCenterLeft180( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 42.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( 180.0f, 6300.0f, 450.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 40.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

void slaromCenterRight180( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 42.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( -180.0f, 6300.0f, 450.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 40.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

// 中心から45度
void slaromCenterLeft45( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 44.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( 45.0f, 12000.0f, 630.0f, 700.0f );
  waitRotation();
  setStraight( 81.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

void slaromCenterRight45( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 44.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( -45.0f, 12000.0f, 630.0f, 700.0f );
  waitRotation();
  setStraight( 81.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

// 中心から135度
void slaromCenterLeft135( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 50.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( 135.0f, 5400.0f, 540.0f, 700.0f );
  waitRotation();
  setStraight( 32.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

void slaromCenterRight135( void )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 50.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( -135.0f, 5400.0f, 540.0f, 700.0f );
  waitRotation();
  setStraight( 32.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

// 斜め90度 ( V90 )
void slaromLeftV90( void )
{
  setStraight( 26.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( 90.0f, 6300.0f, 630.0f, 700.0f );
  waitRotation();
  setStraight( 25.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();  
}

void slaromRightV90( void )
{
  setStraight( 26.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( -90.0f, 6300.0f, 630.0f, 700.0f );
  waitRotation();
  setStraight( 25.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();    
}

// 斜めから復帰
void slaromReturnDiaLeft45( void )
{
  setStraight( 78.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( 45.0f, 10000.0f, 630.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 39.5f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

void slaromReturnDiaRight45( void )
{
  setStraight( 78.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( -45.0f, 10000.0f, 630.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 39.5f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

// 斜めから135度ターン復帰
void slaromReturnDiaLeft135( void )
{
  setStraight( 54.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( 135.0f, 5760.0f, 630.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 67.5f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}

void slaromReturnDiaRight135( void )
{
  setStraight( 54.0f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
  setRotation( -135.0f, 5760.0f, 630.0f, 700.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 67.5f, 0.0f, 700.0f, 700.0f, 700.0f );
  waitStraight();
}