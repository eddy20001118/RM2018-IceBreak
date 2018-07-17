#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "pid.h"
#include "sys.h"
#include  "gimbal_task.h"
 
 /**
	0,1 守门 moving rolling
	2,3 中场
	*/
int16_t test_moto_position[4];
int16_t test_moto_speed[4] = {0,0,0,0};
int16_t test_moto_current[4];
uint8_t test_servo;
uint8_t key1;
uint16_t ANGLE_MAX_VALUE[3] = {90, 370, 260};

int SpeedP[] = {28, 8, 8};
int SpeedD[] = {175, 100, 110};
int PositionP[] = {8, 8, 8};
int PositionD[] = {3, 3, 3};

void test_moto_control(void)
{
   //LED控制函数
	 for(int i=0;i<12;i++){
	 write_led_io(i,LED_ON);	
	 }
   //电机的速度给定
   test_moto_position[0] = rc.ch2 / RC_MAX_VALUE * ANGLE_MAX_VALUE[1]; //moving
   test_moto_position[1] = rc.ch1 / RC_MAX_VALUE * ANGLE_MAX_VALUE[0]; //rolling
	 test_moto_position[2] = rc.ch4 / RC_MAX_VALUE * ANGLE_MAX_VALUE[2]; //moving
   test_moto_position[3] = rc.ch3 / RC_MAX_VALUE * 90; //rolling

   test_moto_speed[0] = pid_calc(&pid_test_moto_position_guardmove, moto_chassis[0].total_angle/19.2, test_moto_position[0]);
   test_moto_speed[1] = pid_calc(&pid_test_moto_position_1,         moto_chassis[1].total_angle/19.2, test_moto_position[1]);
   test_moto_speed[2] = pid_calc(&pid_test_moto_position_move,      moto_chassis[2].total_angle/19.2, test_moto_position[2]);
   test_moto_speed[3] = pid_calc(&pid_test_moto_position,           moto_chassis[3].total_angle/19.2, test_moto_position[3]);
   
	//闭环计算电机电流
    test_moto_current[0] = pid_calc(&pid_test_moto_speed_guardmove, moto_test.speed_rpm, test_moto_speed[0]);
	  test_moto_current[1] = pid_calc(&pid_test_moto_speed_1,         moto_test.speed_rpm, test_moto_speed[1]);
	  test_moto_current[2] = pid_calc(&pid_test_moto_speed_move,      moto_test.speed_rpm, test_moto_speed[2]);
	  test_moto_current[3] = pid_calc(&pid_test_moto_speed,           moto_test.speed_rpm, test_moto_speed[3]);
   
   //发送电机的电流
   set_test_motor_current(test_moto_current);
	 
	    //舵机控制函数周期设定
   set_pwm_group_param(PWM_GROUP1,20000);
	
   //开启控制端口
   start_pwm_output(PWM_IO1);
    
	//舵机控制命令
		if(test_servo == 0 )
		  {
			set_pwm_param(PWM_IO1,2200);
		  }
		else
		  {
		  set_pwm_param(PWM_IO1,1500);
		  }
   

//  扩展板的按键控制电机
//      read_key_io(KEY_IO1,&key1);  
//			if(key1 ==1)
//		{
//			 set_test_motor_current(test_moto_current);
//			}
			
			
			
}
   //电机初始化参数设定
	void test_moto_init(void)
		{
			 pid_init(&pid_test_moto_speed, 7000, 0, 15, 0, 100);
	     pid_init(&pid_test_moto_position, 7000, 0, 8, 0, 3);
			 
			 pid_init(&pid_test_moto_speed_1, 7000, 0, 15, 0, 100);
	     pid_init(&pid_test_moto_position_1, 7000, 0, 8, 0, 3);
		 
		   pid_init(&pid_test_moto_speed_move, 7000, 0, 28, 0, 175);
	     pid_init(&pid_test_moto_position_move, 7000, 0, PositionP[1], 0, PositionD[1]);
		 
		   pid_init(&pid_test_moto_speed_guardmove, 7000, 0, SpeedP[2], 0, SpeedD[2]);
	     pid_init(&pid_test_moto_position_guardmove, 7000, 0, PositionP[2], 0, PositionD[2]);
	}	
