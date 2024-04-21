// https://blog.csdn.net/m0_53966219/article/details/126711218
// https://blog.csdn.net/zhuoqingjoking97298/article/details/120093315
// PID: https://zhuanlan.zhihu.com/p/586532545?utm_id=0
// https://blog.csdn.net/weixin_42208428/article/details/122173575
// https://blog.csdn.net/weixin_43964993/article/details/112383192

#include "Project1.h"
#include <stdbool.h>
#include <stdint.h>
extern image2_t image2_use;			// use 1/3 of the original image (40 continuous lines in the middle)
extern image2_t image2_show;		// show 1/3 of the full size screen
extern image2_t image2_temp;		// show 1/3 of the full size screen
extern uint8_t image_ready;			// MT9V034 new data frame ready in buffer
extern uint8_t image_size;				// 0: full size; 1: half size; 2: 1/3 sized
extern int16_t encoder_speed;
static uint8_t start_line_count = 0; // 起点线检测次数
static uint32_t start_line_time = 0; // 通过起点线的时间
bool if_start_line = false; // 是否检测到起点线


uint16_t calculate_speed(uint16_t steering_signal) {
    // 使用分段函数根据舵机信号调整速度
    return 19;
}




// // Project#1: Line Following Robot (LFR)
// bool detect_start_line(image2_t image) {
//     int center_x = 80;
//     int center_y = 64;
//     int black_count = 0;

//     // 检测十字中心点附近的黑色像素
//     for (int i = center_y - 5; i <= center_y + 5; i++) {
//         for (int j = center_x - 5; j <= center_x + 5; j++) {
//             if (image[i][j] == 0) {
//                 black_count++;
//             }
//         }
//     }

//     // 简单的判定：如果中心区域有足够多的黑色像素，我们认为检测到了起点线
//     return black_count > 50; // 这个阈值可能需要根据实际情况调整
// }
// 外部变量，SysTick 计数器
extern volatile uint32_t sys_tick_counter;



void Project_LFR(void)
{
	uint16_t pw = 1500, pwt = 0;
	uint16_t dc = 0;
	uint16_t tloss = 0;				// target lost loop counter
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;
	
	image_size = 2;         // use 1/3 of the full size
        
	hsp_tft18_init();
	hsp_mt9v034_init();
	hsp_dci_dma_config();
	hsp_pwm_init();
	MEN_HIGH();					// enable H-bridge

	state_pha = PHA2();			state_phb = PHB2();
	state_pha_t = state_pha;	state_phb_t = state_phb;
	image_ready = RESET;
	
	while(1)
	{
        // if (!PUSH())			// push button pressed        
        // {
        //     //delay_1ms(50);		// de-jitter
        //     if (!PUSH())
        //     {
        //         while(!PUSH());
        //         dc = 0;
        //     }
        // }
		
		// state_pha = PHA2();			state_phb = PHB2();
		// if((state_pha_t != state_pha) || (state_phb_t != state_phb))
		// {
		// 	if(state_phb_t == state_phb)
		// 	{
		// 		if(SET == state_phb)
		// 		{
		// 			if(RESET == state_pha) dc++;
		// 			else if(0 < dc) dc--;
		// 		}
		// 		else
		// 		{
		// 			if(SET == state_pha) dc++;
		// 			else if(0 < dc) dc--;
		// 		}
		// 	}
		// 	else
		// 	{
		// 		if(SET == state_pha)
		// 		{
		// 			if(SET == state_phb) dc++;
		// 			else if(0 < dc) dc--;
		// 		}
		// 		else
		// 		{
		// 			if(RESET == state_pha) dc++;
		// 			else if(0 < dc) dc--;
		// 		}
		// 	}
		// 	state_pha_t = state_pha;
		// 	state_phb_t = state_phb;
        //     //delay_1ms(10);		// de-jitter
		// }
		
		// hsp_tft18_show_int16(8, 0, dc);
        // PWM output stage, subjected to duty cycle limits
		// if(35 < dc)
		// 	dc = 35;
		
		// camera image processing
		if(image_ready == SET)
		{
			//threshold = hsp_image2_threshold_otsu(image2_use);
			//threshold = hsp_image2_threshold_mean(image2_use);
			//threshold = hsp_image2_threshold_minmax(image2_use);
			//hsp_image2_show_dma(image2_use);
			//hsp_image2_show_dma(image2_show);
			// if(detect_start_line(image2_temp))
            // {
            //     start_line_count++;
            //     if(start_line_count == 1) // 第一次检测到起点线
            //     {
            //         start_line_time = sys_tick_counter; // 记录时间
            //     }
            //     else if(start_line_count == 2) // 第二次检测到起点线
            //     {
            //         uint32_t current_time = sys_tick_counter;
            //         uint32_t time_elapsed = current_time - start_line_time;

            //         // 假设小车的速度约为1米/秒
            //         if(time_elapsed > 500) // 如果超过1000毫秒
            //         {
            //             hsp_motor_voltage(MOTORF, 0); // 停止电机
            //             break; // 退出循环
            //         }
            //     }
            // }
			if(SW2())
			{
				hsp_motor_voltage(MOTORF, dc);		// run forward
			}
			else
			{
				hsp_motor_voltage(MOTORB, dc);		// run backward
			}
			
			hsp_image2_binary_minmax(image2_use, image2_temp);
			pw = hsp_image_judge(image2_temp);
			dc = calculate_speed(pw);

			//detect initial line
			// if_start_line = detect_start_line(image2_temp);
			//如果检测到起点线，LED闪烁	
			// if(if_start_line) {
			// 	LED1_TOGGLE();
			// }
			// //如果检测到起点线，且通过了起点线，减速
			// if(if_start_line && start_line_count == 2) {
			// 	dc = dc / SLOW_DOWN_FACTOR;
			// }

			if(pw == 0)
			{
				tloss++;
				pw = pwt;		// use previous result
				if(tloss > 10)	// off-road protection 即小车离开轨道时，停止电机
				{
					dc = 0;
					pw = 1500;
					tloss = 0;
				}
			}
			else
			{
				tloss = 0;
			}
			
			// apply steering angle limits
			if(2000 < pw)
				pw = 2000;
			if(1000 > pw)
				pw = 1000;
			if(pwt != pw)
			{
				hsp_servo_angle(SERVO1, pw);
				hsp_servo_angle(SERVO2, pw);
				hsp_servo_angle(SERVO3, pw);
				hsp_servo_angle(SERVO4, pw);
				pwt = pw;
			}
			hsp_tft18_set_region(0, 0, TFT_X_MAX-1, TFT_Y_MAX-1);
			hsp_tft18_show_int16(0, 0, pw);
			hsp_tft18_show_int16_color(56, 0, encoder_speed, WHITE, BLACK);
			
			if(!SW1())
			{
				hsp_image2_show_dma(image2_use);
			}
			else
			{
				hsp_image2_show_dma(image2_temp);
			}
			image_ready = RESET;
		}
		
		if(!S3()) break;
	}
}


#define IMAGEW2 188  // Assuming the width of the image section being scanned

// typedef uint8_t image2_t[IMAGEW2][2];  // Simplified definition of image type

// Constants
#define CenterX 94
const int BasePulseWidth = 1500;
const double Kp = 10.0;  // Proportional gain
const double Kd = 5.0;   // Derivative gain

// Variables
int last_center = 94;
static uint8_t last_gte_ok = RESET;

	uint16_t hsp_image_judge(image2_t image) {
    uint16_t pw = 0;
    uint8_t gte_l, gte_r, gte_ok;
    uint8_t gte_l_idx, gte_r_idx, gte_c_idx;
    int current_center;

    gte_l = gte_r = gte_ok = RESET;
    for (int i = 2; i < IMAGEW2 - 2; i++) {
        if (!gte_l && image[20][i] == 255 && image[20][i+1] == 0) {
            gte_l = SET;
            gte_l_idx = i;
        } else if (gte_l && !gte_r && image[20][i] == 0 && image[20][i+1] == 255) {
            gte_r = SET;
            gte_r_idx = i;
        }
        if (gte_l && gte_r && !gte_ok && gte_r_idx > gte_l_idx + 6 && gte_r_idx < gte_l_idx + 30) {
            gte_ok = SET;
            gte_c_idx = (gte_r_idx + gte_l_idx) / 2;
        }
    }

    if (gte_ok) {
        current_center = gte_c_idx;
        int error = CenterX - current_center;
        int derivative = current_center - last_center;
        pw = BasePulseWidth + (int)(Kp * error + Kd * derivative);
        last_center = current_center;
        last_gte_ok = gte_ok;
    } else if (last_gte_ok) {
        // Use last good value if no line detected
        pw = BasePulseWidth + (int)(Kp * (CenterX - last_center));
    } else {
        pw = BasePulseWidth;  // Default to center position
    }

    return pw;
}
