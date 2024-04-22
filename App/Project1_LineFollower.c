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

uint16_t line_detection_count = 0;
bool if_start_line = false; // 是否检测到起点线
static bool isReversing = false; // 是否正在倒退
static bool needToStop = false;  // 是否需要停车


// Function prototypes
void perform_action_based_on_line_type();
void beeper_beep(uint32_t duration_ms);
void beep();
uint16_t calculate_speed(uint16_t steering_signal);
uint16_t detect_black_points(image2_t image, uint8_t row);
void check_and_perform_stop();




#include <math.h>  // 包含数学库以使用exp函数


uint16_t calculate_speed(uint16_t steering_signal) {
    
	int abs_value=abs(steering_signal-1500);
	uint16_t speed;
	if(abs_value < 200) {
		speed = 30;
	}
	// else if(abs_value < 100) {
	// 	speed = 20;
	// }
	else {
		speed = 25;
	}
	hsp_tft18_show_int16(0, 20, speed);
	return speed;

}




// #include <math.h>  // 包含数学库以使用exp函数

// #define V_MAX 30   // 最大速度
// #define K 0.15     // 适度调整衰减系数，使减速不过激
// #define MAX_DEVIATION 500 // 最大偏差
// #define EARLY_REDUCTION_THRESHOLD 100  // 适中的轻微调整速度阈值
// #define CRITICAL_REDUCTION_THRESHOLD 250  // 合适的强制大幅度降速阈值
// #define MIN_SPEED 16  // 最小速度设置为16

// uint16_t calculate_speed(uint16_t steering_signal, uint16_t last_steering_signal) {
    // int deviation = abs(1500 - steering_signal);
    // int last_deviation = abs(1500 - last_steering_signal);

    // // 根据舵机信号变化趋势预测弯道
    // double prediction_factor = 1.0;
    // if (deviation > last_deviation) {
    //     prediction_factor = 0.8; // 预测弯道，适度提前降速
    // }

    // double reduction_factor = 1.0;
    // if (deviation > EARLY_REDUCTION_THRESHOLD) {
    //     reduction_factor = 0.77; // 轻微调整速度为80%
    // }
    // if (deviation > CRITICAL_REDUCTION_THRESHOLD) {
    //     reduction_factor = 0.7; // 强制大幅度降速为60%
    // }

    // // 使用指数衰减函数来计算速度
    // double exponential_factor = exp(-K * pow((double)deviation / MAX_DEVIATION, 2));
    // uint16_t speed = (uint16_t)(V_MAX * prediction_factor * reduction_factor * exponential_factor);

    // // 设置一个最小速度阈值以防车辆完全停止
    // if (speed < MIN_SPEED) speed = MIN_SPEED;

//     return speed;
// }


extern volatile uint32_t sys_tick_counter;

// uint16_t last_pw = 1500; // 初始为中心值 // 在主循环中更新调用 

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
		if(image_ready == SET)
		{
			
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
			// dc = calculate_speed(pw);
			
			dc = calculate_speed(pw); // 根据当前和上一次的舵机信号调整速度 
			// last_pw = pw; // 更新上一次的舵机信号为当前值 
			// hsp_motor_voltage(MOTORF, dc); // 应用新的速度设置

			perform_action_based_on_line_type();


			check_and_perform_stop();

        	if (needToStop) {
                hsp_motor_voltage(MOTORF, 0); // 停止电机
                needToStop = false;
                isReversing = true;
				 // 设置倒退标志
				 delay_1ms(1000);
            } ;
			
			if (isReversing) {
                hsp_motor_voltage(MOTORB, -20); // 倒退电机速度
				delay_1ms(1000);
				hsp_motor_voltage(MOTORF, 0);
				exit(0) ;
            } ;

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
const double Kp = 20.0;  // Proportional gain
const double Ki = 0;   // Integral gain
const double Kd = 5.0;   // Derivative gain
#define BLACKPOINTS_THRESHOLD 60
// Variables
int last_center = 94;
static uint8_t last_gte_ok = RESET;
static int integral_error = 0; // Initialize integral error

uint16_t line_type = 0; // 0表示中线，1表示起始线，2表示十字线


uint16_t hsp_image_judge(image2_t image) {
    uint16_t pw = 0;
    uint8_t gte_l, gte_r, gte_ok;
    uint8_t gte_l_idx, gte_r_idx, gte_c_idx;
    int current_center;

	uint16_t black_points1 = detect_black_points(image, 20);  // 检测第20行的黑点数
	uint16_t black_points2 = detect_black_points(image, 26);  // 检测第26行的黑点数
	uint16_t black_points3 = detect_black_points(image, 14);  // 检测第14行的黑点数


	bool is_black_ok = black_points1 > BLACKPOINTS_THRESHOLD || black_points2 > BLACKPOINTS_THRESHOLD || black_points3 > BLACKPOINTS_THRESHOLD;

    gte_l = gte_r = gte_ok = RESET;
	
	// 从第20行开始检测黑线
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
		// 通过检测到的线的位置计算舵机PWM
        current_center = gte_c_idx;
        int error = CenterX - current_center;
        int derivative = current_center - last_center;

		integral_error += error;

        pw = BasePulseWidth + Kp * error + Ki * integral_error + Kd * derivative;
        last_center = current_center;
        last_gte_ok = gte_ok;

		// 根据黑点数判断线的类型
		if(is_black_ok) //非中线,采用奇偶判断具体是起始线还是十字线
		{
			line_detection_count++;
			if(line_detection_count % 2 == 1) // 起始线
			{
				line_type = 1;
			}
			else // 十字线
			{
				line_type = 2;
			}
		}
		else // 中线
		{
			line_type = 0;
		}


    } else if (last_gte_ok) {
        // Use last good value if no line detected
		// line_type = 0; // 中线
        pw = BasePulseWidth + (int)(Kp * (CenterX - last_center)) + Ki * integral_error;
    } else {
        pw = BasePulseWidth;  // Default to center position
    }

    return pw;
}





uint16_t detect_black_points(image2_t image, uint8_t row) {
    uint16_t black_points = 0;

    // 统计行中黑色点的数量
    for (int i = 0; i < IMAGEW2; i++) {
        if (image[row][i] == 0) {
            black_points++;
        }
    }
    
    return black_points;
}

void beeper_beep(uint32_t duration_ms) {
    // 控制蜂鸣器鸣叫指定的毫秒数
	BUZZ_ON();
	delay_1ms(duration_ms);
	BUZZ_OFF();

}

void beep() {
	// 控制蜂鸣器鸣叫100毫秒
	beeper_beep(100);
}


void perform_action_based_on_line_type() {
	if (line_type == 1) {
		// 如果检测到起始线，beep一声
		beep();

		
	} else if (line_type == 2) {
		// 如果检测到十字线，beep两声
		beep();
		delay_1ms(100);
		beep();
	}
	//将line_detection_count显示在数码管上
	hsp_cat9555_seg7_decimal(line_detection_count);

}


void check_and_perform_stop() {
    
	if (line_detection_count == 5) {
		needToStop = true; // 设置停车标志
	}
    
}
