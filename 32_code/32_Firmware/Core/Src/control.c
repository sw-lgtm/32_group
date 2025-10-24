#include "control.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include <math.h>

/* ====================================================================== */
/* ==============          运动控制与PID参数定义         ============== */
/* ====================================================================== */

// 通用运动参数
#define TURN_SPEED 0.1f
#define PWM_MAX 999
#define PWM_MIN -999

// --- 速度环 (内环) PID参数 ---
#define L_KP 50.0f  // 左轮 Kp
#define L_KI 310.0f // 左轮 Ki
#define L_KD 0.5f   // 左轮 Kd

#define R_KP 50.0f  // 右轮 Kp 
#define R_KI 285.0f // 右轮 Ki
#define R_KD 0.5f   // 右轮 Kd

// --- 航向环 (外环) PID参数 ---
#define A_KP 0.0f   // 航向角 Kp (核心参数) - 置零以禁用
#define A_KI 0.0f   // 航向角 Ki (消除静差) - 置零以禁用
#define A_KD 0.0f   // 航向角 Kd (抑制震荡) - 置零以禁用
// 航向环输出的速度修正值上限，防止小车剧烈摇摆
#define ANGLE_CORRECTION_MAX 0.2f 


/* ====================================================================== */
/* ==============            内部状态变量定义              ============== */
/* ====================================================================== */

static PID_Controller pid_left;
static PID_Controller pid_right;
static PID_Controller pid_angle;

static Control_State_t current_state = CONTROL_STATE_IDLE;
static int last_pwm_output_L = 0;
static int last_pwm_output_R = 0;

static float base_speed_mps = 0.0f;
static float target_heading_rad = 0.0f;

static bool is_turning = false;
static float turn_target_mileage = 0.0f;
static float turn_start_displacement_L = 0.0f;
// static float turn_start_displacement_R = 0.0f; // 【修正】删除此行，因为它未被使用


void Control_Init(void)
{
    // 初始化内环的速度PID控制器 (使用独立的参数)
    PID_Init(&pid_left,  L_KP, L_KI, L_KD, PWM_MIN, PWM_MAX);
    PID_Init(&pid_right, R_KP, R_KI, R_KD, PWM_MIN, PWM_MAX);
    
    // 初始化外环的航向角PID控制器
    PID_Init(&pid_angle, A_KP, A_KI, A_KD, -ANGLE_CORRECTION_MAX, ANGLE_CORRECTION_MAX);

    Control_Stop();
}

void Control_Update_10ms(void)
{
    // --- 状态1: 航向角闭环直行 ---
    if (current_state == CONTROL_STATE_PID_STRAIGHT)
    {
        float angle_error = target_heading_rad - robot_theta;
        if (angle_error > M_PI) angle_error -= 2.0f * M_PI;
        if (angle_error < -M_PI) angle_error += 2.0f * M_PI;

        pid_angle.setpoint = 0.0f;
        float speed_correction = PID_Calculate(&pid_angle, angle_error, 0.01f);
        
        pid_left.setpoint = base_speed_mps - speed_correction;
        pid_right.setpoint = base_speed_mps + speed_correction;
    }
    // --- 状态2: 普通PID模式 (用于转弯) ---
    else if (current_state == CONTROL_STATE_PID)
    {
        if (is_turning)
        {
            float disp_traveled = fabsf(left_displacement_m - turn_start_displacement_L);
            if (disp_traveled >= turn_target_mileage)
            {
                Control_Stop();
                return;
            }
        }
    }
    else { return; }

    // --- 内环PID计算 (对所有PID状态都生效) ---
    int pwm_out_left = (int)PID_Calculate(&pid_left, left_speed_signed_mps, 0.01f);
    int pwm_out_right = (int)PID_Calculate(&pid_right, right_speed_signed_mps, 0.01f);

    last_pwm_output_L = pwm_out_left;
    last_pwm_output_R = pwm_out_right;
    
    Load(pwm_out_left, pwm_out_right);
}

void Control_Set_TargetSpeed(float target_left, float target_right)
{
    is_turning = false;
    
    if (target_left == target_right && target_left != 0.0f) 
    {
        current_state = CONTROL_STATE_PID_STRAIGHT;
        base_speed_mps = target_left;
        
        float current_angle_deg = robot_theta * 180.0f / M_PI;
        float target_angle_deg = roundf(current_angle_deg / 90.0f) * 90.0f;
        target_heading_rad = target_angle_deg * M_PI / 180.0f;
    }
    else
    {
        current_state = CONTROL_STATE_PID;
        pid_left.setpoint = target_left;
        pid_right.setpoint = target_right;
    }
}

void Control_Turn_By_Angle(float angle_deg)
{
    if (current_state == CONTROL_STATE_OPEN_LOOP) return;

    turn_target_mileage = fabsf((angle_deg / 360.0f) * M_PI * WHEELBASE);
    turn_start_displacement_L = left_displacement_m;

    if (angle_deg > 0) {
        Control_Set_TargetSpeed(TURN_SPEED, -TURN_SPEED);
    } else {
        Control_Set_TargetSpeed(-TURN_SPEED, TURN_SPEED);
    }
    
    is_turning = true;
}

void Control_Set_OpenLoop(int pwm_left, int pwm_right)
{
    is_turning = false;
    current_state = CONTROL_STATE_OPEN_LOOP;
    last_pwm_output_L = pwm_left;
    last_pwm_output_R = pwm_right;
    Load(pwm_left, pwm_right);
}

void Control_Stop(void)
{
    is_turning = false;
    current_state = CONTROL_STATE_IDLE;
    base_speed_mps = 0.0f;
    last_pwm_output_L = 0;
    last_pwm_output_R = 0;
    
    PID_Init(&pid_left,  L_KP, L_KI, L_KD, PWM_MIN, PWM_MAX);
    PID_Init(&pid_right, R_KP, R_KI, R_KD, PWM_MIN, PWM_MAX);
    PID_Init(&pid_angle, A_KP, A_KI, A_KD, -ANGLE_CORRECTION_MAX, ANGLE_CORRECTION_MAX);
    
    Load(0, 0);
}

/* --- Getter Functions --- */
float Control_Get_TargetSpeed_L(void) { return (current_state == CONTROL_STATE_PID || current_state == CONTROL_STATE_PID_STRAIGHT) ? pid_left.setpoint : 0.0f; }
float Control_Get_TargetSpeed_R(void) { return (current_state == CONTROL_STATE_PID || current_state == CONTROL_STATE_PID_STRAIGHT) ? pid_right.setpoint : 0.0f; }
int Control_Get_LastPwm_L(void) { return last_pwm_output_L; }
int Control_Get_LastPwm_R(void) { return last_pwm_output_R; }