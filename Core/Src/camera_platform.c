#include "camera_platform.h"

// 定义静态的PID限制参数
static const PID_Limit pid_h_limit = {
    .kp_limit_min = PROPORTIONAL_LIMIT_MIN_H,
    .kp_limit_max = PROPORTIONAL_LIMIT_MAX_H,
    .ki_limit_min = INTEGRAL_LIMIT_MIN_H,
    .ki_limit_max = INTEGRAL_LIMIT_MAX_H,
    .kd_limit_min = DERIVATIVE_LIMIT_MIN_H,
    .kd_limit_max = DERIVATIVE_LIMIT_MAX_H,
    .output_min = HORIZONTAL_MIN,
    .output_max = HORIZONTAL_MAX
};

static const PID_Limit pid_v_limit = {
    .kp_limit_min = PROPORTIONAL_LIMIT_MIN_V,
    .kp_limit_max = PROPORTIONAL_LIMIT_MAX_V,
    .ki_limit_min = INTEGRAL_LIMIT_MIN_V,
    .ki_limit_max = INTEGRAL_LIMIT_MAX_V,
    .kd_limit_min = DERIVATIVE_LIMIT_MIN_V,
    .kd_limit_max = DERIVATIVE_LIMIT_MAX_V,
    .output_min = VERTICAL_MIN,
    .output_max = VERTICAL_MAX
};

// PID controller instance
PID_Controller pid_horizontal = {
    .Kp = 25.0,
    .Ki = 30.0,
    .Kd = 20.0,
    .error = 0.0,
    .prev_error = 0.0,
    .integral = 0.0,
    .derivative = 0.0,
    .p_term = 0.0,
    .i_term = 0.0,
    .d_term = 0.0,
    .output = -60.0,
    .ramp_step = 2.0,
    .limit = &pid_h_limit
};

PID_Controller pid_vertical = {
    .Kp = 10.0,
    .Ki = 15.0,
    .Kd = 10.0,
    .error = 0.0,
    .prev_error = 0.0,
    .integral = 0.0,
    .derivative = 0.0,
    .p_term = 0.0,
    .i_term = 0.0,
    .d_term = 0.0,
    .output = -60.0,
    .ramp_step = 2.0,
    .limit = &pid_v_limit
};

// 增量式PID控制器实例
Inc_PID_Controller inc_pid_horizontal = {
    .Kp = 10.0,
    .Ki = 250.0,
    .Kd = 0.0,
    .error = 0.0,
    .prev_error = 0.0,
    .prev_prev_error = 0.0,
    .output = 0.0,
    .delta_output = 0.0,
    .ramp_step = 2.0,
    .limit = &pid_h_limit
};

Inc_PID_Controller inc_pid_vertical = {
    .Kp = 8.0,
    .Ki = 200.0,
    .Kd = 0.0,
    .error = 0.0,
    .prev_error = 0.0,
    .prev_prev_error = 0.0,
    .output = -60.0,
    .delta_output = 0.0,
    .ramp_step = 2.0,
    .limit = &pid_v_limit
};

// Global variables for error storage
float x_error = 0.0; // 水平误差
float y_error = 0.0; // 垂直误差

// 定义全局平台状态变量
Platform_Status platform_status = {
    .x_error = 0.0f,
    .y_error = 0.0f,
    .h_angle = 0.0f,
    .v_angle = -60.0f,
    .error_updated = false,
    .angle_updated = true,
    .target_detected = false,
    .target_locked = false,
    .cmd_received = false,
    .pid_mode = PID_MODE_INCREMENT
};

void set_platform(uint16_t h_pwm, uint16_t v_pwm) {
    // Ensure PWM values are within limits
    if (h_pwm < PWM_MIN) h_pwm = PWM_MIN;
    if (h_pwm > PWM_MAX) h_pwm = PWM_MAX;
    if (v_pwm < PWM_MIN) v_pwm = PWM_MIN;
    if (v_pwm > PWM_MAX) v_pwm = PWM_MAX;
    TIM4->CCR1 = h_pwm;
    TIM4->CCR2 = v_pwm;
}

// 设置舵机平台角度
void set_platform_angle(float h_angle, float v_angle) {
    // Convert horizontal angle to PWM
    uint16_t h_pwm = (uint16_t)(((h_angle - HORIZONTAL_MIN) / (HORIZONTAL_MAX - HORIZONTAL_MIN)) * (PWM_MAX - PWM_MIN) + PWM_MIN);
    
    // Convert vertical angle to PWM
    uint16_t v_pwm = (uint16_t)(((v_angle - VERTICAL_MIN) / (VERTICAL_MAX - VERTICAL_MIN)) * (PWM_MAX - PWM_MIN) + PWM_MIN);
    
    // Call the set_platform function with calculated PWM values
    set_platform(h_pwm, v_pwm);
}

void camera_platform_init(void) {
    // Initialize PID controllers
    // pid_controller_init(&pid_horizontal, 25.0f, 30.0f, 20.0f, 2.0f, &pid_h_limit);
    // pid_controller_init(&pid_vertical, 10.0f, 15.0f, 10.0f, 2.0f, &pid_v_limit);
    
    // 初始化增量式PID控制器
    // inc_pid_controller_init(&inc_pid_horizontal, 300.0f, 50.0f, 600.0f, 2.0f, &pid_h_limit);
    // inc_pid_controller_init(&inc_pid_vertical, 250.0f, 40.0f, 400.0f, 2.0f, &pid_v_limit);

    // Initialize the platform
    set_platform_angle(platform_status.h_angle, platform_status.v_angle);

    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

float limit_range(float integral, float min, float max) {
    if (integral < min) {
        return min;
    } else if (integral > max) {
        return max;
    }
    return integral;
}

// PID控制器初始化函数
void pid_controller_init(PID_Controller *pid, float kp, float ki, float kd, float ramp_step, const PID_Limit *limit) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->output = 0.0f;
    pid->ramp_step = ramp_step;
    pid->limit = limit;
}

// PID计算函数
float pid_calculate(PID_Controller *pid, float error) {
    float output;

    pid->error = error;
    
    // 计算P项并限制
    pid->p_term = pid->Kp * error;
    pid->p_term = limit_range(pid->p_term, pid->limit->kp_limit_min, pid->limit->kp_limit_max);

    // 计算I项并限制
    pid->integral = pid->Ki * error * PID_DT;
    pid->i_term += pid->integral;
    pid->i_term = limit_range(pid->i_term, pid->limit->ki_limit_min, pid->limit->ki_limit_max);

    // 计算D项并限制
    pid->derivative = (error - pid->prev_error) / PID_DT;
    pid->d_term = pid->Kd * pid->derivative;
    pid->d_term = limit_range(pid->d_term, pid->limit->kd_limit_min, pid->limit->kd_limit_max);

    // 计算总输出
    output = pid->p_term + pid->i_term + pid->d_term;
    
    // 限制输出范围
    output = limit_range(output, pid->limit->output_min, pid->limit->output_max);

    // 应用斜率限制
    float output_change = output - pid->output;
    if (output_change > pid->ramp_step) {
        output = pid->output + pid->ramp_step;
    } else if (output_change < -pid->ramp_step) {
        output = pid->output - pid->ramp_step;
    }

    // 更新状态
    pid->prev_error = error;
    pid->output = output;

    return output;
}

// 增量式PID控制器初始化
void inc_pid_controller_init(Inc_PID_Controller *pid, float kp, float ki, float kd, float ramp_step, const PID_Limit *limit) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
    pid->delta_output = 0.0f;
    pid->ramp_step = ramp_step;
    pid->limit = limit;
}

// 增量式PID计算函数
float inc_pid_calculate(Inc_PID_Controller *pid, float error) {
    // 保存误差
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = pid->error;
    pid->error = error;
    
    // 计算增量
    float delta_p = pid->Kp * (pid->error - pid->prev_error);
    float delta_i = pid->Ki * pid->error * PID_DT;
    float delta_d = pid->Kd * (pid->error - 2 * pid->prev_error + pid->prev_prev_error) / PID_DT;
    
    // 计算输出增量
    pid->delta_output = delta_p + delta_i + delta_d;
    
    // 限制增量变化
    pid->delta_output = limit_range(pid->delta_output, -pid->ramp_step, pid->ramp_step);
    
    // 更新输出
    pid->output += pid->delta_output;
    
    // 限制总输出范围
    pid->output = limit_range(pid->output, pid->limit->output_min, pid->limit->output_max);
    
    return pid->output;
}

// 更新平台位置函数，使用新的PID控制器
void update_platform_position(float x_error_input, float y_error_input) {
    // 检查是否接收到新指令
    if (!platform_status.cmd_received) {
        return;
    }

    // 检查是否锁定目标
    if (!platform_status.target_locked) {
        platform_status.cmd_received = false;  // 清除指令标志
        return;
    }

    float h_angle_output, v_angle_output;

    // 根据选择的模式使用不同的PID控制器
    if (platform_status.pid_mode == PID_MODE_INCREMENT) {
        h_angle_output = inc_pid_calculate(&inc_pid_horizontal, x_error_input);
        v_angle_output = inc_pid_calculate(&inc_pid_vertical, y_error_input);
    } else {
        h_angle_output = pid_calculate(&pid_horizontal, x_error_input);
        v_angle_output = pid_calculate(&pid_vertical, y_error_input);
    }

    // 设置平台位置
    set_platform_angle(h_angle_output, v_angle_output);

    // 清除指令标志
    platform_status.cmd_received = false;
}

// 实现设置水平误差的函数
void set_platform_x_error(float x_err) {
    platform_status.x_error = x_err;
    x_error = x_err;  // 保持与原来的全局变量兼容
}

// 实现设置垂直误差的函数
void set_platform_y_error(float y_err) {
    platform_status.y_error = y_err;
    y_error = y_err;  // 保持与原来的全局变量兼容
}

// 实现设置目标水平角度的函数
void set_platform_h_angle(float h_ang) {
    // 限制角度在有效范围内
    if (h_ang < HORIZONTAL_MIN) h_ang = HORIZONTAL_MIN;
    if (h_ang > HORIZONTAL_MAX) h_ang = HORIZONTAL_MAX;
    platform_status.h_angle = h_ang;
    set_platform_angle(h_ang, platform_status.v_angle);
}

// 实现设置目标垂直角度的函数
void set_platform_v_angle(float v_ang) {
    // 限制角度在有效范围内
    if (v_ang < VERTICAL_MIN) v_ang = VERTICAL_MIN;
    if (v_ang > VERTICAL_MAX) v_ang = VERTICAL_MAX;
    platform_status.v_angle = v_ang;
    set_platform_angle(platform_status.h_angle, v_ang);
} 


// 新增设置目标锁定状态的函数
void set_target_locked(bool detected) {
    // 如果保存的是false, 新传入的是true, 则设置locked
    if (!platform_status.target_detected && detected) {
        platform_status.target_locked = true;
    }
    if (platform_status.target_detected && !detected) {
        platform_status.target_locked = false;
    }
    // 保存当前状态
    platform_status.target_detected = detected;
}

void camera_10ms_task(void) {
    if (platform_status.target_detected && platform_status.error_updated && platform_status.cmd_received) {
        update_platform_position(platform_status.x_error, platform_status.y_error);
    }
    if (platform_status.angle_updated && platform_status.cmd_received) {
        set_platform_angle(platform_status.h_angle, platform_status.v_angle);
        platform_status.cmd_received = false;  // 清除指令标志
    }
}
