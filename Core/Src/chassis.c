#include "chassis.h"

/* 刹车缓降 */
static float brake_scale = 1.0f;

/* 电机占空比自适应低通滤波 */
#define DUTY_ALPHA_SLOW  0.25f    /* 小修正 (平滑) */
#define DUTY_ALPHA_FAST  0.80f    /* 大修正 (急弯快速响应) */
#define DUTY_GAP_THRESH  0.12f    /* 偏差阈值 */
static float duty_filt_L = 0.0f;
static float duty_filt_R = 0.0f;
static uint8_t filt_bypass = 0; /* 硬刹/停止时跳过滤波 */

/* --- TB6612 电机控制 (带刹车缩放 + 自适应低通滤波) --- */

static void tb6612_set_left(float duty)
{
    if (!filt_bypass) {
        float gap = duty - duty_filt_L;
        if (gap < 0) gap = -gap;
        float alpha = (gap > DUTY_GAP_THRESH) ? DUTY_ALPHA_FAST : DUTY_ALPHA_SLOW;
        duty_filt_L += alpha * (duty - duty_filt_L);
        duty = duty_filt_L;
    }
    duty *= brake_scale;
    uint16_t cmp = (uint16_t)(fabsf(duty) * CHASSIS_PWM_MAX);

    if (duty > 0.001f) {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
    } else if (duty < -0.001f) {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
    } else {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

static void tb6612_set_right(float duty)
{
    if (!filt_bypass) {
        float gap = duty - duty_filt_R;
        if (gap < 0) gap = -gap;
        float alpha = (gap > DUTY_GAP_THRESH) ? DUTY_ALPHA_FAST : DUTY_ALPHA_SLOW;
        duty_filt_R += alpha * (duty - duty_filt_R);
        duty = duty_filt_R;
    }
    duty *= brake_scale;
    uint16_t cmp = (uint16_t)(fabsf(duty) * CHASSIS_PWM_MAX);

    if (duty > 0.001f) {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp);
    } else if (duty < -0.001f) {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp);
    } else {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void Chassis_Init(void)
{
    /* STBY 拉高使能驱动芯片 */
    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

    /* 启动 PWM 输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* 初始停止 */
    Chassis_Stop();
}

void Chassis_SetMotor(uint8_t motor, float duty)
{
    if (duty > 1.0f)  duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    filt_bypass = 0;  /* 正常驱动启用滤波 */
    if (motor == CHASSIS_MOTOR_L)
        tb6612_set_left(duty);
    else
        tb6612_set_right(duty);
}

/* 绕过自适应滤波，直接输出 PWM (速度环专用) */
void Chassis_SetMotorRaw(uint8_t motor, float duty)
{
    if (duty > 1.0f)  duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    uint16_t cmp = (uint16_t)(fabsf(duty) * CHASSIS_PWM_MAX);

    if (motor == CHASSIS_MOTOR_L) {
        if (duty > 0.001f) {
            HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
        } else if (duty < -0.001f) {
            HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
        } else {
            HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        }
    } else {
        if (duty > 0.001f) {
            HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp);
        } else if (duty < -0.001f) {
            HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp);
        } else {
            HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        }
    }
}

void Chassis_Stop(void)
{
    brake_scale = 1.0f;
    filt_bypass = 1;  /* 立即停止，跳过滤波 */
    duty_filt_L = 0.0f;
    duty_filt_R = 0.0f;
    tb6612_set_left(0.0f);
    tb6612_set_right(0.0f);
}

void Chassis_BrakeHard(void)
{
    brake_scale = 1.0f;
    /* 左轮短路制动 */
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)CHASSIS_PWM_MAX);
    /* 右轮短路制动 */
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)CHASSIS_PWM_MAX);
}

void Chassis_SoftBrake(void)
{
    brake_scale = 1.0f;
}

uint8_t Chassis_BrakeUpdate(void)
{
    if (brake_scale > 0.0f) {
        brake_scale -= 0.04f;   /* 每 10ms 降 4%, 250ms 内降至 0 */
        if (brake_scale < 0.0f) brake_scale = 0.0f;
    }
    if (brake_scale <= 0.0f) {
        Chassis_Stop();
        return 1;  /* 刹车完成 */
    }
    return 0;
}

/* 读传感器 → 左右速度比 (公用于开环巡线和闭环巡线) */
void Chassis_LineRatio(float *rL, float *rR)
{
    uint8_t x1 = TRACK_X1_READ();
    uint8_t x2 = TRACK_X2_READ();
    uint8_t x3 = TRACK_X3_READ();
    uint8_t x4 = TRACK_X4_READ();

    if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0) {
        *rL = 0.0f;  *rR = 0.0f;
    } else if (x1 == 1 && (x2 == 0 || x3 == 0) && x4 == 1) {
        *rL = 1.0f;  *rR = 1.0f;
    } else if (x1 == 0 && x2 == 0 && x3 == 1 && x4 == 1) {
        *rL = 0.86f; *rR = 1.29f;
    } else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0) {
        *rL = 1.29f; *rR = 0.86f;
    } else if (x1 == 0) {
        *rL = 0.57f; *rR = 1.43f;
    } else if (x4 == 0) {
        *rL = 1.43f; *rR = 0.57f;
    } else {
        *rL = 1.0f;  *rR = 1.0f;
    }
}

/* --- 简单巡线算法 (移植自 HW_xunji) --- */

void Chassis_LineFollow(void)
{
    float rL, rR;
    Chassis_LineRatio(&rL, &rR);

    if (rL == 0.0f && rR == 0.0f) {
        Chassis_Stop();
    } else {
        Chassis_SetMotor(CHASSIS_MOTOR_L,  0.35f * rL);
        Chassis_SetMotor(CHASSIS_MOTOR_R,  0.35f * rR);
    }
}

/* --- 进阶巡线算法 (移植自 AI_xunji_moshi_pro) --- */

void Chassis_LineFollowAdv(void)
{
    static int trackState = 0;
    static int flag_F, forbid_F;
    static uint8_t forbid_turn = 0;

    uint8_t x1 = TRACK_X1_READ();
    uint8_t x2 = TRACK_X2_READ();
    uint8_t x3 = TRACK_X3_READ();
    uint8_t x4 = TRACK_X4_READ();

    /*
     * trackState:
     *   0 = 普通巡线模式
     *   1 = 左转中（持续左转直到回到 1001 状态）
     *   2 = 右转中（持续右转直到回到 1001 状态）
     * flag_F: S弯抑制标志
     * forbid_turn: 地图巡线 T 型路口防误转
     */

    switch (trackState) {
    case 0:
        /* 全白 / 十字路口 → 直行通过 */
        if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && flag_F != 1) {
            Chassis_SetMotor(CHASSIS_MOTOR_L,  0.20f);
            Chassis_SetMotor(CHASSIS_MOTOR_R,  0.20f);
        }
        /* X2、X3 在黑线上 → 直行 */
        if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1) {
            Chassis_SetMotor(CHASSIS_MOTOR_L,  0.20f);
            Chassis_SetMotor(CHASSIS_MOTOR_R,  0.20f);
            flag_F = 0;
            forbid_F = 0;
            if (forbid_turn == 1 || forbid_turn == 4)
                forbid_turn = 2;
        }
        /* 左偏出线 → 进入左转状态 */
        else if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 0) {
            Chassis_SetMotor(CHASSIS_MOTOR_L,  0.05f);
            Chassis_SetMotor(CHASSIS_MOTOR_R,  0.20f);
            trackState = 1;
        }
        /* 右偏出线 → 判断T型路口 */
        else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 1) {
            if (forbid_turn == 0) {
                Chassis_SetMotor(CHASSIS_MOTOR_L,  0.20f);
                Chassis_SetMotor(CHASSIS_MOTOR_R,  0.20f);
                forbid_turn = 1;
            } else if (forbid_turn == 3) {
                Chassis_SetMotor(CHASSIS_MOTOR_L,  0.20f);
                Chassis_SetMotor(CHASSIS_MOTOR_R,  0.20f);
                forbid_turn = 4;
            } else if (forbid_turn != 0 && forbid_turn != 1 && forbid_turn != 4) {
                Chassis_SetMotor(CHASSIS_MOTOR_L,  0.20f);
                Chassis_SetMotor(CHASSIS_MOTOR_R,  0.05f);
                trackState = 2;
            }
        }
        /* X4 检测到短线 → S弯抑制，强行左转 */
        else if (x4 == 0 && forbid_F != 1) {
            Chassis_SetMotor(CHASSIS_MOTOR_L,  0.10f);
            Chassis_SetMotor(CHASSIS_MOTOR_R,  1.00f);
            flag_F = 1;
        }
        /* X1 检测到黑线 → 左转大弯 */
        else if (x1 == 0) {
            Chassis_SetMotor(CHASSIS_MOTOR_L,  0.10f);
            Chassis_SetMotor(CHASSIS_MOTOR_R,  1.00f);
        }
        /* 中间偏左 → 微调左转 */
        else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 1) {
            Chassis_SetMotor(CHASSIS_MOTOR_L,  0.20f);
            Chassis_SetMotor(CHASSIS_MOTOR_R,  0.20f);
        }
        /* 中间偏右 → 微调右转 */
        else if (x1 == 1 && x2 == 0 && x3 == 1 && x4 == 1) {
            Chassis_SetMotor(CHASSIS_MOTOR_L,  0.20f);
            Chassis_SetMotor(CHASSIS_MOTOR_R,  0.20f);
        }
        break;

    case 1:
        /* 持续左转直到回正 */
        Chassis_SetMotor(CHASSIS_MOTOR_L,  0.08f);
        Chassis_SetMotor(CHASSIS_MOTOR_R,  0.58f);
        if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1) {
            trackState = 0;
            forbid_F = 1;
            HAL_Delay(200);
        }
        break;

    case 2:
        /* 持续右转直到回正 */
        Chassis_SetMotor(CHASSIS_MOTOR_L,  0.58f);
        Chassis_SetMotor(CHASSIS_MOTOR_R,  0.08f);
        if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1) {
            HAL_Delay(200);
            trackState = 0;
            forbid_turn = 3;
        }
        break;
    }
}
