#include "serialcmd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "main.h"
#include "speed_ctrl.h"
#include "balance.h"

static char buf[64];
static uint8_t buf_idx = 0;
static volatile uint8_t cmd_ready = 0;

volatile uint8_t  g_task_mode   = 2;
volatile float    g_ball_target = 0.0f;
volatile uint8_t  g_key_pressed = 0;
volatile uint32_t g_lap_time_ms = 0;

void SerialCmd_RxByte(uint8_t byte)
{
    if (byte == '\r') return;
    if (byte == '\n') {
        buf[buf_idx] = '\0';
        buf_idx = 0;
        cmd_ready = 1;
    } else if (buf_idx < 63) {
        buf[buf_idx++] = byte;
    }
}

static void spd_pid_cmd(const char *k, float v, uint8_t rel)
{
    PID_Controller *pL = Spd_GetPidL();
    PID_Controller *pR = Spd_GetPidR();

    if (strcmp(k, "bff") == 0) {
        float n = rel ? Balance_GetFFGain() + v : v;
        if (n < 0.0f) n = 0.0f;
        Balance_SetFFGain(n);
        printf("BFF:%.1f\r\n", (double)n);
        return;
    }
    if (strcmp(k, "bkp") == 0) {
        float n = rel ? Balance_GetKp() + v : v;
        if (n < 0.0f) n = 0.0f;
        Balance_SetGains(n, Balance_GetKi(), Balance_GetKd());
        printf("BKP:%.3f\r\n", (double)n);
        return;
    }
    if (strcmp(k, "bki") == 0) {
        float n = rel ? Balance_GetKi() + v : v;
        if (n < 0.0f) n = 0.0f;
        Balance_SetGains(Balance_GetKp(), n, Balance_GetKd());
        printf("BKI:%.3f\r\n", (double)n);
        return;
    }
    if (strcmp(k, "bkd") == 0) {
        float n = rel ? Balance_GetKd() + v : v;
        if (n < 0.0f) n = 0.0f;
        Balance_SetGains(Balance_GetKp(), Balance_GetKi(), n);
        printf("BKD:%.3f\r\n", (double)n);
        return;
    }
    if (strcmp(k, "bkv") == 0) {
        float n = rel ? Balance_GetKv() + v : v;
        if (n < 0.0f) n = 0.0f;
        Balance_SetKv(n);
        printf("BKV:%.3f\r\n", (double)n);
        return;
    }
    if (strcmp(k, "skp") == 0) {
        float n = rel ? pL->kp + v : v;
        if (n < 0.0f) n = 0.0f;
        PID_SetGains(pL, n, pL->ki, pL->kd);
        PID_SetGains(pR, n, pR->ki, pR->kd);
        printf("SKP:%.3f\r\n", (double)n);
    } else if (strcmp(k, "ski") == 0) {
        float n = rel ? pL->ki + v : v;
        if (n < 0.0f) n = 0.0f;
        PID_SetGains(pL, pL->kp, n, pL->kd);
        PID_SetGains(pR, pR->kp, n, pR->kd);
        printf("SKI:%.3f\r\n", (double)n);
    } else if (strcmp(k, "skd") == 0) {
        float n = rel ? pL->kd + v : v;
        if (n < 0.0f) n = 0.0f;
        PID_SetGains(pL, pL->kp, pL->ki, n);
        PID_SetGains(pR, pR->kp, pR->ki, n);
        printf("SKD:%.3f\r\n", (double)n);
    } else if (strcmp(k, "lskp") == 0) {
	        float n = rel ? pL->kp + v : v;
	        if (n < 0.0f) n = 0.0f;
	        PID_SetGains(pL, n, pL->ki, pL->kd);
	        printf("LSKP:%.3f\r\n", (double)n);
	    } else if (strcmp(k, "rskp") == 0) {
	        float n = rel ? pR->kp + v : v;
	        if (n < 0.0f) n = 0.0f;
	        PID_SetGains(pR, n, pR->ki, pR->kd);
	        printf("RSKP:%.3f\r\n", (double)n);
	    } else if (strcmp(k, "lski") == 0) {
	        float n = rel ? pL->ki + v : v;
	        if (n < 0.0f) n = 0.0f;
	        PID_SetGains(pL, pL->kp, n, pL->kd);
	        printf("LSKI:%.3f\r\n", (double)n);
	    } else if (strcmp(k, "rski") == 0) {
	        float n = rel ? pR->ki + v : v;
	        if (n < 0.0f) n = 0.0f;
	        PID_SetGains(pR, pR->kp, n, pR->kd);
	        printf("RSKI:%.3f\r\n", (double)n);
	    } else if (strcmp(k, "lskd") == 0) {
	        float n = rel ? pL->kd + v : v;
	        if (n < 0.0f) n = 0.0f;
	        PID_SetGains(pL, pL->kp, pL->ki, n);
	        printf("LSKD:%.3f\r\n", (double)n);
	    } else if (strcmp(k, "rskd") == 0) {
	        float n = rel ? pR->kd + v : v;
	        if (n < 0.0f) n = 0.0f;
	        PID_SetGains(pR, pR->kp, pR->ki, n);
	        printf("RSKD:%.3f\r\n", (double)n);
	    } else if (strcmp(k, "pkp") == 0) {
        float n = rel ? Spd_GetPosKp() + v : v;
        if (n < 0.0f) n = 0.0f;
        Spd_SetPosKp(n);
        printf("PKP:%.5f\r\n", (double)n);
    }
}

static void analyze_cmd(char *cmd, Gimbal_Axis *g, PID_Controller *vp)
{
    char *rel = strpbrk(cmd, "+-");
    if (rel && rel != cmd) {
        char op = *rel;
        *rel = '\0';
        float val = (float)atof(rel + 1);
        if (op == '-') val = -val;
        spd_pid_cmd(cmd, val, 1);
        return;
    }

    char *colon = strchr(cmd, ':');
    if (!colon) return;

    *colon = '\0';
    char *key = cmd;
    float val = (float)atof(colon + 1);

    if (strcmp(key, "mode") == 0) {
        if (val >= 2 && val <= 6) {
            g_task_mode = (uint8_t)val;
            printf("Task mode: %d\r\n", g_task_mode);
        }
    } else if (strcmp(key, "target") == 0) {
        g_ball_target = val;
        printf("Ball target: %.1f cm\r\n", (double)val);
    } else if (strcmp(key, "kp") == 0 && vp) {
        PID_SetGains(vp, val, vp->ki, vp->kd);
        printf("OK! %s==%.3f\r\n", key, (double)val);
    } else if (strcmp(key, "ki") == 0 && vp) {
        PID_SetGains(vp, vp->kp, val, vp->kd);
        printf("OK! %s==%.3f\r\n", key, (double)val);
    } else if (strcmp(key, "kd") == 0 && vp) {
        PID_SetGains(vp, vp->kp, vp->ki, val);
        printf("OK! %s==%.3f\r\n", key, (double)val);
    } else if (strcmp(key, "enable") == 0 && g) {
        Gimbal_Enable(g, (uint8_t)val);
        printf("OK! %s==%.0f\r\n", key, (double)val);
    } else if (strcmp(key, "dir") == 0) {
        Spd_Direct((uint8_t)val);
        printf("DirectMode:%d\r\n", (int)(uint8_t)val);
    } else if (strcmp(key, "bsmode") == 0) {
        Balance_SetSpeedMode((uint8_t)val);
        printf("SpeedMode:%d\r\n", (int)(uint8_t)val);
    } else {
        spd_pid_cmd(key, val, 0);
        return;
    }
}

void SerialCmd_Init(void)
{
    HAL_NVIC_SetPriority(UART4_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}

void SerialCmd_Update(Gimbal_Axis *g, PID_Controller *vp)
{
    if (cmd_ready) {
        cmd_ready = 0;
        analyze_cmd(buf, g, vp);
    }
}
