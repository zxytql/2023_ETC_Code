/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-26     yuleitao       the first version
 */

#include <simple_filter.h>
#include <math.h>

// 限幅滤波
float limiting_filter(limiting_filter_t *fliter, float value)
{
    fliter->cur_value = value;
    if (((fliter->cur_value - fliter->last_value) > fliter->amp) || //
            ((fliter->last_value - fliter->cur_value) > fliter->amp))
        return fliter->last_value;
    else
        return fliter->cur_value;
}

/**
 * @brief          一阶低通滤波初始化
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
 * @brief          一阶低通滤波计算
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out = first_order_filter_type->num[0] / //
            (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * //
            first_order_filter_type->out + first_order_filter_type->frame_period / //
            (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * //
            first_order_filter_type->input; //
}


float sloper_update(sloper_t *sloper, float cmd)
{
    sloper->cmd = cmd;
    if (fabs(sloper->cmd) > (fabs(sloper->soft) + 0.01f))
    {
        if (sloper->cmd > 0.0f && sloper->soft > 0.0f)
            sloper->soft += sloper->acc;
        else if (sloper->cmd < 0.0f && sloper->soft < 0.0f)
            sloper->soft -= sloper->acc;
        else if (sloper->cmd > 0.0f && sloper->soft < 0.0f)
            sloper->soft += sloper->dec;
        else if (sloper->cmd < 0.0f && sloper->soft > 0.0f)
            sloper->soft -= sloper->dec;
        else
            sloper->soft = sloper->cmd;
    }
    else if (fabs(sloper->cmd) < (fabs(sloper->soft) - 0.01f))
    {
        if (sloper->cmd > 0.0f && sloper->soft > 0.0f)
            sloper->soft -= sloper->dec;
        else if (sloper->cmd < 0.0f && sloper->soft < 0.0f)
            sloper->soft += sloper->dec;
        else if (sloper->cmd > 0.0f && sloper->soft < 0.0f)
            sloper->soft += sloper->dec;
        else if (sloper->cmd < 0.0f && sloper->soft > 0.0f)
            sloper->soft -= sloper->dec;
        else
            sloper->soft = sloper->cmd;
    }
    else
    {
        sloper->soft = sloper->cmd;
    }

    return sloper->soft;
}

void sloper_init(sloper_t *sloper, float acc, float dec)
{
    sloper->acc = acc;
    sloper->dec = dec;
    sloper->soft = 0.0f;
    sloper->cmd = 0.0f;
}

void sloper_clear(sloper_t *sloper)
{
    sloper->soft = 0.0f;
    sloper->cmd = 0.0f;
}
