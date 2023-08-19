/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-26     yuleitao       the first version
 */
#ifndef APPLICATIONS_ALGORITHM_SIMPLE_FILTER_SIMPLE_FILTER_H_
#define APPLICATIONS_ALGORITHM_SIMPLE_FILTER_SIMPLE_FILTER_H_

typedef struct _sloper
{
    float soft; // 幅值
    float cmd;
    float acc;
    float dec;
} sloper_t;

typedef struct _limiting_filter
{
    float amp; // 幅值
    float cur_value;
    float last_value;

} limiting_filter_t;

typedef struct _first_order_filter_type
{
    float input;
    float out;
    float num[1];
    float frame_period;
} first_order_filter_type_t;

void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
float sloper_update(sloper_t *sloper, float cmd);
void sloper_init(sloper_t *sloper, float acc, float dec);
void sloper_clear(sloper_t *sloper);

#endif /* APPLICATIONS_ALGORITHM_SIMPLE_FILTER_SIMPLE_FILTER_H_ */
