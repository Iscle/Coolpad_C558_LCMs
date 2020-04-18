/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "lcm_drv.h"
#include <linux/string.h>

#define REGFLAG_DELAY 0xFE
#define REGFLAG_END_OF_TABLE 0xFF

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v) lcm_util.set_reset_pin(v)

#define UDELAY(n) lcm_util.udelay(n)
#define MDELAY(n) lcm_util.mdelay(n)

#define dsi_set_cmdq_V2(cmd, count, para_list, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, para_list, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define dsi_write_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define dsi_write_regs(addr, para, nums) lcm_util.dsi_write_regs(addr, para, nums)
#define dsi_dcs_read_lcm_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[32];
};

#define MIN_ID_VOLTAGE 1400

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xF7, 4, {0xA9, 0x51, 0x2C, 0x82}},
	{0xC0, 2, {0x11, 0x09}},
	{0xC1, 1, {0x41}},
	{0xC5, 3, {0x00, 0x2D, 0x80}},
	{0xB1, 2, {0xB0, 0x11}},
	{0xB4, 1, {0x02}},
	{0xB6, 2, {0x02, 0x22}},
	{0xB7, 1, {0xC6}},
	{0xBE, 2, {0x00, 0x04}},
	{0xE9, 1, {0x00}},
	{0xF0, 3, {0x36, 0xA5, 0x53}},
	{0xE0, 12, {0x13, 0x36, 0x21, 0x00, 0x00, 0x00, 0x13, 0x36, 0x21, 0x00, 0x00, 0x00}},
	{0x36, 1, {0x08}},
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{REGFLAG_DELAY, 120, {}},
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for (i = 0; i < count; i++) {
		struct LCM_setting_table t = table[i];

		switch (t.cmd) {
		case REGFLAG_DELAY:
			MDELAY(t.count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(t.cmd, t.count, t.para_list, force_update);
			break;
		}
	}
}

static void lcm_set_util_funcs(LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->type = LCM_TYPE_DSI;
	params->dsi.packet_size = 256;
	params->width = 320;
	params->dsi.vertical_sync_active = 4;
	params->height = 480;
	params->dsi.vertical_backporch = 16;
	params->dsi.LANE_NUM = LCM_ONE_LANE;
	params->dsi.vertical_frontporch = 20;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.horizontal_sync_active = 10;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.horizontal_backporch = 80;
	params->dsi.horizontal_frontporch = 80;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_active_line = 480;
	params->dsi.horizontal_active_pixel = 320;
	params->dsi.PLL_CLOCK = 160;
}

static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(120);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	//push_table(lcm_deep_sleep_in_setting, sizeof(lcm_deep_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(120);
	SET_RESET_PIN(1);
	MDELAY(120);
}

static void lcm_resume(void)
{
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int data[1];
	unsigned char buf[2];

	data[0] = 0x23700;
	dsi_set_cmdq(data, sizeof(data) / sizeof(unsigned int), 1);
	dsi_dcs_read_lcm_reg_v2(0x04, buf, sizeof(buf) / sizeof(unsigned char));

	if (buf[0] == 0x54 && buf[1] == 0x80) {
		return 1;
	}

	return 0;
}

LCM_DRIVER rm68140_hvga_qf3902_prj_c558_ivo_drv =
{
	.name = "rm68140_hvga_qf3902_prj_c558_ivo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
};
