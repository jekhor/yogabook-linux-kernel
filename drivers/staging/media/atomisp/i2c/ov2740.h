// SPDX-License-Identifier: GPL-2.0
/*
 * Support for  OV2740 camera sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 */

#ifndef __OV2740_H__
#define __OV2740_H__
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "../include/linux/atomisp_platform.h"

#define V4L2_IDENT_OV2740 2740

/* Defines for register writes and register array processing */
#define OV2740_BYTE_MAX	32
#define OV2740_SHORT_MAX	16
#define I2C_MSG_LENGTH		0x2
#define I2C_RETRY_COUNT		5

#define OV2740_READ_MODE	0x3820
#define OV2740_TEST_PATTERN_MODE			0x0601

#define OV2740_HFLIP_BIT	0x1
#define OV2740_VFLIP_BIT	0x2
#define OV2740_VFLIP_OFFSET	1
#define OV2740_IMG_ORIENTATION	0x0101

#define I2C_RETRY_COUNT		5
#define MAX_FMTS		1

#define OV2740_PID_LOW		0x300c
#define OV2740_PID_HIGH		0x300b
#define OV2740_REV		0x2
#define OV2740_MOD_ID		0x2740

#define OV2740_RES_WIDTH_MAX	1932
#define OV2740_RES_HEIGHT_MAX	1092

#define ISP_PADDING_W 16
#define ISP_PADDING_H 16
#define OV2740_ISP_MAX_WIDTH	(OV2740_RES_WIDTH_MAX - ISP_PADDING_W)
#define OV2740_ISP_MAX_HEIGHT	(OV2740_RES_HEIGHT_MAX - ISP_PADDING_H)

#define OV2740_MAX_GAIN_VALUE 0x7C0

#define OV2740_FOCAL_LENGTH_NUM	235	/*2.35mm*/
#define OV2740_FOCAL_LENGTH_DEM	100
#define OV2740_F_NUMBER_DEFAULT_NUM	240
#define OV2740_F_NUMBER_DEM	100

#define OV2740_COARSE_INTEGRATION_TIME_H		0x3500
#define OV2740_COARSE_INTEGRATION_TIME_M		0x3501
#define OV2740_COARSE_INTEGRATION_TIME_L		0x3502
#define OV2740_GLOBAL_GAIN			0x3508

#define OV2740_INTG_BUF_COUNT		2

#define OV2740_VT_PIX_CLK_DIV			0x30b0
#define OV2740_VT_SYS_CLK_DIV			0x30b1
#define OV2740_PRE_PLL_CLK_DIV			0x0304
#define OV2740_PLL_MULTIPLIER_H			0x30b2
#define OV2740_PLL_MULTIPLIER_L 		0x30b3
#define OV2740_PLL_PLL1_PRE_DIV			0x30b4
#define OV2740_MIPI_DIV					0x3010

#define OV2740_OP_PIX_DIV			0x0300
#define OV2740_OP_SYS_DIV			0x0302
#define OV2740_FRAME_LENGTH_LINES		0x380e
#define OV2740_COARSE_INTG_TIME_MIN		0x1004
#define OV2740_FINE_INTG_TIME_MIN		0x1008

#define OV2740_BIN_FACTOR_MAX			1
#define OV2740_MCLK		192

#define OV2740_HORIZONTAL_START_H 0x3800
#define OV2740_VERTICAL_START_H 0x3802
#define OV2740_HORIZONTAL_END_H 0x3804
#define OV2740_VERTICAL_END_H 0x3806
#define OV2740_HORIZONTAL_OUTPUT_SIZE_H 0x3808
#define OV2740_VERTICAL_OUTPUT_SIZE_H 0x380A

#define OV2740_SW_STREAM			0x0100
#define OV2740_START_STREAMING			0x01
#define OV2740_STOP_STREAMING			0x00

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV2740_FOCAL_LENGTH_DEFAULT 0xA60064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV2740_F_NUMBER_DEFAULT 0x1200064

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define OV2740_F_NUMBER_RANGE 0x1D0a1D0a

#define	v4l2_format_capture_type_entry(_width, _height, \
		_pixelformat, _bytesperline, _colorspace) \
	{\
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,\
		.fmt.pix.width = (_width),\
		.fmt.pix.height = (_height),\
		.fmt.pix.pixelformat = (_pixelformat),\
		.fmt.pix.bytesperline = (_bytesperline),\
		.fmt.pix.colorspace = (_colorspace),\
		.fmt.pix.sizeimage = (_height)*(_bytesperline),\
	}

#define	s_output_format_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps) \
	{\
		.v4l2_fmt = v4l2_format_capture_type_entry(_width, \
			_height, _pixelformat, _bytesperline, \
				_colorspace),\
		.fps = (_fps),\
	}

#define	s_output_format_reg_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps, _reg_setting) \
	{\
		.s_fmt = s_output_format_entry(_width, _height,\
				_pixelformat, _bytesperline, \
				_colorspace, _fps),\
		.reg_setting = (_reg_setting),\
	}

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

#define	v4l2_queryctrl_entry_integer(_id, _name,\
		_minimum, _maximum, _step, \
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_INTEGER, \
		.name = _name, \
		.minimum = (_minimum), \
		.maximum = (_maximum), \
		.step = (_step), \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}
#define	v4l2_queryctrl_entry_boolean(_id, _name,\
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_BOOLEAN, \
		.name = _name, \
		.minimum = 0, \
		.maximum = 1, \
		.step = 1, \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}

#define	s_ctrl_id_entry_integer(_id, _name, \
		_minimum, _maximum, _step, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_integer(_id, _name,\
				_minimum, _maximum, _step,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

#define	s_ctrl_id_entry_boolean(_id, _name, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_boolean(_id, _name,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

enum ov2740_tok_type {
	OV2740_8BIT  = 0x0001,
	OV2740_16BIT = 0x0002,
	OV2740_RMW   = 0x0010,
	OV2740_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	OV2740_TOK_DELAY  = 0xfe00, /* delay token for reg list */
	OV2740_TOK_MASK = 0xfff0
};

/*
 * If register address or register width is not 32 bit width,
 * user needs to convert it manually
 */

struct s_register_setting {
	u32 reg;
	u32 val;
};

struct s_output_format {
	struct v4l2_format v4l2_fmt;
	int fps;
};

/**
 * struct ov2740_fwreg - Fisare burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @val: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct ov2740_fwreg {
	enum ov2740_tok_type type; /* value, register or FW burst string */
	u16 addr;	/* target address */
	u32 val[8];
};

/**
 * struct ov2740_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct ov2740_reg {
	enum ov2740_tok_type type;
	union {
		u16 sreg;
		struct ov2740_fwreg *fwreg;
	} reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

#define to_ov2740_sensor(x) container_of(x, struct ov2740_device, sd)

#define OV2740_MAX_WRITE_BUF_SIZE	30
struct ov2740_write_buffer {
	u16 addr;
	u8 data[OV2740_MAX_WRITE_BUF_SIZE];
};

struct ov2740_write_ctrl {
	int index;
	struct ov2740_write_buffer buffer;
};


struct ov2740_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct camera_sensor_platform_data *platform_data;
	struct mutex input_lock; /* serialize sensor's ioctl */
	struct ov2740_resolution *res;
	bool power_on;
	//u8 *otp_data;
	int status;
	int streaming;
	int power;
	int run_mode;
	int vt_pix_clk_freq_mhz;
	u16 sensor_id;
	u16 coarse_itg;
	u16 fine_itg;
	u16 gain;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	u8 type;
	u8 sensor_revision;

	struct v4l2_ctrl_handler ctrl_handler;
};

struct ov2740_format_struct {
	u8 *desc;
	struct regval_list *regs;
	u32 pixelformat;
};

struct ov2740_resolution {
	u8 *desc;
	const struct ov2740_reg *regs;
	int res;
	int width;
	int height;
	int fps;
	int pix_clk_freq;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	u8 bin_factor_x;
	u8 bin_factor_y;
	bool used;
	u32 skip_frames;
};

struct ov2740_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/* init settings */
static struct ov2740_reg const ov2740_init_config[] = {
    {OV2740_8BIT, {0x0103}, 0x01},
    {OV2740_TOK_DELAY, {0}, 0x0a},
    {OV2740_8BIT, {0x0302}, 0x1e},
    {OV2740_8BIT, {0x030d}, 0x1e},
    {OV2740_8BIT, {0x030e}, 0x02},
    {OV2740_8BIT, {0x0312}, 0x01},
    {OV2740_8BIT, {0x3000}, 0x00},
    {OV2740_8BIT, {0x3018}, 0x32},
    {OV2740_8BIT, {0x3031}, 0x0a},
    {OV2740_8BIT, {0x3080}, 0x08},
    {OV2740_8BIT, {0x3083}, 0xB4},
    {OV2740_8BIT, {0x3103}, 0x00},
    {OV2740_8BIT, {0x3104}, 0x01},
    {OV2740_8BIT, {0x3106}, 0x01},
    {OV2740_8BIT, {0x3500}, 0x00},
    {OV2740_8BIT, {0x3501}, 0x44},
    {OV2740_8BIT, {0x3502}, 0x40},
    {OV2740_8BIT, {0x3503}, 0x88},
    {OV2740_8BIT, {0x3507}, 0x00},
    {OV2740_8BIT, {0x3508}, 0x00},
    {OV2740_8BIT, {0x3509}, 0x80},
    {OV2740_8BIT, {0x350c}, 0x00},
    {OV2740_8BIT, {0x350d}, 0x80},
    {OV2740_8BIT, {0x3510}, 0x00},
    {OV2740_8BIT, {0x3511}, 0x00},
    {OV2740_8BIT, {0x3512}, 0x20},
    {OV2740_8BIT, {0x3632}, 0x00},
    {OV2740_8BIT, {0x3633}, 0x10},
    {OV2740_8BIT, {0x3634}, 0x10},
    {OV2740_8BIT, {0x3635}, 0x10},
    {OV2740_8BIT, {0x3645}, 0x13},
    {OV2740_8BIT, {0x3646}, 0x81},
    {OV2740_8BIT, {0x3636}, 0x10},
    {OV2740_8BIT, {0x3651}, 0x0a},
    {OV2740_8BIT, {0x3656}, 0x02},
    {OV2740_8BIT, {0x3659}, 0x04},
    {OV2740_8BIT, {0x365a}, 0xda},
    {OV2740_8BIT, {0x365b}, 0xa2},
    {OV2740_8BIT, {0x365c}, 0x04},
    {OV2740_8BIT, {0x365d}, 0x1d},
    {OV2740_8BIT, {0x365e}, 0x1a},
    {OV2740_8BIT, {0x3662}, 0xd7},
    {OV2740_8BIT, {0x3667}, 0x78},
    {OV2740_8BIT, {0x3669}, 0x0a},
    {OV2740_8BIT, {0x366a}, 0x92},
    {OV2740_8BIT, {0x3700}, 0x54},
    {OV2740_8BIT, {0x3702}, 0x10},
    {OV2740_8BIT, {0x3706}, 0x42},
    {OV2740_8BIT, {0x3709}, 0x30},
    {OV2740_8BIT, {0x370b}, 0xc2},
    {OV2740_8BIT, {0x3714}, 0x63},
    {OV2740_8BIT, {0x3715}, 0x01},
    {OV2740_8BIT, {0x3716}, 0x00},
    {OV2740_8BIT, {0x371a}, 0x3e},
    {OV2740_8BIT, {0x3732}, 0x0e},
    {OV2740_8BIT, {0x3733}, 0x10},
    {OV2740_8BIT, {0x375f}, 0x0e},
    {OV2740_8BIT, {0x3768}, 0x30},
    {OV2740_8BIT, {0x3769}, 0x44},
    {OV2740_8BIT, {0x376a}, 0x22},
    {OV2740_8BIT, {0x377b}, 0x20},
    {OV2740_8BIT, {0x377c}, 0x00},
    {OV2740_8BIT, {0x377d}, 0x0c},
    {OV2740_8BIT, {0x3798}, 0x00},
    {OV2740_8BIT, {0x37a1}, 0x55},
    {OV2740_8BIT, {0x37a8}, 0x6d},
    {OV2740_8BIT, {0x37c2}, 0x04},
    {OV2740_8BIT, {0x37c5}, 0x00},
    {OV2740_8BIT, {0x37c8}, 0x00},
    {OV2740_8BIT, {0x3800}, 0x00},
    {OV2740_8BIT, {0x3801}, 0x00},
    {OV2740_8BIT, {0x3802}, 0x00},
    {OV2740_8BIT, {0x3803}, 0x00},
    {OV2740_8BIT, {0x3804}, 0x07},
    {OV2740_8BIT, {0x3805}, 0x8f},
    {OV2740_8BIT, {0x3806}, 0x04},
    {OV2740_8BIT, {0x3807}, 0x47},
    {OV2740_8BIT, {0x3808}, 0x07},
    {OV2740_8BIT, {0x3809}, 0x88},
    {OV2740_8BIT, {0x380a}, 0x04},
    {OV2740_8BIT, {0x380b}, 0x40},
    {OV2740_8BIT, {0x380c}, 0x04},
    {OV2740_8BIT, {0x380d}, 0x38},
    {OV2740_8BIT, {0x380e}, 0x04},
    {OV2740_8BIT, {0x380f}, 0x60},
    {OV2740_8BIT, {0x3810}, 0x00},
    {OV2740_8BIT, {0x3811}, 0x04},
    {OV2740_8BIT, {0x3812}, 0x00},
    {OV2740_8BIT, {0x3813}, 0x04},
    {OV2740_8BIT, {0x3814}, 0x01},
    {OV2740_8BIT, {0x3815}, 0x01},
    {OV2740_8BIT, {0x3820}, 0x80},
    {OV2740_8BIT, {0x3821}, 0x46},
    {OV2740_8BIT, {0x3822}, 0x84},
    {OV2740_8BIT, {0x3829}, 0x00},
    {OV2740_8BIT, {0x382a}, 0x01},
    {OV2740_8BIT, {0x382b}, 0x01},
    {OV2740_8BIT, {0x3830}, 0x04},
    {OV2740_8BIT, {0x3836}, 0x01},
    {OV2740_8BIT, {0x3837}, 0x08},
    {OV2740_8BIT, {0x3839}, 0x01},
    {OV2740_8BIT, {0x383a}, 0x00},
    {OV2740_8BIT, {0x383b}, 0x08},
    {OV2740_8BIT, {0x383c}, 0x00},
    {OV2740_8BIT, {0x3f0b}, 0x00},
    {OV2740_8BIT, {0x4001}, 0x20},
    {OV2740_8BIT, {0x4009}, 0x07},
    {OV2740_8BIT, {0x4003}, 0x10},
    {OV2740_8BIT, {0x4010}, 0xe0},
    {OV2740_8BIT, {0x4016}, 0x00},
    {OV2740_8BIT, {0x4017}, 0x10},
    {OV2740_8BIT, {0x4044}, 0x02},
    {OV2740_8BIT, {0x4304}, 0x08},
    {OV2740_8BIT, {0x4307}, 0x30},
    {OV2740_8BIT, {0x4320}, 0x80},
    {OV2740_8BIT, {0x4322}, 0x00},
    {OV2740_8BIT, {0x4323}, 0x00},
    {OV2740_8BIT, {0x4324}, 0x00},
    {OV2740_8BIT, {0x4325}, 0x00},
    {OV2740_8BIT, {0x4326}, 0x00},
    {OV2740_8BIT, {0x4327}, 0x00},
    {OV2740_8BIT, {0x4328}, 0x00},
    {OV2740_8BIT, {0x4329}, 0x00},
    {OV2740_8BIT, {0x432c}, 0x03},
    {OV2740_8BIT, {0x432d}, 0x81},
    {OV2740_8BIT, {0x4501}, 0x84},
    {OV2740_8BIT, {0x4502}, 0x40},
    {OV2740_8BIT, {0x4503}, 0x18},
    {OV2740_8BIT, {0x4504}, 0x04},
    {OV2740_8BIT, {0x4508}, 0x02},
    {OV2740_8BIT, {0x4601}, 0x10},
    {OV2740_8BIT, {0x4800}, 0x00},
    {OV2740_8BIT, {0x4816}, 0x52},
    {OV2740_8BIT, {0x481F}, 0x29},
    {OV2740_8BIT, {0x4820}, 0x01},
    {OV2740_8BIT, {0x4837}, 0x16},
    {OV2740_8BIT, {0x5000}, 0x7f},
    {OV2740_8BIT, {0x5001}, 0x00},
    {OV2740_8BIT, {0x5005}, 0x38},
    {OV2740_8BIT, {0x501e}, 0x0d},
    {OV2740_8BIT, {0x5040}, 0x00},
    {OV2740_8BIT, {0x58F4}, 0x32},
    {OV2740_8BIT, {0x5901}, 0x00},
    {OV2740_8BIT, {0x0100 }, 0x00},
    {OV2740_TOK_TERM, {0}, 0}
};

/************************** settings for ov2740 *************************/
static struct ov2740_reg const ov2740_1452x1092_30fps[] = {
	//1452x1092 30fps
	//@@ MIPI10bit_1452x1092_30fps_2lanes_576Mbps
    {OV2740_8BIT, {0x0302}, 0x1e},
    {OV2740_8BIT, {0x0303}, 0x00},
    {OV2740_8BIT, {0x030d}, 0x1e},
    {OV2740_8BIT, {0x030e}, 0x02},
    {OV2740_8BIT, {0x030f}, 0x04},
    {OV2740_8BIT, {0x0312}, 0x01},
    {OV2740_8BIT, {0x3501}, 0x44},
    {OV2740_8BIT, {0x3502}, 0x40},
    {OV2740_8BIT, {0x3714}, 0x63},
    {OV2740_8BIT, {0x3768}, 0x30},
    {OV2740_8BIT, {0x37c2}, 0x04},
    {OV2740_8BIT, {0x3800}, 0x00},
    {OV2740_8BIT, {0x3801}, 0xec},
    {OV2740_8BIT, {0x3802}, 0x00},
    {OV2740_8BIT, {0x3803}, 0x00},
    {OV2740_8BIT, {0x3804}, 0x06},
    {OV2740_8BIT, {0x3805}, 0xa3},
    {OV2740_8BIT, {0x3806}, 0x04},
    {OV2740_8BIT, {0x3807}, 0x47},
    {OV2740_8BIT, {0x3808}, 0x05},
    {OV2740_8BIT, {0x3809}, 0xac},
    {OV2740_8BIT, {0x380a}, 0x04},
    {OV2740_8BIT, {0x380b}, 0x44},
    {OV2740_8BIT, {0x380c}, 0x04},
    {OV2740_8BIT, {0x380d}, 0x38},
    {OV2740_8BIT, {0x380e}, 0x06},
    {OV2740_8BIT, {0x380f}, 0xf0},
    {OV2740_8BIT, {0x3810}, 0x00},
    {OV2740_8BIT, {0x3811}, 0x04},
    {OV2740_8BIT, {0x3812}, 0x00},
    {OV2740_8BIT, {0x3813}, 0x04},
    {OV2740_8BIT, {0x3814}, 0x01},
    {OV2740_8BIT, {0x3815}, 0x01},
    {OV2740_8BIT, {0x3820}, 0x80},
    {OV2740_8BIT, {0x3821}, 0x46},
    {OV2740_8BIT, {0x3822}, 0x84},
    {OV2740_8BIT, {0x3829}, 0x00},
    {OV2740_8BIT, {0x382a}, 0x01},
    {OV2740_8BIT, {0x382b}, 0x01},
    {OV2740_8BIT, {0x3830}, 0x04},
    {OV2740_8BIT, {0x3836}, 0x01},
    {OV2740_8BIT, {0x4009}, 0x07},
    {OV2740_8BIT, {0x4601}, 0x10},
    {OV2740_8BIT, {0x4837}, 0x1b},//2015.09.23, update to fix 4:3 resolution issue
    {OV2740_8BIT, {0x5000}, 0xff}, //enable LENC
    {OV2740_TOK_TERM, {0}, 0}
};
static struct ov2740_reg const ov2740_1296x736_30fps[] = {
	// 1296x736 30fps
	//@@ MIPI10bit_1296x736_30fps_2lanes_288Mbps
    {OV2740_8BIT, {0x0302}, 0x1e},
    {OV2740_8BIT, {0x0303}, 0x01},
    {OV2740_8BIT, {0x030d}, 0x1e},
    {OV2740_8BIT, {0x030e}, 0x02},
    {OV2740_8BIT, {0x030f}, 0x04},
    {OV2740_8BIT, {0x0312}, 0x01},
    {OV2740_8BIT, {0x3501}, 0x44},
    {OV2740_8BIT, {0x3502}, 0x40},
    {OV2740_8BIT, {0x3714}, 0x63},
    {OV2740_8BIT, {0x3768}, 0x30},
    {OV2740_8BIT, {0x37c2}, 0x04},
    {OV2740_8BIT, {0x3800}, 0x01},
    {OV2740_8BIT, {0x3801}, 0x3c},
    {OV2740_8BIT, {0x3802}, 0x00},
    {OV2740_8BIT, {0x3803}, 0xb0},
    {OV2740_8BIT, {0x3804}, 0x06},
    {OV2740_8BIT, {0x3805}, 0x53},
    {OV2740_8BIT, {0x3806}, 0x03},
    {OV2740_8BIT, {0x3807}, 0x91},
    {OV2740_8BIT, {0x3808}, 0x05},
    {OV2740_8BIT, {0x3809}, 0x10},
    {OV2740_8BIT, {0x380a}, 0x02},
    {OV2740_8BIT, {0x380b}, 0xe0},
    {OV2740_8BIT, {0x380c}, 0x08},
    {OV2740_8BIT, {0x380d}, 0x70},
    {OV2740_8BIT, {0x380e}, 0x03},
    {OV2740_8BIT, {0x380f}, 0x78},
    {OV2740_8BIT, {0x3810}, 0x00},
    {OV2740_8BIT, {0x3811}, 0x04},
    {OV2740_8BIT, {0x3812}, 0x00},
    {OV2740_8BIT, {0x3813}, 0x02},
    {OV2740_8BIT, {0x3814}, 0x01},
    {OV2740_8BIT, {0x3815}, 0x01},
    {OV2740_8BIT, {0x3820}, 0x80},
    {OV2740_8BIT, {0x3821}, 0x46},
    {OV2740_8BIT, {0x3822}, 0x84},
    {OV2740_8BIT, {0x3829}, 0x00},
    {OV2740_8BIT, {0x382a}, 0x01},
    {OV2740_8BIT, {0x382b}, 0x01},
    {OV2740_8BIT, {0x3830}, 0x04},
    {OV2740_8BIT, {0x3836}, 0x01},
    {OV2740_8BIT, {0x4009}, 0x07},
    {OV2740_8BIT, {0x4601}, 0x10},
    {OV2740_8BIT, {0x4837}, 0x37},
    {OV2740_8BIT, {0x5000}, 0xff},
    {OV2740_TOK_TERM, {0}, 0}
};

static struct ov2740_reg const ov2740_736x496_30fps[] = {
    //@@ MIPI10bit_736x496_binning_30fps_2lanes_288Mbps
    {OV2740_8BIT, {0x0302}, 0x1e},
    {OV2740_8BIT, {0x0303}, 0x01},
    {OV2740_8BIT, {0x030d}, 0x1e},
    {OV2740_8BIT, {0x030e}, 0x02},
    {OV2740_8BIT, {0x030f}, 0x04},
    {OV2740_8BIT, {0x0312}, 0x01},
    {OV2740_8BIT, {0x3501}, 0x22},
    {OV2740_8BIT, {0x3502}, 0x00},
    {OV2740_8BIT, {0x3714}, 0x26},
    {OV2740_8BIT, {0x3768}, 0x20},
    {OV2740_8BIT, {0x37c2}, 0x14},
    {OV2740_8BIT, {0x3800}, 0x00},
    {OV2740_8BIT, {0x3801}, 0xdc},
    {OV2740_8BIT, {0x3802}, 0x00},
    {OV2740_8BIT, {0x3803}, 0x2a},
    {OV2740_8BIT, {0x3804}, 0x06},
    {OV2740_8BIT, {0x3805}, 0xb3},
    {OV2740_8BIT, {0x3806}, 0x04},
    {OV2740_8BIT, {0x3807}, 0x1d},
    {OV2740_8BIT, {0x3808}, 0x02},
    {OV2740_8BIT, {0x3809}, 0xe0},
    {OV2740_8BIT, {0x380a}, 0x01},
    {OV2740_8BIT, {0x380b}, 0xf0},
    {OV2740_8BIT, {0x380c}, 0x04},
    {OV2740_8BIT, {0x380d}, 0x38},
    {OV2740_8BIT, {0x380e}, 0x06},
    {OV2740_8BIT, {0x380f}, 0xf0},
    {OV2740_8BIT, {0x3810}, 0x00},
    {OV2740_8BIT, {0x3811}, 0x04},
    {OV2740_8BIT, {0x3812}, 0x00},
    {OV2740_8BIT, {0x3813}, 0x02},
    {OV2740_8BIT, {0x3814}, 0x03},
    {OV2740_8BIT, {0x3815}, 0x01},
    {OV2740_8BIT, {0x3820}, 0x80},
    {OV2740_8BIT, {0x3821}, 0x67},
    {OV2740_8BIT, {0x3822}, 0x84},
    {OV2740_8BIT, {0x3829}, 0x00},
    {OV2740_8BIT, {0x382a}, 0x03},
    {OV2740_8BIT, {0x382b}, 0x01},
    {OV2740_8BIT, {0x3830}, 0x04},
    {OV2740_8BIT, {0x3836}, 0x02},
    {OV2740_8BIT, {0x4009}, 0x03},
    {OV2740_8BIT, {0x4601}, 0x50},
    {OV2740_8BIT, {0x4837}, 0x37},
    {OV2740_8BIT, {0x5000}, 0xff},
    {OV2740_TOK_TERM, {0}, 0}
};


static struct ov2740_reg const ov2740_336x256_30fps[] = {
           //@@ MIPI10bit_336x256_30fps_2lanes_228Mbps
    {OV2740_8BIT, {0x0302}, 0x1e},
    {OV2740_8BIT, {0x0303}, 0x01},
    {OV2740_8BIT, {0x030d}, 0x1e},
    {OV2740_8BIT, {0x030e}, 0x02},
    {OV2740_8BIT, {0x030f}, 0x04},
    {OV2740_8BIT, {0x0312}, 0x01},
    {OV2740_8BIT, {0x3501}, 0x10},
    {OV2740_8BIT, {0x3502}, 0x00},
    {OV2740_8BIT, {0x3632}, 0x01},
    {OV2740_8BIT, {0x3714}, 0x2b},
    {OV2740_8BIT, {0x3768}, 0x20},
    {OV2740_8BIT, {0x3800}, 0x01},
    {OV2740_8BIT, {0x3801}, 0x08},
    {OV2740_8BIT, {0x3802}, 0x00},
    {OV2740_8BIT, {0x3803}, 0x14},
    {OV2740_8BIT, {0x3804}, 0x06},
    {OV2740_8BIT, {0x3805}, 0x87},
    {OV2740_8BIT, {0x3806}, 0x04},
    {OV2740_8BIT, {0x3807}, 0x33},
    {OV2740_8BIT, {0x3808}, 0x01},
    {OV2740_8BIT, {0x3809}, 0x50},
    {OV2740_8BIT, {0x380a}, 0x01},
    {OV2740_8BIT, {0x380b}, 0x00},
    {OV2740_8BIT, {0x380c}, 0x04},
    {OV2740_8BIT, {0x380d}, 0x38},
    {OV2740_8BIT, {0x380e}, 0x06},
    {OV2740_8BIT, {0x380f}, 0xf0},
    {OV2740_8BIT, {0x3810}, 0x00},
    {OV2740_8BIT, {0x3811}, 0x08},
    {OV2740_8BIT, {0x3812}, 0x00},
    {OV2740_8BIT, {0x3813}, 0x04},
    {OV2740_8BIT, {0x3814}, 0x07},
    {OV2740_8BIT, {0x3815}, 0x01},
    {OV2740_8BIT, {0x3820}, 0x80},
    {OV2740_8BIT, {0x3821}, 0xe6},
    {OV2740_8BIT, {0x3822}, 0x84},
    {OV2740_8BIT, {0x3829}, 0x00},
    {OV2740_8BIT, {0x382a}, 0x07},
    {OV2740_8BIT, {0x382b}, 0x01},
    {OV2740_8BIT, {0x3830}, 0x04},
    {OV2740_8BIT, {0x3836}, 0x02},
    {OV2740_8BIT, {0x3837}, 0x08},
    {OV2740_8BIT, {0x4009}, 0x03},
    {OV2740_8BIT, {0x4601}, 0x28},
    {OV2740_8BIT, {0x4837}, 0x37},
    {OV2740_8BIT, {0x5000}, 0xff},
    {OV2740_TOK_TERM, {0}, 0}
};

static struct ov2740_reg const ov2740_1632x1092_30fps[] = {
     //@@ MIPI10bit_1632x1092_30fps_2lanes_576Mbps
    {OV2740_8BIT, {0x0302}, 0x1e},
    {OV2740_8BIT, {0x0303}, 0x00},
    {OV2740_8BIT, {0x030d}, 0x1e},
    {OV2740_8BIT, {0x030e}, 0x02},
    {OV2740_8BIT, {0x030f}, 0x04},
    {OV2740_8BIT, {0x0312}, 0x01},
    {OV2740_8BIT, {0x3501}, 0x44},
    {OV2740_8BIT, {0x3502}, 0x40},
    {OV2740_8BIT, {0x3714}, 0x63},
    {OV2740_8BIT, {0x3768}, 0x30},
    {OV2740_8BIT, {0x37c2}, 0x04},
    {OV2740_8BIT, {0x3800}, 0x00},
    {OV2740_8BIT, {0x3801}, 0x92},
    {OV2740_8BIT, {0x3802}, 0x00},
    {OV2740_8BIT, {0x3803}, 0x00},
    {OV2740_8BIT, {0x3804}, 0x06},
    {OV2740_8BIT, {0x3805}, 0xff},
    {OV2740_8BIT, {0x3806}, 0x04},
    {OV2740_8BIT, {0x3807}, 0x47},
    {OV2740_8BIT, {0x3808}, 0x06},
    {OV2740_8BIT, {0x3809}, 0x60},
    {OV2740_8BIT, {0x380a}, 0x04},
    {OV2740_8BIT, {0x380b}, 0x44},
    {OV2740_8BIT, {0x380c}, 0x04},
    {OV2740_8BIT, {0x380d}, 0x38},
    {OV2740_8BIT, {0x380e}, 0x06},
    {OV2740_8BIT, {0x380f}, 0xf0},
    {OV2740_8BIT, {0x3810}, 0x00},
    {OV2740_8BIT, {0x3811}, 0x04},
    {OV2740_8BIT, {0x3812}, 0x00},
    {OV2740_8BIT, {0x3813}, 0x04},
    {OV2740_8BIT, {0x3814}, 0x01},
    {OV2740_8BIT, {0x3815}, 0x01},
    {OV2740_8BIT, {0x3820}, 0x80},
    {OV2740_8BIT, {0x3821}, 0x46},
    {OV2740_8BIT, {0x3822}, 0x84},
    {OV2740_8BIT, {0x3829}, 0x00},
    {OV2740_8BIT, {0x382a}, 0x01},
    {OV2740_8BIT, {0x382b}, 0x01},
    {OV2740_8BIT, {0x3830}, 0x04},
    {OV2740_8BIT, {0x3836}, 0x01},
    {OV2740_8BIT, {0x4009}, 0x07},
    {OV2740_8BIT, {0x4601}, 0x10},
    {OV2740_8BIT, {0x4837}, 0x1b},
    {OV2740_8BIT, {0x5000}, 0xff},

    {OV2740_TOK_TERM, {0}, 0}

};

static struct ov2740_reg const ov2740_1932x1092_30fps[] = {
    //@@ MIPI10bit_1932x1092_30fps_2lanes_576Mbps
    {OV2740_8BIT, {0x0302}, 0x1e},
    {OV2740_8BIT, {0x0303}, 0x00},
    {OV2740_8BIT, {0x030d}, 0x1e},
    {OV2740_8BIT, {0x030e}, 0x02},
    {OV2740_8BIT, {0x0312}, 0x01},
    {OV2740_8BIT, {0x3501}, 0x44},
    {OV2740_8BIT, {0x3502}, 0x40},
    {OV2740_8BIT, {0x3714}, 0x63},
    {OV2740_8BIT, {0x3768}, 0x30},
    {OV2740_8BIT, {0x37c2}, 0x04},
    {OV2740_8BIT, {0x3800}, 0x00},
    {OV2740_8BIT, {0x3801}, 0x00},
    {OV2740_8BIT, {0x3802}, 0x00},
    {OV2740_8BIT, {0x3803}, 0x00},
    {OV2740_8BIT, {0x3804}, 0x07},
    {OV2740_8BIT, {0x3805}, 0x8f},
    {OV2740_8BIT, {0x3806}, 0x04},
    {OV2740_8BIT, {0x3807}, 0x47},
    {OV2740_8BIT, {0x3808}, 0x07},
    {OV2740_8BIT, {0x3809}, 0x8c},
    {OV2740_8BIT, {0x380a}, 0x04},
    {OV2740_8BIT, {0x380b}, 0x44},
    {OV2740_8BIT, {0x380c}, 0x04},
    {OV2740_8BIT, {0x380d}, 0x38},
    {OV2740_8BIT, {0x380e}, 0x06},
    {OV2740_8BIT, {0x380f}, 0xf0},
    {OV2740_8BIT, {0x3810}, 0x00},
    {OV2740_8BIT, {0x3811}, 0x02},
    {OV2740_8BIT, {0x3812}, 0x00},
    {OV2740_8BIT, {0x3813}, 0x04},
    {OV2740_8BIT, {0x3814}, 0x01},
    {OV2740_8BIT, {0x3815}, 0x01},
    {OV2740_8BIT, {0x3820}, 0x80},
    {OV2740_8BIT, {0x3821}, 0x46},
    {OV2740_8BIT, {0x3822}, 0x84},
    {OV2740_8BIT, {0x3829}, 0x00},
    {OV2740_8BIT, {0x382a}, 0x01},
    {OV2740_8BIT, {0x382b}, 0x01},
    {OV2740_8BIT, {0x3830}, 0x04},
    {OV2740_8BIT, {0x3836}, 0x01},
    {OV2740_8BIT, {0x4009}, 0x07},
    {OV2740_8BIT, {0x4601}, 0x10},
    {OV2740_8BIT, {0x4837}, 0x1b},
    {OV2740_8BIT, {0x5000}, 0xff},
    {OV2740_TOK_TERM, {0}, 0}
};

/* TODO settings of preview/still/video will be updated with new use case */
static struct ov2740_resolution ov2740_res_preview[] = {
	{
		//4:3
		.desc = "ov2740_1452x1092_30fps",
		.regs = ov2740_1452x1092_30fps,
		.width = 1452,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		//3:2
		.desc = "ov2740_1632x1092_30fps",
		.regs = ov2740_1632x1092_30fps,
		.width = 1632,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		//16:9
		.desc = "ov2740_1932x1092_30fps",
		.regs = ov2740_1932x1092_30fps,
		.width = 1932,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
};
#define N_RES_PREVIEW (ARRAY_SIZE(ov2740_res_preview))

/*
 * Disable non-preview configurations until the configuration selection is
 * improved.
 */
#if 0
struct ov2740_resolution ov2740_res_still[] = {
	{
		//4:3
		.desc = "ov2740_1452x1092_30fps",
		.regs = ov2740_1452x1092_30fps,
		.width = 1452,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		//3:2
		.desc = "ov2740_1632x1092_30fps",
		.regs = ov2740_1632x1092_30fps,
		.width = 1632,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		//16:9
		.desc = "ov2740_1932x1092_30fps",
		.regs = ov2740_1932x1092_30fps,
		.width = 1932,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
};
#define N_RES_STILL (ARRAY_SIZE(ov2740_res_still))

struct ov2740_resolution ov2740_res_video[] = {
	{
		//4:3
		.desc = "ov2740_1452x1092_30fps",
		.regs = ov2740_1452x1092_30fps,
		.width = 1452,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		//3:2
		.desc = "ov2740_1632x1092_30fps",
		.regs = ov2740_1632x1092_30fps,
		.width = 1632,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
	{
		//16:9
		.desc = "ov2740_1932x1092_30fps",
		.regs = ov2740_1932x1092_30fps,
		.width = 1932,
		.height = 1092,
		.fps = 30,
		.pix_clk_freq = 58 ,
		.pixels_per_line = 0x0438, /* consistent with regs arrays */
		.lines_per_frame = 0x06f0, /* consistent with regs arrays */
		.bin_factor_x = 0,
		.bin_factor_y = 0,
		.used = 0,
		.skip_frames = 1,
	},
};
#define N_RES_VIDEO (ARRAY_SIZE(ov2740_res_video))
#endif

struct ov2740_resolution *ov2740_res = ov2740_res_preview;
static int N_RES = N_RES_PREVIEW;


static struct ov2740_reg const ov2740_suspend[] = {
	 {OV2740_8BIT,  {0x0100}, 0x0},
	 {OV2740_TOK_TERM, {0}, 0}
};

static struct ov2740_reg const ov2740_streaming[] = {
	 {OV2740_8BIT,  {0x0100}, 0x1},
	 {OV2740_TOK_TERM, {0}, 0}
};

static struct ov2740_reg const ov2740_param_hold[] = {
	{OV2740_8BIT,  {0x3208}, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{OV2740_TOK_TERM, {0}, 0}
};

static struct ov2740_reg const ov2740_param_update[] = {
        {OV2740_8BIT,  {0x3208}, 0x10},	/* GROUPED_PARAMETER_HOLD */
	{OV2740_8BIT,  {0x320B}, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{OV2740_8BIT,  {0x3208}, 0xA0},	/* GROUPED_PARAMETER_HOLD */
	{OV2740_TOK_TERM, {0}, 0}
};


static int
ov2740_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val);
static int ov2740_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val);
static int ov2740_s_stream(struct v4l2_subdev *sd, int enable);
#endif

