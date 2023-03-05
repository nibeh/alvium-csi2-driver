#ifndef ALVIUM_REGS_H
#define ALVIUM_REGS_H

////////////////////////////////////////////////////////////////////////////////
// DEFINES
////////////////////////////////////////////////////////////////////////////////

/* D-PHY 1.2 clock frequency range (up to 2.5 Gbps per lane, DDR) */
// TODO where do these csi clk values come from?
#define CSI_HOST_CLK_MIN_FREQ 40000000
#define CSI_HOST_CLK_MAX_FREQ 750000000
#define CSI_HOST_CLK_MAX_FREQ_4L 735000000

// Version of the GenCP over CSI spec
#define GENCP_OVER_CCI_SPEC_VERSION_MAJOR 1
#define GENCP_OVER_CCI_SPEC_VERSION_MINOR 1
#define GENCP_OVER_CCI_SPEC_VERSION_PATCH 26

// Version of the BCRM spec
#define BCRM_SPEC_VERSION_MAJOR 1
#define BCRM_SPEC_VERSION_MINOR 1
#define BCRM_SPEC_VERSION_PATCH 9

// CCI registers
#define CCI_REG_LAYOUT_VER_32R 0x0000
#define CCI_DEVICE_CAP_64R 0x0008
#define CCI_GCPRM_16R 0x0010
#define CCI_BCRM_16R 0x0014
#define CCI_DEVICE_GUID_512R 0x0018
#define CCI_MANUF_NAME_512R 0x0058
#define CCI_MODEL_NAME_512R 0x0098
#define CCI_FAMILY_NAME_512R 0x00D8
#define CCI_DEVICE_VERSION_512R 0x0118
#define CCI_MANUF_INFO_512R 0x0158
#define CCI_SERIAL_NUM_512R 0x0198
#define CCI_USER_DEF_NAME_512R 0x01D8
#define CCI_CHECKSUM_32R 0x0218
#define CCI_CHANGE_MODE_8W 0x021C
#define CCI_CURRENT_MODE_8R 0x021D
#define CCI_SOFT_RESET_8W 0x021E
#define CCI_HEARTBEAT_8RW 0x021F

// GCPRM register offsets
#define GCPRM_LAYOUT_VERSION_32R 0x0000
#define GCPRM_GENCP_OUTBUF_ADDR_16R 0x0004
#define GCPRM_GENCP_OUTBUF_SIZE_16R 0x0008
#define GCPRM_GENCP_INBUF_ADDR_16R 0x000C
#define GCPRM_GENCP_INBUF_SIZE_16R 0x0010
#define GCPRM_GENCP_CHECKSUM_32R 0x0014
#define GCPRM_GENCP_OUTHANDSHAKE_8RW 0x0018
#define GCPRM_GENCP_INHANDSHAKE_8RW 0x001C
#define GCPRM_GENCP_OUT_SIZE_W16 0x0020
#define GCPRM_GENCP_IN_SIZE_R16 0x0024

// SBRM register offsets
#define SBRM_VERSION_32R 0x0000
#define SBRM_GENCP_CCI_DEV_CAP_64R 0x0004
#define SBRM_NUM_OF_STREAM_CHAN_32R 0x000C
#define SBRM_SUPP_CSI2_LANE_COUNTS_8R 0x0010
#define SBRM_CSI2_LANE_COUNT_8RW 0x0014
#define SBRM_MIN_SUPP_CSI2_CLK_FREQ_32R 0x0018
#define SBRM_MAX_SUPP_CSI2_CLK_FREQ_32R 0x001C
#define SBRM_CSI2_CLK_FREQ_32RW 0x0020
#define SBRM_SIRM_ADDR_64R 0x0024
#define SBRM_SIRM_LENGTH_32R 0x002C

// SIRM register offsets
#define SIRM_STREAM_ENABLE_8RW 0x0000
#define SIRM_LEADER_SIZE_32R 0x0004
#define SIRM_PAYLOAD_SIZE_64R 0x0008
#define SIRM_TRAILER_SIZE_32R 0x0010
#define SIRM_CSI2_DATA_ID_INQ1_64R 0x0014
#define SIRM_CSI2_DATA_ID_INQ2_64R 0x001C
#define SIRM_CSI2_DATA_ID_INQ3_64R 0x0024
#define SIRM_CSI2_DATA_ID_INQ4_64R 0x002C
#define SIRM_CSI2_DATA_ID_8RW 0x0034
#define SIRM_IPU_X_MIN_32W 0x0038
#define SIRM_IPU_X_MAX_32W 0x003C
#define SIRM_IPU_X_INC_32W 0x0040
#define SIRM_IPU_Y_MIN_32W 0x0044
#define SIRM_IPU_Y_MAX_32W 0x0048
#define SIRM_IPU_Y_INC_32W 0x004C
#define SIRM_IPU_X_32R 0x0050
#define SIRM_IPU_Y_32R 0x0054
#define SIRM_GENCP_IMAGE_SIZE_X_32R 0x0058
#define SIRM_GENCP_IMAGE_SIZE_Y_32R 0x005C
#define SIRM_GENCP_IMAGE_OFFSET_X_32R 0x0060
#define SIRM_GENCP_IMAGE_OFFSET_Y_32R 0x0064
#define SIRM_GENCP_IMAGE_PADDING_X_16R 0x0068
#define SIRM_GENCP_IMAGE_PIXEL_FORMAT_32R 0x006C
#define SIRM_GENCP_IMAGE_PAYLOAD_TYPE_16R 0x0070
#define SIRM_GENCP_VALID_PAYLOAD_SIZE_64R 0x0074
#define SIRM_GENCP_CHUNK_LAYOUT_ID_32R 0x007C

// BCRM register offsets
#define BCRM_VERSION_32R 0x0000
#define BCRM_FEATURE_INQUIRY_64R 0x0008
#define BCRM_DEVICE_FIRMWARE_VERSION_64R 0x0010
#define BCRM_WRITE_HANDSHAKE_8RW 0x0018
#define BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R 0x0040
#define BCRM_CSI2_LANE_COUNT_8RW 0x0044
#define BCRM_CSI2_CLOCK_MIN_32R 0x0048
#define BCRM_CSI2_CLOCK_MAX_32R 0x004C
#define BCRM_CSI2_CLOCK_32RW 0x0050
#define BCRM_BUFFER_SIZE_32R 0x0054
#define BCRM_IPU_X_MIN_32W 0x0058
#define BCRM_IPU_X_MAX_32W 0x005C
#define BCRM_IPU_X_INC_32W 0x0060
#define BCRM_IPU_Y_MIN_32W 0x0064
#define BCRM_IPU_Y_MAX_32W 0x0068
#define BCRM_IPU_Y_INC_32W 0x006C
#define BCRM_IPU_X_32R 0x0070
#define BCRM_IPU_Y_32R 0x0074
#define BCRM_PHY_RESET_8RW 0x0078
#define BCRM_ACQUISITION_START_8RW 0x0080
#define BCRM_ACQUISITION_STOP_8RW 0x0084
#define BCRM_ACQUISITION_ABORT_8RW 0x0088
#define BCRM_ACQUISITION_STATUS_8R 0x008C
#define BCRM_ACQUISITION_FRAME_RATE_64RW 0x0090
#define BCRM_ACQUISITION_FRAME_RATE_MIN_64R 0x0098
#define BCRM_ACQUISITION_FRAME_RATE_MAX_64R 0x00A0
#define BCRM_ACQUISITION_FRAME_RATE_INC_64R 0x00A8
#define BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW 0x00B0
#define BCRM_FRAME_START_TRIGGER_MODE_8RW 0x00B4
#define BCRM_FRAME_START_TRIGGER_SOURCE_8RW 0x00B8
#define BCRM_FRAME_START_TRIGGER_ACTIVATION_8RW 0x00BC
#define BCRM_FRAME_START_TRIGGER_SOFTWARE_8W 0x00C0
#define BCRM_FRAME_START_TRIGGER_DELAY_32RW 0x00C4
#define BCRM_EXPOSURE_ACTIVE_LINE_MODE_8RW 0x00C8
#define BCRM_EXPOSURE_ACTIVE_LINE_SELECTOR_8RW 0x00CC
#define BCRM_LINE_CONFIGURATION_32RW 0x00D0
#define BCRM_IMG_WIDTH_32RW 0x0100
#define BCRM_IMG_WIDTH_MIN_32R 0x0104
#define BCRM_IMG_WIDTH_MAX_32R 0x0108
#define BCRM_IMG_WIDTH_INC_32R 0x010C
#define BCRM_IMG_HEIGHT_32RW 0x0110
#define BCRM_IMG_HEIGHT_MIN_32R 0x0114
#define BCRM_IMG_HEIGHT_MAX_32R 0x0118
#define BCRM_IMG_HEIGHT_INC_32R 0x011C
#define BCRM_IMG_OFFSET_X_32RW 0x0120
#define BCRM_IMG_OFFSET_X_MIN_32R 0x0124
#define BCRM_IMG_OFFSET_X_MAX_32R 0x0128
#define BCRM_IMG_OFFSET_X_INC_32R 0x012C
#define BCRM_IMG_OFFSET_Y_32RW 0x0130
#define BCRM_IMG_OFFSET_Y_MIN_32R 0x0134
#define BCRM_IMG_OFFSET_Y_MAX_32R 0x0138
#define BCRM_IMG_OFFSET_Y_INC_32R 0x013C
#define BCRM_IMG_MIPI_DATA_FORMAT_32RW 0x0140
#define BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R 0x0148
#define BCRM_IMG_BAYER_PATTERN_INQUIRY_8R 0x0150
#define BCRM_IMG_BAYER_PATTERN_8RW 0x0154
#define BCRM_IMG_REVERSE_X_8RW 0x0158
#define BCRM_IMG_REVERSE_Y_8RW 0x015C
#define BCRM_SENSOR_WIDTH_32R 0x0160
#define BCRM_SENSOR_HEIGHT_32R 0x0164
#define BCRM_WIDTH_MAX_32R 0x0168
#define BCRM_HEIGHT_MAX_32R 0x016C
#define BCRM_EXPOSURE_TIME_64RW 0x0180
#define BCRM_EXPOSURE_TIME_MIN_64R 0x0188
#define BCRM_EXPOSURE_TIME_MAX_64R 0x0190
#define BCRM_EXPOSURE_TIME_INC_64R 0x0198
#define BCRM_EXPOSURE_AUTO_8RW 0x01A0
#define BCRM_INTENSITY_AUTO_PRECEDENCE_8RW 0x01A4
#define BCRM_INTENSITY_AUTO_PRECEDENCE_VALUE_32RW 0x01A8
#define BCRM_INTENSITY_AUTO_PRECEDENCE_MIN_32R 0x01AC
#define BCRM_INTENSITY_AUTO_PRECEDENCE_MAX_32R 0x01B0
#define BCRM_INTENSITY_AUTO_PRECEDENCE_INC_32R 0x01B4
#define BCRM_BLACK_LEVEL_32RW 0x01B8
#define BCRM_BLACK_LEVEL_MIN_32R 0x01BC
#define BCRM_BLACK_LEVEL_MAX_32R 0x01C0
#define BCRM_BLACK_LEVEL_INC_32R 0x01C4
#define BCRM_GAIN_64RW 0x01C8
#define BCRM_GAIN_MIN_64R 0x01D0
#define BCRM_GAIN_MAX_64R 0x01D8
#define BCRM_GAIN_INC_64R 0x01E0
#define BCRM_GAIN_AUTO_8RW 0x01E8
#define BCRM_GAMMA_64RW 0x01F0
#define BCRM_GAMMA_MIN_64R 0x01F8
#define BCRM_GAMMA_MAX_64R 0x0200
#define BCRM_GAMMA_INC_64R 0x0208
#define BCRM_CONTRAST_VALUE_32RW 0x0214
#define BCRM_CONTRAST_VALUE_MIN_32R 0x0218
#define BCRM_CONTRAST_VALUE_MAX_32R 0x021C
#define BCRM_CONTRAST_VALUE_INC_32R 0x0220
#define BCRM_SATURATION_32RW 0x0240
#define BCRM_SATURATION_MIN_32R 0x0244
#define BCRM_SATURATION_MAX_32R 0x0248
#define BCRM_SATURATION_INC_32R 0x024C
#define BCRM_HUE_32RW 0x0250
#define BCRM_HUE_MIN_32R 0x0254
#define BCRM_HUE_MAX_32R 0x0258
#define BCRM_HUE_INC_32R 0x025C
#define BCRM_ALL_BALANCE_RATIO_64RW 0x0260
#define BCRM_ALL_BALANCE_RATIO_MIN_64R 0x0268
#define BCRM_ALL_BALANCE_RATIO_MAX_64R 0x0270
#define BCRM_ALL_BALANCE_RATIO_INC_64R 0x0278
#define BCRM_RED_BALANCE_RATIO_64RW 0x0280
#define BCRM_RED_BALANCE_RATIO_MIN_64R 0x0288
#define BCRM_RED_BALANCE_RATIO_MAX_64R 0x0290
#define BCRM_RED_BALANCE_RATIO_INC_64R 0x0298
#define BCRM_GREEN_BALANCE_RATIO_64RW 0x02A0
#define BCRM_GREEN_BALANCE_RATIO_MIN_64R 0x02A8
#define BCRM_GREEN_BALANCE_RATIO_MAX_64R 0x02B0
#define BCRM_GREEN_BALANCE_RATIO_INC_64R 0x02B8
#define BCRM_BLUE_BALANCE_RATIO_64RW 0x02C0
#define BCRM_BLUE_BALANCE_RATIO_MIN_64R 0x02C8
#define BCRM_BLUE_BALANCE_RATIO_MAX_64R 0x02D0
#define BCRM_BLUE_BALANCE_RATIO_INC_64R 0x02D8
#define BCRM_WHITE_BALANCE_AUTO_8RW 0x02E0
#define BCRM_SHARPNESS_32RW 0x0300
#define BCRM_SHARPNESS_MIN_32R 0x0304
#define BCRM_SHARPNESS_MAX_32R 0x0308
#define BCRM_SHARPNESS_INC_32R 0x030C
#define BCRM_DEVICE_TEMPERATURE_32R 0x0310
#define BCRM_EXPOSURE_AUTO_MIN_64RW 0x0330
#define BCRM_EXPOSURE_AUTO_MAX_64RW 0x0338
#define BCRM_GAIN_AUTO_MIN_64RW 0x0340
#define BCRM_GAIN_AUTO_MAX_64RW 0x0348

#define _BCRM_LAST_ADDR BCRM_GAIN_AUTO_MAX_64RW

#define AV_CAM_REG_SIZE 2
#define AV_CAM_DATA_SIZE_8 1
#define AV_CAM_DATA_SIZE_16 2
#define AV_CAM_DATA_SIZE_32 4
#define AV_CAM_DATA_SIZE_64 8

////////////////////////////////////////////////////////////////////////////////
// ENUMS
////////////////////////////////////////////////////////////////////////////////

/* BCRM_IMG_MIPI_DATA_FORMAT_32RW register values */
enum MIPI_DATA_FORMAT {
	MIPI_DATA_FORMAT_YUV_420_8_LEG = 0x1A,
	MIPI_DATA_FORMAT_YUV_420_8 = 0x18,
	MIPI_DATA_FORMAT_YUV_420_10 = 0x19,
	MIPI_DATA_FORMAT_YUV_420_8_CSPS = 0x1C,
	MIPI_DATA_FORMAT_YUV_420_10_CSPS = 0x1D,
	MIPI_DATA_FORMAT_YUV_422_8 = 0x1E,
	MIPI_DATA_FORMAT_YUV_422_10 = 0x1F,
	MIPI_DATA_FORMAT_RGB888 = 0x24,
	MIPI_DATA_FORMAT_RGB666 = 0x23,
	MIPI_DATA_FORMAT_RGB565 = 0x22,
	MIPI_DATA_FORMAT_RGB555 = 0x21,
	MIPI_DATA_FORMAT_RGB444 = 0x20,
	MIPI_DATA_FORMAT_RAW6 = 0x28,
	MIPI_DATA_FORMAT_RAW7 = 0x29,
	MIPI_DATA_FORMAT_RAW8 = 0x2A,
	MIPI_DATA_FORMAT_RAW10 = 0x2B,
	MIPI_DATA_FORMAT_RAW12 = 0x2C,
	MIPI_DATA_FORMAT_RAW14 = 0x2D,
	MIPI_DATA_FORMAT_JPEG = 0x30
};

/* BCRM_IMG_BAYER_PATTERN_8RW register values */
enum BAYER_PATTERN {
	BAYER_PATTERN_MONO = 0,
	BAYER_PATTERN_GR = 1,
	BAYER_PATTERN_RG = 2,
	BAYER_PATTERN_GB = 3,
	BAYER_PATTERN_BG = 4
};

/* BCRM_IMG_REVERSE_X_8RW */
enum REVERSE_X {
	REVERSE_X_OFF = 0,
	REVERSE_X_FLIP_H = 1,
};

/* BCRM_IMG_REVERSE_X_8RW */
enum REVERSE_Y {
	REVERSE_Y_OFF = 0,
	REVERSE_Y_FLIP_V = 1,
};

/* BCRM_IMG_BAYER_PATTERN_8RW register values */
enum EXPOSURE_AUTO {
	EXPOSURE_AUTO_OFF = 0,
	EXPOSURE_AUTO_ONCE = 1,
	EXPOSURE_AUTO_CONTINUOUS = 2
};

/* BCRM_GAIN_AUTO_8RW register values */
enum GAIN_AUTO {
	GAIN_AUTO_OFF = 0,
	GAIN_AUTO_ONCE = 1,
	GAIN_AUTO_CONTINUOUS = 2
};

/* BCRM_WHITE_BALANCE_AUTO_8RW register values */
enum WHITEBALANCE_AUTO {
	WHITEBALANCE_AUTO_OFF = 0,
	WHITEBALANCE_AUTO_ONCE = 1,
	WHITEBALANCE_AUTO_CONTINUOUS = 2
};

/* BCRM_ACQUISITION_STATUS_8R register values */
enum ACQUISITION_STATUS {
	ACQUISITION_STATUS_STOPPED = 0,
	ACQUISITION_STATUS_RUNNING = 1
};

/* CCI Device capability String encoding */
enum CCI_STRING_ENC {
	CCI_STRING_ENC_ASCII = 0,
	CCI_STRING_ENC_UTF8 = 1,
	CCI_STRING_ENC_UTF16 = 2
};

enum CCI_REG_INFO {
	CCI_REGISTER_LAYOUT_VERSION = 0,
	RESERVED4BIT,
	DEVICE_CAPABILITIES,
	GCPRM_ADDRESS,
	RESERVED2BIT,
	BCRM_ADDRESS,
	RESERVED2BIT_2,
	DEVICE_GUID,
	MANUFACTURER_NAME,
	MODEL_NAME,
	FAMILY_NAME,
	DEVICE_VERSION,
	MANUFACTURER_INFO,
	SERIAL_NUMBER,
	USER_DEFINED_NAME,
	CHECKSUM,
	CHANGE_MODE,
	CURRENT_MODE,
	SOFT_RESET,
	HEARTBEAT,
	MAX_CMD = SOFT_RESET
};

////////////////////////////////////////////////////////////////////////////////
// UNIONS
////////////////////////////////////////////////////////////////////////////////
/* CCI_DEVICE_CAP_64R regsiter values */
union cci_device_caps_reg {
	struct {
		__u8 user_name : 1;
		__u8 bcrm : 1;
		__u8 gencp : 1;
		__u8 reserved : 1;
		__u8 string_encoding : 4;
		__u8 family_name : 1;
		__u64 reserved2 : 55;
	} caps;

	__u64 value;
};

/* BCRM_VERSION_32R register values */
union bcrm_version_reg {
	struct {
		__u16 minor : 16;
		__u16 major : 16;
	} handshake;

	__u32 value;
};

/* BCRM_DEVICE_FIRMWARE_VERSION_64R register values */
union bcrm_device_firmware_version_reg {
	struct {
		__u8 special : 8;
		__u8 major : 8;
		__u16 minor : 16;
		__u32 patch : 32;
	} version;

	__u64 value;
};

/* BCRM_WRITE_HANDSHAKE_8RW register values */
union bcrm_write_done_handshake_reg {
	struct {
		__u8 finished : 1;
		__u8 reserved : 6;
		__u8 handshake_supported : 1;
	} handshake;

	__u8 value;
};

/* BCRM_FEATURE_INQUIRY_64R register values */
union bcrm_feature_reg {
	struct {
		__u8 reverse_x_avail : 1;
		__u8 reverse_y_avail : 1;
		__u8 intensity_auto_prcedence_avail : 1;
		__u8 black_level_avail : 1;
		__u8 gain_avail : 1;
		__u8 gamma_avail : 1;
		__u8 contrast_avail : 1;
		__u8 saturation_avail : 1;
		__u8 hue_avail : 1;
		__u8 white_balance_avail : 1;
		__u8 sharpness_avail : 1;
		__u8 exposure_auto : 1;
		__u8 gain_auto : 1;
		__u8 white_balance_auto_avail : 1;
		__u8 device_temperature_avail : 1;
		__u8 acquisition_abort : 1;
		__u8 acquisition_frame_rate : 1;
		__u8 frame_trigger : 1;
		__u8 exposure_active_line_avail : 1;
		__u64 reserved : 45;
	} feature_inq;

	__u64 value;
};

/* BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R register values */
union bcrm_supported_lanecount_reg {
	struct {
		__u8 one_lane_avail : 1;
		__u8 two_lane_avail : 1;
		__u8 three_lane_avail : 1;
		__u8 four_lane_avail : 1;
		__u8 reserved : 4;
	} lane_count;

	__u8 value;
};

/* BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R register values */
union bcrm_avail_mipi_reg {
	struct {
		__u8 yuv420_8_leg_avail : 1;
		__u8 yuv420_8_avail : 1;
		__u8 yuv420_10_avail : 1;
		__u8 yuv420_8_csps_avail : 1;
		__u8 yuv420_10_csps_avail : 1;
		__u8 yuv422_8_avail : 1;
		__u8 yuv422_10_avail : 1;
		__u8 rgb888_avail : 1;
		__u8 rgb666_avail : 1;
		__u8 rgb565_avail : 1;
		__u8 rgb555_avail : 1;
		__u8 rgb444_avail : 1;
		__u8 raw6_avail : 1;
		__u8 raw7_avail : 1;
		__u8 raw8_avail : 1;
		__u8 raw10_avail : 1;
		__u8 raw12_avail : 1;
		__u8 raw14_avail : 1;
		__u8 jpeg_avail : 1;
		__u64 reserved : 45;
	} avail_mipi;

	__u64 value;
};

/* BCRM_IMG_BAYER_PATTERN_INQUIRY_8R register values */
union bcrm_bayer_inquiry_reg {
	struct {
		__u8 monochrome_avail : 1;
		__u8 bayer_GR_avail : 1;
		__u8 bayer_RG_avail : 1;
		__u8 bayer_GB_avail : 1;
		__u8 bayer_BG_avail : 1;
		__u8 reserved : 3;
	} bayer_pattern;

	__u8 value;
};

struct cci_cmd {
	__u8 command_index; /* diagnostc test name */
	const __u32 address; /* NULL for no alias name */
	__u32 byte_count;
};

static struct cci_cmd cci_cmd_tbl[MAX_CMD] = {
	/* command index        address */
	{ CCI_REGISTER_LAYOUT_VERSION, CCI_REG_LAYOUT_VER_32R, 4 },
	{ DEVICE_CAPABILITIES, CCI_DEVICE_CAP_64R, 8 },
	{ GCPRM_ADDRESS, CCI_GCPRM_16R, 2 },
	{ BCRM_ADDRESS, CCI_BCRM_16R, 2 },
	{ DEVICE_GUID, CCI_DEVICE_GUID_512R, 64 },
	{ MANUFACTURER_NAME, CCI_MANUF_NAME_512R, 64 },
	{ MODEL_NAME, CCI_MODEL_NAME_512R, 64 },
	{ FAMILY_NAME, CCI_FAMILY_NAME_512R, 64 },
	{ DEVICE_VERSION, CCI_DEVICE_VERSION_512R, 64 },
	{ MANUFACTURER_INFO, CCI_MANUF_INFO_512R, 64 },
	{ SERIAL_NUMBER, CCI_SERIAL_NUM_512R, 64 },
	{ USER_DEFINED_NAME, CCI_USER_DEF_NAME_512R, 64 },
	{ CHECKSUM, CCI_CHECKSUM_32R, 4 },
	{ CHANGE_MODE, CCI_CHANGE_MODE_8W, 1 },
	{ CURRENT_MODE, CCI_CURRENT_MODE_8R, 1 },
	{ SOFT_RESET, CCI_SOFT_RESET_8W, 1 },
	{ HEARTBEAT, CCI_HEARTBEAT_8RW, 1 },
};

struct __attribute__((__packed__)) cci_reg {
	__u32 layout_version;
	__u32 reserved_4bit;
	__u64 device_capabilities;
	__u16 gcprm_address;
	__u16 reserved_2bit;
	__u16 bcrm_addr;
	__u16 reserved_2bit_2;
	char device_guid[64];
	char manufacturer_name[64];
	char model_name[64];
	char family_name[64];
	char device_version[64];
	char manufacturer_info[64];
	char serial_number[64];
	char user_defined_name[64];
	__u32 checksum;
	__u8 change_mode;
	__u8 current_mode;
	__u8 soft_reset;
	__u8 heartbeat;
};

struct __attribute__((__packed__)) gencp_reg {
	__u32 gcprm_layout_version;
	__u16 gencp_out_buffer_address;
	__u16 reserved_2byte;
	__u16 gencp_out_buffer_size;
	__u16 reserved_2byte_1;
	__u16 gencp_in_buffer_address;
	__u16 reserved_2byte_2;
	__u16 gencp_in_buffer_size;
	__u16 reserved_2byte_3;
	__u32 checksum;
};

enum alvium_mode {
	ALVIUM_BCRM_MODE = 0,
	ALVIUM_GENCP_MODE = 1,
};

#define V4L2_AV_CSI2_BASE 0x1000
#define V4L2_AV_CSI2_WIDTH_R (V4L2_AV_CSI2_BASE + 0x0001)
#define V4L2_AV_CSI2_WIDTH_W (V4L2_AV_CSI2_BASE + 0x0002)
#define V4L2_AV_CSI2_WIDTH_MINVAL_R (V4L2_AV_CSI2_BASE + 0x0003)
#define V4L2_AV_CSI2_WIDTH_MAXVAL_R (V4L2_AV_CSI2_BASE + 0x0004)
#define V4L2_AV_CSI2_WIDTH_INCVAL_R (V4L2_AV_CSI2_BASE + 0x0005)
#define V4L2_AV_CSI2_HEIGHT_R (V4L2_AV_CSI2_BASE + 0x0006)
#define V4L2_AV_CSI2_HEIGHT_W (V4L2_AV_CSI2_BASE + 0x0007)
#define V4L2_AV_CSI2_HEIGHT_MINVAL_R (V4L2_AV_CSI2_BASE + 0x0008)
#define V4L2_AV_CSI2_HEIGHT_MAXVAL_R (V4L2_AV_CSI2_BASE + 0x0009)
#define V4L2_AV_CSI2_HEIGHT_INCVAL_R (V4L2_AV_CSI2_BASE + 0x000A)
#define V4L2_AV_CSI2_PIXELFORMAT_R (V4L2_AV_CSI2_BASE + 0x000B)
#define V4L2_AV_CSI2_PIXELFORMAT_W (V4L2_AV_CSI2_BASE + 0x000C)
#define V4L2_AV_CSI2_PALYLOADSIZE_R (V4L2_AV_CSI2_BASE + 0x000D)
#define V4L2_AV_CSI2_STREAMON_W (V4L2_AV_CSI2_BASE + 0x000E)
#define V4L2_AV_CSI2_STREAMOFF_W (V4L2_AV_CSI2_BASE + 0x000F)
#define V4L2_AV_CSI2_ABORT_W (V4L2_AV_CSI2_BASE + 0x0010)
#define V4L2_AV_CSI2_ACQ_STATUS_R (V4L2_AV_CSI2_BASE + 0x0011)
#define V4L2_AV_CSI2_HFLIP_R (V4L2_AV_CSI2_BASE + 0x0012)
#define V4L2_AV_CSI2_HFLIP_W (V4L2_AV_CSI2_BASE + 0x0013)
#define V4L2_AV_CSI2_VFLIP_R (V4L2_AV_CSI2_BASE + 0x0014)
#define V4L2_AV_CSI2_VFLIP_W (V4L2_AV_CSI2_BASE + 0x0015)
#define V4L2_AV_CSI2_OFFSET_X_W (V4L2_AV_CSI2_BASE + 0x0016)
#define V4L2_AV_CSI2_OFFSET_X_R (V4L2_AV_CSI2_BASE + 0x0017)
#define V4L2_AV_CSI2_OFFSET_X_MIN_R (V4L2_AV_CSI2_BASE + 0x0018)
#define V4L2_AV_CSI2_OFFSET_X_MAX_R (V4L2_AV_CSI2_BASE + 0x0019)
#define V4L2_AV_CSI2_OFFSET_X_INC_R (V4L2_AV_CSI2_BASE + 0x001A)
#define V4L2_AV_CSI2_OFFSET_Y_W (V4L2_AV_CSI2_BASE + 0x001B)
#define V4L2_AV_CSI2_OFFSET_Y_R (V4L2_AV_CSI2_BASE + 0x001C)
#define V4L2_AV_CSI2_OFFSET_Y_MIN_R (V4L2_AV_CSI2_BASE + 0x001D)
#define V4L2_AV_CSI2_OFFSET_Y_MAX_R (V4L2_AV_CSI2_BASE + 0x001E)
#define V4L2_AV_CSI2_OFFSET_Y_INC_R (V4L2_AV_CSI2_BASE + 0x001F)
#define V4L2_AV_CSI2_SENSOR_WIDTH_R (V4L2_AV_CSI2_BASE + 0x0020)
#define V4L2_AV_CSI2_SENSOR_HEIGHT_R (V4L2_AV_CSI2_BASE + 0x0021)
#define V4L2_AV_CSI2_MAX_WIDTH_R (V4L2_AV_CSI2_BASE + 0x0022)
#define V4L2_AV_CSI2_MAX_HEIGHT_R (V4L2_AV_CSI2_BASE + 0x0023)
#define V4L2_AV_CSI2_CURRENTMODE_R (V4L2_AV_CSI2_BASE + 0x0024)
#define V4L2_AV_CSI2_CHANGEMODE_W (V4L2_AV_CSI2_BASE + 0x0025)
#define V4L2_AV_CSI2_BAYER_PATTERN_R (V4L2_AV_CSI2_BASE + 0x0026)
#define V4L2_AV_CSI2_BAYER_PATTERN_W (V4L2_AV_CSI2_BASE + 0x0027)
#define V4L2_AV_CSI2_FRAMERATE_R (V4L2_AV_CSI2_BASE + 0x0028)
#define V4L2_AV_CSI2_FRAMERATE_W (V4L2_AV_CSI2_BASE + 0x0029)
#define V4L2_AV_CSI2_FRAMERATE_MINVAL_R (V4L2_AV_CSI2_BASE + 0x002A)
#define V4L2_AV_CSI2_FRAMERATE_MAXVAL_R (V4L2_AV_CSI2_BASE + 0x002B)
#define V4L2_AV_CSI2_FRAMERATE_INCVAL_R (V4L2_AV_CSI2_BASE + 0x002C)
#define V4L2_AV_CSI2_PHY_RESET_R (V4L2_AV_CSI2_BASE + 0x002D)
#define V4L2_AV_CSI2_PHY_RESET_W (V4L2_AV_CSI2_BASE + 0x002E)

// #define STR_HELPER(x) #x
// #define STR(x) STR_HELPER(x)

/* Driver release version */
// #define DRV_VER_MAJOR 1
// #define DRV_VER_MINOR 0
// #define DRV_VER_PATCH 0
// #define DRIVER_VERSION STR(DRV_VER_MAJOR) "." STR(DRV_VER_MINOR) "." STR(DRV_VER_PATCH)

#define BCRM_MAJOR_VERSION 0x0001
#define BCRM_MINOR_VERSION 0x0000
#define BCRM_DEVICE_VERSION 0x00010000

#define GCPRM_MAJOR_VERSION 0x0001
#define GCPRM_MINOR_VERSION 0x0000
#define GCPRM_DEVICE_VERSION 0x00010000

/* MIPI CSI-2 data types */
#define MIPI_DT_YUV420 0x18 /* YYY.../UYVY.... */
#define MIPI_DT_YUV420_LEGACY 0x1a /* UYY.../VYY...   */
#define MIPI_DT_YUV422 0x1e /* UYVY...         */
#define MIPI_DT_RGB444 0x20
#define MIPI_DT_RGB555 0x21
#define MIPI_DT_RGB565 0x22
#define MIPI_DT_RGB666 0x23
#define MIPI_DT_RGB888 0x24
#define MIPI_DT_RAW6 0x28
#define MIPI_DT_RAW7 0x29
#define MIPI_DT_RAW8 0x2a
#define MIPI_DT_RAW10 0x2b
#define MIPI_DT_RAW12 0x2c
#define MIPI_DT_RAW14 0x2d
#define MIPI_DT_CUSTOM 0x31

enum bayer_format {
	monochrome, /* 0 */
	bayer_gr,
	bayer_rg,
	bayer_gb,
	bayer_bg,
};

struct bcrm_to_v4l2 {
	int64_t min_bcrm;
	int64_t max_bcrm;
	int64_t step_bcrm;
	int32_t min_v4l2;
	int32_t max_v4l2;
	int32_t step_v4l2;
};

enum convert_type {
	min_enum, /* 0 */
	max_enum,
	step_enum,
};

#define EXP_ABS 100000UL
#define UHZ_TO_HZ 1000000UL
#define FRAQ_NUM 1000

#define CCI_REG_LAYOUT_MINVER_MASK (0x0000ffff)
#define CCI_REG_LAYOUT_MINVER_SHIFT (0)
#define CCI_REG_LAYOUT_MAJVER_MASK (0xffff0000)
#define CCI_REG_LAYOUT_MAJVER_SHIFT (16)

#define CCI_REG_LAYOUT_MINVER 0
#define CCI_REG_LAYOUT_MAJVER 1

#define AV_ATTR_REVERSE_X                                                      \
	{                                                                      \
		"Reverse X", 0                                                 \
	}
#define AV_ATTR_REVERSE_Y                                                      \
	{                                                                      \
		"Reverse Y", 1                                                 \
	}
// #define AV_ATTR_INTENSITY_AUTO        {"Intensity Auto",    2}
#define AV_ATTR_BRIGHTNESS                                                     \
	{                                                                      \
		"Brightness", 3                                                \
	}
/* Red & Blue balance features are enabled by default since it doesn't have
 * option in BCRM FEATURE REGISTER
 */
#define AV_ATTR_RED_BALANCE                                                    \
	{                                                                      \
		"Red Balance", 3                                               \
	}
#define AV_ATTR_BLUE_BALANCE                                                   \
	{                                                                      \
		"Blue Balance", 3                                              \
	}
#define AV_ATTR_GAIN                                                           \
	{                                                                      \
		"Gain", 4                                                      \
	}
#define AV_ATTR_GAMMA                                                          \
	{                                                                      \
		"Gamma", 5                                                     \
	}
#define AV_ATTR_CONTRAST                                                       \
	{                                                                      \
		"Contrast", 6                                                  \
	}
#define AV_ATTR_SATURATION                                                     \
	{                                                                      \
		"Saturation", 7                                                \
	}
#define AV_ATTR_HUE                                                            \
	{                                                                      \
		"Hue", 8                                                       \
	}
#define AV_ATTR_WHITEBALANCE                                                   \
	{                                                                      \
		"White Balance", 9                                             \
	}
#define AV_ATTR_SHARPNESS                                                      \
	{                                                                      \
		"Sharpnesss", 10                                               \
	}
#define AV_ATTR_EXPOSURE_AUTO                                                  \
	{                                                                      \
		"Exposure Auto", 11                                            \
	}
#define AV_ATTR_EXPOSURE_AUTO_MIN                                              \
	{                                                                      \
		"Exposure Auto Min", 11                                        \
	}
#define AV_ATTR_EXPOSURE_AUTO_MAX                                              \
	{                                                                      \
		"Exposure Auto Max", 11                                        \
	}
#define AV_ATTR_AUTOGAIN                                                       \
	{                                                                      \
		"Auto Gain", 12                                                \
	}
#define AV_ATTR_AUTOGAIN_MIN                                                   \
	{                                                                      \
		"Auto Gain Min", 12                                            \
	}
#define AV_ATTR_AUTOGAIN_MAX                                                   \
	{                                                                      \
		"Auto Gain Max", 12                                            \
	}
#define AV_ATTR_EXPOSURE                                                       \
	{                                                                      \
		"Exposure", 13                                                 \
	}
#define AV_ATTR_EXPOSURE_ABS                                                   \
	{                                                                      \
		"Exposure Absolute", 13                                        \
	}
#define AV_ATTR_WHITEBALANCE_AUTO                                              \
	{                                                                      \
		"Auto White Balance", 14                                       \
	}
#define AV_ATTR_EXPOSURE_ACTIVE_LINE_MODE                                      \
	{                                                                      \
		"Exposure Active Line Mode", 18                                \
	}
#define AV_ATTR_EXPOSURE_ACTIVE_LINE_SELECTOR                                  \
	{                                                                      \
		"Exposure Active Line Selector", 18                            \
	}
#define AV_ATTR_EXPOSURE_ACTIVE_INVERT                                         \
	{                                                                      \
		"Exposure Active Invert", 18                                   \
	}
#define AV_ATTR_LINK_FREQ                                                      \
	{                                                                      \
		"Link Frequency", 19                                           \
	}
#define AV_ATTR_TEMPERATURE                                                    \
	{                                                                      \
		"Device Temperature", 20                                       \
	}

struct alvium_ctrl_mapping {
	u8 reg_size;
	u8 data_size;
	u16 min_offset;
	u16 max_offset;
	u16 reg_offset;
	u16 step_offset;
	u32 id;
	u32 type;
	u32 flags;
	struct {
		s8 *name;
		u8 feature_avail;
	} attr;
};

#ifndef V4L2_CID_EXPOSURE_AUTO_MIN
#define V4L2_CID_EXPOSURE_AUTO_MIN (V4L2_CID_CAMERA_CLASS_BASE + 40)
#endif

#ifndef V4L2_CID_EXPOSURE_AUTO_MAX
#define V4L2_CID_EXPOSURE_AUTO_MAX (V4L2_CID_CAMERA_CLASS_BASE + 41)
#endif

#ifndef V4L2_CID_AUTOGAIN_MIN
#define V4L2_CID_AUTOGAIN_MIN (V4L2_CID_CAMERA_CLASS_BASE + 42)
#endif

#ifndef V4L2_CID_AUTOGAIN_MAX
#define V4L2_CID_AUTOGAIN_MAX (V4L2_CID_CAMERA_CLASS_BASE + 43)
#endif

#ifndef V4L2_CID_EXPOSURE_ACTIVE_LINE_MODE
#define V4L2_CID_EXPOSURE_ACTIVE_LINE_MODE (V4L2_CID_CAMERA_CLASS_BASE + 44)
#endif

#ifndef V4L2_CID_EXPOSURE_ACTIVE_LINE_SELECTOR
#define V4L2_CID_EXPOSURE_ACTIVE_LINE_SELECTOR (V4L2_CID_CAMERA_CLASS_BASE + 45)
#endif

#ifndef V4L2_CID_EXPOSURE_ACTIVE_INVERT
#define V4L2_CID_EXPOSURE_ACTIVE_INVERT (V4L2_CID_CAMERA_CLASS_BASE + 46)
#endif

const struct alvium_ctrl_mapping alvium_ctrl_mappings[] = {
	{
		.id = V4L2_CID_BRIGHTNESS,
		.attr = AV_ATTR_BRIGHTNESS,
		.min_offset = BCRM_BLACK_LEVEL_MIN_32R,
		.max_offset = BCRM_BLACK_LEVEL_MAX_32R,
		.reg_offset = BCRM_BLACK_LEVEL_32RW,
		.step_offset = BCRM_BLACK_LEVEL_INC_32R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_32,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
	},
	{
		.id = V4L2_CID_CONTRAST,
		.attr = AV_ATTR_CONTRAST,
		.min_offset = BCRM_CONTRAST_VALUE_MIN_32R,
		.max_offset = BCRM_CONTRAST_VALUE_MAX_32R,
		.reg_offset = BCRM_CONTRAST_VALUE_32RW,
		.step_offset = BCRM_CONTRAST_VALUE_INC_32R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_32,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
	},
	{
		.id = V4L2_CID_SATURATION,
		.attr = AV_ATTR_SATURATION,
		.min_offset = BCRM_SATURATION_MIN_32R,
		.max_offset = BCRM_SATURATION_MAX_32R,
		.reg_offset = BCRM_SATURATION_32RW,
		.step_offset = BCRM_SATURATION_INC_32R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_32,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
	},
	{
		.id = V4L2_CID_HUE,
		.attr = AV_ATTR_HUE,
		.min_offset = BCRM_HUE_MIN_32R,
		.max_offset = BCRM_HUE_MAX_32R,
		.reg_offset = BCRM_HUE_32RW,
		.step_offset = BCRM_HUE_INC_32R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_32,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
	},
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.attr = AV_ATTR_WHITEBALANCE_AUTO,
		.reg_offset = BCRM_WHITE_BALANCE_AUTO_8RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_8,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = 0,
	},
	{
		.id = V4L2_CID_DO_WHITE_BALANCE,
		.attr = AV_ATTR_WHITEBALANCE,
		.reg_offset = BCRM_WHITE_BALANCE_AUTO_8RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_8,
		.type = V4L2_CTRL_TYPE_BUTTON,
		.flags = 0,
	},
	{
		.id = V4L2_CID_RED_BALANCE,
		.attr = AV_ATTR_RED_BALANCE,
		.min_offset = BCRM_RED_BALANCE_RATIO_MIN_64R,
		.max_offset = BCRM_RED_BALANCE_RATIO_MAX_64R,
		.reg_offset = BCRM_RED_BALANCE_RATIO_64RW,
		.step_offset = BCRM_RED_BALANCE_RATIO_INC_64R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_BLUE_BALANCE,
		.attr = AV_ATTR_BLUE_BALANCE,
		.min_offset = BCRM_BLUE_BALANCE_RATIO_MIN_64R,
		.max_offset = BCRM_BLUE_BALANCE_RATIO_MAX_64R,
		.reg_offset = BCRM_BLUE_BALANCE_RATIO_64RW,
		.step_offset = BCRM_BLUE_BALANCE_RATIO_INC_64R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_GAMMA,
		.attr = AV_ATTR_GAMMA,
		.min_offset = BCRM_GAMMA_MIN_64R,
		.max_offset = BCRM_GAMMA_MAX_64R,
		.reg_offset = BCRM_GAMMA_64RW,
		.step_offset = BCRM_GAMMA_INC_64R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE,
		.attr = AV_ATTR_EXPOSURE,
		.min_offset = BCRM_EXPOSURE_TIME_MIN_64R,
		.max_offset = BCRM_EXPOSURE_TIME_MAX_64R,
		.reg_offset = BCRM_EXPOSURE_TIME_64RW,
		.step_offset = BCRM_EXPOSURE_TIME_INC_64R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_AUTOGAIN,
		.attr = AV_ATTR_AUTOGAIN,
		.reg_offset = BCRM_GAIN_AUTO_8RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_8,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = 0,
	},
	{
		.id = V4L2_CID_GAIN,
		.attr = AV_ATTR_GAIN,
		.min_offset = BCRM_GAIN_MIN_64R,
		.max_offset = BCRM_GAIN_MAX_64R,
		.reg_offset = BCRM_GAIN_64RW,
		.step_offset = BCRM_GAIN_INC_64R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_HFLIP,
		.attr = AV_ATTR_REVERSE_X,
		.reg_offset = BCRM_IMG_REVERSE_X_8RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_8,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = 0,
	},
	{
		.id = V4L2_CID_VFLIP,
		.attr = AV_ATTR_REVERSE_Y,
		.reg_offset = BCRM_IMG_REVERSE_Y_8RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_8,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = 0,
	},
	{
		.id = V4L2_CID_SHARPNESS,
		.attr = AV_ATTR_SHARPNESS,
		.min_offset = BCRM_SHARPNESS_MIN_32R,
		.max_offset = BCRM_SHARPNESS_MAX_32R,
		.reg_offset = BCRM_SHARPNESS_32RW,
		.step_offset = BCRM_SHARPNESS_INC_32R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_32,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_AUTO,
		.attr = AV_ATTR_EXPOSURE_AUTO,
		.reg_offset = BCRM_EXPOSURE_AUTO_8RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_8,
		.type = V4L2_CTRL_TYPE_MENU,
		.flags = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_ABSOLUTE,
		.attr = AV_ATTR_EXPOSURE_ABS,
		.min_offset = BCRM_EXPOSURE_TIME_MIN_64R,
		.max_offset = BCRM_EXPOSURE_TIME_MAX_64R,
		.reg_offset = BCRM_EXPOSURE_TIME_64RW,
		.step_offset = BCRM_EXPOSURE_TIME_INC_64R,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_AUTO_MIN,
		.attr = AV_ATTR_EXPOSURE_AUTO_MIN,
		.reg_offset = BCRM_EXPOSURE_AUTO_MIN_64RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_AUTO_MAX,
		.attr = AV_ATTR_EXPOSURE_AUTO_MAX,
		.reg_offset = BCRM_EXPOSURE_AUTO_MAX_64RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_AUTOGAIN_MIN,
		.attr = AV_ATTR_AUTOGAIN_MIN,
		.reg_offset = BCRM_GAIN_AUTO_MIN_64RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	{
		.id = V4L2_CID_AUTOGAIN_MAX,
		.attr = AV_ATTR_AUTOGAIN_MAX,
		.reg_offset = BCRM_GAIN_AUTO_MAX_64RW,
		.reg_size = AV_CAM_REG_SIZE,
		.data_size = AV_CAM_DATA_SIZE_64,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = 0,
	},
	// {
	// 	.id = V4L2_CID_EXPOSURE_ACTIVE_LINE_MODE,
	// 	.attr = AV_ATTR_EXPOSURE_ACTIVE_LINE_MODE,
	// 	.reg_offset = BCRM_EXPOSURE_ACTIVE_LINE_MODE_8RW,
	// 	.reg_size = AV_CAM_REG_SIZE,
	// 	.data_size = AV_CAM_DATA_SIZE_8,
	// 	.type = V4L2_CTRL_TYPE_BOOLEAN,
	// 	.flags = 0,
	// },
	// {
	// 	.id = V4L2_CID_EXPOSURE_ACTIVE_LINE_SELECTOR,
	// 	.attr = AV_ATTR_EXPOSURE_ACTIVE_LINE_SELECTOR,
	// 	.reg_offset = BCRM_EXPOSURE_ACTIVE_LINE_SELECTOR_8RW,
	// 	.reg_size = AV_CAM_REG_SIZE,
	// 	.data_size = AV_CAM_DATA_SIZE_8,
	// 	.type = V4L2_CTRL_TYPE_INTEGER,
	// 	.flags = 0,
	// },
	// {
	// 	.id = V4L2_CID_EXPOSURE_ACTIVE_INVERT,
	// 	.attr = AV_ATTR_EXPOSURE_ACTIVE_INVERT,
	// 	.reg_size = AV_CAM_REG_SIZE,
	// 	.data_size = AV_CAM_DATA_SIZE_8,
	// 	.type = V4L2_CTRL_TYPE_BOOLEAN,
	// 	.flags = 0,
	// },
	// {
	// 	.id = V4L2_CID_LINK_FREQ,
	// 	.attr = AV_ATTR_LINK_FREQ,
	// 	.min_offset = BCRM_CSI2_CLOCK_MIN_32R,
	// 	.max_offset = BCRM_CSI2_CLOCK_MAX_32R,
	// 	.reg_offset = BCRM_CSI2_CLOCK_32RW,
	// 	.reg_size = AV_CAM_REG_SIZE,
	// 	.data_size = AV_CAM_DATA_SIZE_32,
	// 	.type = V4L2_CTRL_TYPE_INTEGER,
	// 	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	// },
	// {
	// 	.id = V4L2_CID_TEMPERATURE,
	// 	.attr = AV_ATTR_TEMPERATURE,
	// 	.reg_offset = BCRM_DEVICE_TEMPERATURE_32R,
	// 	.reg_size = AV_CAM_REG_SIZE,
	// 	.data_size = AV_CAM_DATA_SIZE_32,
	// 	.type = V4L2_CTRL_TYPE_INTEGER,
	// 	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	// },
};

#endif /* ALVIUM_REGS_H */
