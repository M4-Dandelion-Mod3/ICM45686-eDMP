#include <stdio.h>
#include "inv_imu_defs.h"
#include "inv_imu_driver.h"
#include "inv_imu_edmp.h"
#include "inv_imu_edmp_defs.h"
#include "inv_imu_driver_advanced.h"
#include "read_aux_data_mode.h"
#include "main.h"
#include "tim.h"

#define ICM_USE_HARD_SPI
//#define ICM_USE_I2C

#if defined(ICM_USE_HARD_SPI)
#include "spi.h"
#elif defined(ICM_USE_I2C)
#include "myiic.h"
#define ICM_I2C_ADDR 0x69
#endif

#define UI_I2C  0 /**< identifies I2C interface. */
#define UI_SPI4 1 /**< identifies 4-wire SPI interface. */

#define	SPI_IMU_CS(x) if(x)HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);else HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);  //选中IMU	

#define INV_MSG(level,msg, ...) 	      // printf("%d," msg "\r\n", __LINE__, ##__VA_ARGS__)
 
 /*
   * WOM threshold value in mg.
   * 1g/256 resolution (wom_th = mg * 256 / 1000)
   */
#define DEFAULT_WOM_THS_MG 52 >> 2 // 52 mg
static uint32_t          odr_us; /* Store current ODR for accel/EDMP */


static inv_imu_device_t  imu_dev; /* Driver structure */

static uint8_t discard_accel_samples; /* Indicates how many accel samples should be discarded */
static uint8_t discard_gyro_samples; /* Indicates how many gyro samples should be discarded */

extern uint8_t pedometer_en; /* Indicates pedometer state */
extern uint8_t smd_en; /* Indicates SMD state */
extern uint8_t tilt_en; /* Indicates tilt state */
extern uint8_t r2w_en; /* Indicates R2W state */
extern uint8_t tap_en; /* Indicates tap state */
extern uint8_t ff_en; /* Indicates freefall state */
extern uint8_t lowg_en; /* Indicates lowg state */
extern uint8_t highg_en; /* Indicates highg state */
extern uint8_t power_save_en; /* Indicates power save mode state */

int si_print_error_if_any(int rc);
//#define SI_CHECK_RC(rc)                                                                            \
//	do {                                                                                           \
//		if (si_print_error_if_any(rc)) {                                                           \
//			INV_MSG(INV_MSG_LEVEL_ERROR, "At %s (line %d)", __FILE__, __LINE__);                   \
//			HAL_Delay(100); ;                                                                   \
//			return rc;                                                                             \
//		}                                                                                          \
//	} while (0)

/*
 * Error codes
 */
int si_print_error_if_any(int rc)
{
	if (rc != 0) {
		switch (rc) {
		case INV_IMU_ERROR:
			// printf("Unspecified error (%d)", rc);
			break;
		case INV_IMU_ERROR_TRANSPORT:
			// printf("Error occurred at transport level (%d)", rc);
			break;
		case INV_IMU_ERROR_TIMEOUT:
			// printf("Action did not complete in the expected time window (%d)",rc);
			break;
		case INV_IMU_ERROR_BAD_ARG:
			// printf("Invalid argument provided (%d)", rc);
			break;
		case INV_IMU_ERROR_EDMP_BUF_EMPTY:
			// printf("EDMP buffer is empty (%d)", rc);
			break;
		default:
			// printf("Unknown error (%d)", rc);
			break;
		}
	}

	return rc;
}
/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len)
{
#if defined(ICM_USE_HARD_SPI)
    reg |= 0x80;
    SPI_IMU_CS(0);
    /* 写入要读的寄存器地址 */
    SPI2_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    while(len)
	{
		*buf = SPI2_ReadWriteByte(0x00);
		len--;
		buf++;
	}
    SPI_IMU_CS(1);
#elif defined(ICM_USE_I2C)
	IICreadBytes(ICM_I2C_ADDR, reg, len, buf);
#endif
	return 0;
}

static uint8_t io_write_reg(uint8_t reg, uint8_t value)
{
#if defined(ICM_USE_HARD_SPI)
    SPI_IMU_CS(0);
    /* 写入要读的寄存器地址 */
    /* 写入要读的寄存器地址 */
    SPI2_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    SPI2_ReadWriteByte(value);
    SPI_IMU_CS(1);
#elif defined(ICM_USE_I2C)
	IICwriteBytes(ICM42688_ADDRESS, reg, 1, &value);
#endif
    return 0;
}

static int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len)
{
	int rc;
	
    SPI_IMU_CS(0);
	for (uint32_t i = 0; i < len; i++) 
	{
		rc = io_write_reg(reg + i, buf[i]);
		if (rc)
			return rc;
	}
	
    SPI_IMU_CS(1);
	return 0;
}

/* Initializes IMU device and apply configuration. */



int configure_and_enable_edmp_algo()
{
	int                            rc = 0;
	dmp_ext_sen_odr_cfg_apex_odr_t dmp_odr;
	accel_config0_accel_odr_t      accel_odr;
	inv_imu_edmp_apex_parameters_t apex_parameters;
	inv_imu_edmp_int_state_t       apex_int_config;
	uint8_t misc;
	/* Configure ODR depending on which feature is enabled */
	if (ff_en || lowg_en || highg_en) {
		/* 800 Hz */
		odr_us    = 1250;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_800_HZ;
	} else if (tap_en) {
		/* 400 Hz */
		odr_us    = 2500;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
	} else if (r2w_en) {
		/* 100 Hz */
		odr_us    = 10000;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;
	} else {
		/* 50 Hz */
		odr_us    = 20000;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;
	}
	
	inv_imu_read_reg(&imu_dev, IPREG_MISC, 1, &misc);
	/* Set EDMP ODR */
	rc |= inv_imu_edmp_set_frequency(&imu_dev, dmp_odr);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu_dev, accel_odr);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
	SI_CHECK_RC(rc);

	/* Select WUOSC clock to have accel in ULP (lowest power mode) */
	rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);
	SI_CHECK_RC(rc);

	/* Set AVG to 1x */
	rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);
	SI_CHECK_RC(rc);

	/* Ensure all DMP features are disabled before running init procedure */
#if INV_IMU_USE_BASIC_SMD
	/* Disable WOM required for Basic SMD */
	rc |= inv_imu_adv_disable_wom(&imu_dev);
#endif
	rc |= inv_imu_edmp_disable_pedometer(&imu_dev);
	rc |= inv_imu_edmp_disable_smd(&imu_dev);
	rc |= inv_imu_edmp_disable_tilt(&imu_dev);
	rc |= inv_imu_edmp_disable_r2w(&imu_dev);
	rc |= inv_imu_edmp_disable_tap(&imu_dev);
	rc |= inv_imu_edmp_disable_ff(&imu_dev);
	rc |= inv_imu_edmp_disable(&imu_dev);
	SI_CHECK_RC(rc);

	/* Request DMP to re-initialize APEX */
	rc |= inv_imu_edmp_recompute_apex_decimation(&imu_dev);
	SI_CHECK_RC(rc);

	/* Configure APEX parameters */
	rc |= inv_imu_edmp_get_apex_parameters(&imu_dev, &apex_parameters);
	apex_parameters.r2w_sleep_time_out = 6400; /* 6.4 s */
	if (tap_en) {
		/* TAP supports 400 Hz and 800 Hz ODR */
		if (dmp_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ) {
			apex_parameters.tap_tmax             = TAP_TMAX_800HZ;
			apex_parameters.tap_tmin             = TAP_TMIN_800HZ;
			apex_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_800HZ;
		} else {
			apex_parameters.tap_tmax             = TAP_TMAX_400HZ;
			apex_parameters.tap_tmin             = TAP_TMIN_400HZ;
			apex_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_400HZ;
		}
	}
	apex_parameters.power_save_en = power_save_en;
	if (power_save_en) {
		rc |= inv_imu_adv_configure_wom(&imu_dev, DEFAULT_WOM_THS_MG, DEFAULT_WOM_THS_MG,
		                                DEFAULT_WOM_THS_MG, TMST_WOM_CONFIG_WOM_INT_MODE_ANDED,
		                                TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
		rc |= inv_imu_adv_enable_wom(&imu_dev);
	} else {
		rc |= inv_imu_adv_disable_wom(&imu_dev);
	}
	rc |= inv_imu_edmp_set_apex_parameters(&imu_dev, &apex_parameters);
	SI_CHECK_RC(rc);

	/* Set accel in low-power mode if ODR slower than 800 Hz, otherwise in low-noise mode */
	if (odr_us <= 1250 /* 800 Hz and faster */)
		rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
	else
		rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
	SI_CHECK_RC(rc);

	/* Wait for accel startup time */
	delay_us(ACC_STARTUP_TIME_US);

	/* Disable all APEX interrupt and enable only the one we need */
	memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));

	/* Enable requested features */
	if (pedometer_en) {
		rc |= inv_imu_edmp_enable_pedometer(&imu_dev);
		apex_int_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
		apex_int_config.INV_STEP_DET      = INV_IMU_ENABLE;
	}

	if (smd_en) {
#if INV_IMU_USE_BASIC_SMD
		/* Configure and enable WOM required for Basic SMD */
		rc |= inv_imu_adv_configure_wom(&imu_dev, 6, 6, 6, // 6 * 1/256mg ~= 23.44mg for each axis
		                                TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
		                                TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
		rc |= inv_imu_adv_enable_wom(&imu_dev);
#endif
		rc |= inv_imu_edmp_enable_smd(&imu_dev);
		apex_int_config.INV_SMD = INV_IMU_ENABLE;
	}

	if (tilt_en) {
		rc |= inv_imu_edmp_enable_tilt(&imu_dev);
		apex_int_config.INV_TILT_DET = INV_IMU_ENABLE;
	}

	if (r2w_en) {
		rc |= inv_imu_edmp_enable_r2w(&imu_dev);
		apex_int_config.INV_R2W       = INV_IMU_ENABLE;
		apex_int_config.INV_R2W_SLEEP = INV_IMU_ENABLE;
	}

	if (tap_en) {
		rc |= inv_imu_edmp_enable_tap(&imu_dev);
		apex_int_config.INV_TAP = INV_IMU_ENABLE;
	}

	if (ff_en) {
		rc |= inv_imu_edmp_enable_ff(&imu_dev);
		apex_int_config.INV_FF = INV_IMU_ENABLE;
	}

	if (lowg_en) {
		rc |= inv_imu_edmp_enable_ff(&imu_dev);
		apex_int_config.INV_LOWG = INV_IMU_ENABLE;
	}

	if (highg_en) {
		rc |= inv_imu_edmp_enable_ff(&imu_dev);
		apex_int_config.INV_HIGHG = INV_IMU_ENABLE;
	}
	

	/* Apply interrupt configuration */
	rc |= inv_imu_edmp_set_config_int_apex(&imu_dev, &apex_int_config);
	SI_CHECK_RC(rc);

	inv_imu_read_reg(&imu_dev, IPREG_MISC, 1, &misc);
	/* Enable EDMP if at least one feature is enabled */
	if (pedometer_en || smd_en || tilt_en || r2w_en || tap_en || ff_en || lowg_en || highg_en)
		rc |= inv_imu_edmp_enable(&imu_dev);

	SI_CHECK_RC(rc);

	return rc;
}

int inv_imu_edmp_init_apex_my(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	uint8_t         value;
	edmp_apex_en0_t save_edmp_apex_en0;
	edmp_apex_en1_t save_edmp_apex_en1;
	edmp_apex_en0_t edmp_apex_en0 = { 0 };
	edmp_apex_en1_t edmp_apex_en1 = { 0 };
	reg_host_msg_t  reg_host_msg;
	fifo_sram_sleep_t  fifo_sram_sleep;
	
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);
	if (save_edmp_apex_en1.edmp_enable)
		return INV_IMU_ERROR;
	
	uint8_t vec[6] = {0x00,0x40,0x04,0x40,0x08,0x40};
    status |= inv_imu_write_reg(s, 0xA24F, 6, vec);
    status |= inv_imu_write_reg(s, 0xA255, 1, (uint8_t[]){0x05}); /* SP start */
    if (status) return status;
	
	value = 0x02<<2;
	status |= inv_imu_write_reg(s,0x2B,1,&value);
	
	
	status |= inv_imu_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 0x03;
	status |= inv_imu_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	
	/* Clear SRAM */
	value = 0x00;
	for (int i = 0; i < EDMP_ROM_DATA_SIZE; i++)
	status |= inv_imu_write_sram(s, (uint32_t)EDMP_RAM_BASE + i, 1, &value);
	
	value = 0x02;
	status |= inv_imu_write_reg(s,0x2A,1,&value);
	
	HAL_Delay(10);
	
	status |= inv_imu_write_reg(s,0xA271,1,(uint8_t *)0x00);
	
	value |= 0x40;
	status |= inv_imu_write_reg(s,0x2A,1,&value);
	
	status |= inv_imu_write_reg(s,0x73,1,(uint8_t *)0x20);
	
	delay_us(200);
	status |= inv_imu_edmp_wait_for_idle(s);
	
	/* Reset states */
	status |= inv_imu_edmp_mask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ON_DEMAND_MASK);

	/* Restore original DMP state, with DMP necessarily disabled as it was checked at the beginning of this function */
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);

	status |= inv_imu_edmp_unmask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);

	return status;
	
}
int setup_imu_apex(void)
{
    int rc = INV_IMU_OK;
    uint8_t whoami = 0;

    /* 1. 绑定底层 SPI/I2C 接口 */
    imu_dev.transport.read_reg   = icm45686_read_regs;
    imu_dev.transport.write_reg  = icm45686_write_regs;
    imu_dev.transport.serif_type = UI_SPI4;
    imu_dev.transport.sleep_us   = delay_us;

    /* 2. 检查 WHOAMI */
    rc = inv_imu_get_who_am_i(&imu_dev, &whoami);
    if (rc || whoami != INV_IMU_WHOAMI) {
//        printf("WHOAMI mismatch: read 0x%02X, expected 0x%02X\n", whoami, INV_IMU_WHOAMI);
        return -1;
    }

    /* 3. 传感器基础配置 */
    rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G);
    rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
    rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_800_HZ);
    rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_800_HZ);
    rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
    rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);

    /* 4. 设置加速度计低功耗时钟源为 RCOSC（eDMP 运行所需） */
    rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);

    /* 5. 打开加速度计和陀螺仪 LN 模式 */
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);

    if (rc) return rc;

    /* 6. 禁用所有 eDMP 功能，清理状态 */
    rc |= inv_imu_edmp_disable_pedometer(&imu_dev);
    rc |= inv_imu_edmp_disable_smd(&imu_dev);
    rc |= inv_imu_edmp_disable_tilt(&imu_dev);
    rc |= inv_imu_edmp_disable_r2w(&imu_dev);
    rc |= inv_imu_edmp_disable_tap(&imu_dev);
    rc |= inv_imu_edmp_disable_ff(&imu_dev);
    rc |= inv_imu_edmp_disable(&imu_dev);

    /* 7. 初始化 eDMP/APEX 内核 ++++++++++++++++++++++++++*/
    rc |= inv_imu_edmp_init_apex_my(&imu_dev);

    /* 8. 获取并调整 APEX 参数 */
    inv_imu_edmp_apex_parameters_t apex_params;
    rc |= inv_imu_edmp_get_apex_parameters(&imu_dev, &apex_params);

    // 自由落体检测参数（根据 TDK 推荐设置）
	apex_params.lowg_peak_th       = 180;    // mg 单位下的低G阈值 (~180 mg)
	apex_params.lowg_peak_th_hyst  = 50;     // 滞后区 (~50 mg)
	apex_params.lowg_time_th       = 20;     // 检测窗口时间 (20 * 1.25ms = 25ms)
	apex_params.ff_min_duration    = 150;    // 最短自由落体持续时间 (约 150ms)
	apex_params.ff_max_duration    = 1000;   // 最长自由落体持续时间 (1s)
	apex_params.ff_debounce_duration = 2000; // 事件防抖时间 (2s)
    // SMD 运动检测参数
    apex_params.smd_sensitivity = 2;         // 0~3 灵敏度等级（2=中等）

    // 可选省电
    apex_params.power_save_en = 1;

    rc |= inv_imu_edmp_set_apex_parameters(&imu_dev, &apex_params);

    /* 9. 配置中断 */
    inv_imu_int_state_t int_cfg;
    memset(&int_cfg, INV_IMU_DISABLE, sizeof(int_cfg));
    int_cfg.INV_EDMP_EVENT = INV_IMU_ENABLE;   // APEX 总中断
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_cfg);

    /* 10. 启用功能：自由落体 & 运动检测 */
    rc |= inv_imu_edmp_enable_ff(&imu_dev);    // 自由落体
    rc |= inv_imu_edmp_enable_smd(&imu_dev);   // 运动检测

    /* 11. 设置中断掩码：只开 FF 与 SMD */
    inv_imu_edmp_int_state_t apex_int_cfg;
    memset(&apex_int_cfg, INV_IMU_DISABLE, sizeof(apex_int_cfg));
    apex_int_cfg.INV_FF  = INV_IMU_ENABLE;     // 自由落体事件
    apex_int_cfg.INV_SMD = INV_IMU_ENABLE;     // 运动事件
    rc |= inv_imu_edmp_set_config_int_apex(&imu_dev, &apex_int_cfg);

    /* 12. 启用 eDMP 引擎 */
    rc |= inv_imu_edmp_enable(&imu_dev);

    if (rc == 0){
		return rc;
		
	}
	else
		while(1);
	
}

int setup_imu_apex1(void)
{
	int                      rc     = 0;
	uint8_t                  whoami = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t      int_config;
	uint8_t tmp;

	/* 1. 绑定底层读写 */
	imu_dev.transport.read_reg   = icm45686_read_regs;
	imu_dev.transport.write_reg  = icm45686_write_regs;
	imu_dev.transport.serif_type = UI_SPI4;
	imu_dev.transport.sleep_us   = delay_us;

	/* 2. SPI 片脚初始化（你现有代码） */
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_Initure.Pin   = SPI2_NSS_Pin;          // PB12
	GPIO_Initure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_Initure.Pull  = GPIO_PULLUP;
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_Initure);
	SPI_IMU_CS(1);                              // 先拉高
	delay_us(3000);                             // 3 ms 上电稳定


	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_14NS;
		rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t*)&drive_config0);
		SI_CHECK_RC(rc);
		delay_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	/* Check whoami */
	rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
	SI_CHECK_RC(rc);
	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return -1;
	}

	rc |= inv_imu_soft_reset(&imu_dev);
	SI_CHECK_RC(rc);
	

	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
	SI_CHECK_RC(rc);

	/* Interrupts configuration: Enable only EDMP interrupt */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_EDMP_EVENT = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);
	
	rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G);
//	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
//	rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_800_HZ);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu_dev,IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);// IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER);IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4
//	rc |= inv_imu_set_gyro_ln_bw(&imu_dev,IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);// IPREG_SYS1_REG_172_GYRO_UI_LPFBW_NO_FILTER);IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4
	SI_CHECK_RC(rc);
	
	/* Sensor registers are not available in ULP, so select RCOSC clock to use LP mode. */
	rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);
	
//	rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
	rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
	SI_CHECK_RC(rc);

	uint8_t val;
	inv_imu_read_reg(&imu_dev, 0x10, 1, &val);
//	printf("PWR_MGMT0 = 0x%02X (ACCEL_MODE=%d)\n", val, val & 0x03);

	// 读取 RTC_MODE（RTC_CONFIG[5]）
	inv_imu_read_reg(&imu_dev, 0x26, 1, &val);  // RTC_CONFIG 地址 = 0xA400 + 0x26
//	printf("RTC_CONFIG = 0x%02X (RTC_MODE=%d)\n", val, (val >> 5) & 0x01);

	inv_imu_read_reg(&imu_dev,0xA258,1,(uint8_t *)&val);

	// 步骤3：设置 ACCEL_LP_CLK_SEL = 1 (bit4)
	val |= 0x10;

	// 步骤4：写回
	inv_imu_write_reg(&imu_dev,0xA258,1, &val);
	
	inv_imu_read_reg(&imu_dev,0xA258,1,(uint8_t *)&val);
	
	
	
	
	/* Set power modes */

	/* 8. APEX 初始化 */
	rc |= inv_imu_edmp_init_apex(&imu_dev);
	rc |= configure_and_enable_edmp_algo();
	SI_CHECK_RC(rc);

	return rc;
}
int bsp_IcmGetRawData_f(float accel_mg[3], float gyro_dps[3], float *temp_degc)
{
	int rc = 0;
	inv_imu_sensor_data_t d;
	
	rc |= inv_imu_get_register_data(&imu_dev, &d);
	SI_CHECK_RC(rc);
	
	accel_mg[0] = (float)((d.accel_data[0] * 4 /* mg */) / 32.768);
	accel_mg[1] = (float)((d.accel_data[1] * 4 /* mg */) / 32.768);
	accel_mg[2] = (float)((d.accel_data[2] * 4 /* mg */) / 32.768);
	gyro_dps[0] = (float)((d.gyro_data[0] * 1000 /* dps */) / 32768.0);
	gyro_dps[1] = (float)((d.gyro_data[1] * 1000 /* dps */) / 32768.0);
	gyro_dps[2] = (float)((d.gyro_data[2] * 1000 /* dps */) / 32768.0);
	*temp_degc  = (float)(25 + (d.temp_data / 128.0));
	return 0;
}
int bsp_IcmGetRawData(int16_t accel_mg[3], int16_t gyro_dps[3], int16_t *temp_degc)
{
	int rc = 0;
	inv_imu_sensor_data_t d;
	
	rc |= inv_imu_get_register_data(&imu_dev, &d);
	SI_CHECK_RC(rc);
	
	accel_mg[0] = d.accel_data[0];
	accel_mg[1] = d.accel_data[1];
	accel_mg[2] = d.accel_data[2];
	gyro_dps[0] = d.gyro_data[0];
	gyro_dps[1] = d.gyro_data[1];
	gyro_dps[2] = d.gyro_data[2];
	*temp_degc  = d.temp_data;
	return 0;
}
uint8_t inv_imu_set_ag_sleep()
{
	uint8_t rc = 0;
	rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_OFF);
	rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_OFF);
	return rc;
}

uint8_t inv_imu_wake_up()
{
	uint8_t rc = 0;
	rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
	rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
	return rc;
}
	
int setup_imu(int use_ln, int accel_en, int gyro_en)
{
	int                      rc     = 0;
	uint8_t                  whoami = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t      int_config;
#if defined(ICM_USE_HARD_SPI)
	/* Init transport layer */
	imu_dev.transport.read_reg   = icm45686_read_regs;
	imu_dev.transport.write_reg  = icm45686_write_regs;
	imu_dev.transport.serif_type = UI_SPI4;
	imu_dev.transport.sleep_us   = delay_us;
	GPIO_InitTypeDef GPIO_Initure;
			
	__HAL_RCC_GPIOB_CLK_ENABLE();           //使能GPIOA时钟
	
	//PA4
	GPIO_Initure.Pin=SPI2_NSS_Pin;            //PB12
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //快速         
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //初始化
	
	SPI_IMU_CS(1);			                //SPI FLASH不选中
//	SPI1_Init();		   			        //初始化SPI
//	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8); 
			
#endif
	/* Wait 3 ms to ensure device is properly supplied  */
	HAL_Delay(100);

	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		SI_CHECK_RC(rc);
		delay_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	/* Check whoami */
	rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
	SI_CHECK_RC(rc);
	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return -1;
	}

	rc |= inv_imu_soft_reset(&imu_dev);
	SI_CHECK_RC(rc);


	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
	SI_CHECK_RC(rc);

	/* Interrupts configuration */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_UI_DRDY = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* Set FSR */
	rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G);
	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_6400_HZ);
	rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_6400_HZ);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu_dev,IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);// IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER);IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4
	rc |= inv_imu_set_gyro_ln_bw(&imu_dev,IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);// IPREG_SYS1_REG_172_GYRO_UI_LPFBW_NO_FILTER);IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4
	SI_CHECK_RC(rc);

	/* Sensor registers are not available in ULP, so select RCOSC clock to use LP mode. */
	rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
	SI_CHECK_RC(rc);

	/* Set power modes */
	if (use_ln) {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
	} else {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LP);
	}

	/* Discard N samples at 50Hz to ignore samples at sensor enabling time */
	if (accel_en)
		discard_accel_samples = (ACC_STARTUP_TIME_US / 20000) + 1;
	if (gyro_en)
		discard_gyro_samples = (GYR_STARTUP_TIME_US / 20000) + 1;

	SI_CHECK_RC(rc);

	return rc;
}

