#include <linux/module.h>
#include "cam_sensor_cmn_header.h"
#include "cam_actuator_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#include "oplus_cam_actuator_core.h"

#define ak7375c_write_slave 0xE8
#define ak7375c_read_slave 0xE9
#define check_store_ak7375c_add 0x4B
#define check_ak7375c_PID_size 24
#define update_ak7375c_PID_size 22

static uint32_t change_ak7375c_setting_mode[4] = {0xAE, 0x3B, 0x00, 0x0};

static uint32_t release_ak7375c_setting_mode[4] = {0xAE, 0x00, 0x00, 0x0};

static uint32_t store_ak7375c_arr[4] = {0x03, 0x02, 0xD8, 0x0}; // store delay 216ms

static uint32_t update_ak7375c_PID_arr[update_ak7375c_PID_size][4] = {
	{0x10, 0x24, 0x00, 0x0},
	{0x11, 0x3C, 0x00, 0x0},
	{0x12, 0x54, 0x00, 0x0},
	{0x13, 0x56, 0x00, 0x0},
	{0x14, 0x21, 0x00, 0x0},
	{0x15, 0x00, 0x00, 0x0},
	{0x16, 0x21, 0x00, 0x0},
	{0x17, 0x40, 0x00, 0x0},
	{0x18, 0xDB, 0x00, 0x0},
	{0x1A, 0x00, 0x00, 0x0},
	{0x1B, 0x5C, 0x00, 0x0},
	{0x1C, 0xB0, 0x00, 0x0},
	{0x1D, 0x92, 0x00, 0x0},
	{0x1E, 0x50, 0x00, 0x0},
	{0x1F, 0x52, 0x00, 0x0},
	{0x20, 0x01, 0x00, 0x0},
	{0x21, 0x02, 0x00, 0x0},
	{0x22, 0x04, 0x00, 0x0},
	{0x23, 0x28, 0x00, 0x0},
	{0x24, 0xFF, 0x00, 0x0},
	{0x25, 0x1E, 0x00, 0x0},
	{0x26, 0x58, 0x00, 0x0}
};

static uint32_t check_ak7375c_PID_arr[check_ak7375c_PID_size][4] = {
	{0x0A, 0x89, 0x00, 0x0},
	{0x0B, 0x88, 0x00, 0x0},
	{0x10, 0x24, 0x00, 0x0},
	{0x11, 0x3C, 0x00, 0x0},
	{0x12, 0x54, 0x00, 0x0},
	{0x13, 0x56, 0x00, 0x0},
	{0x14, 0x21, 0x00, 0x0},
	{0x15, 0x00, 0x00, 0x0},
	{0x16, 0x21, 0x00, 0x0},
	{0x17, 0x40, 0x00, 0x0},
	{0x18, 0xDB, 0x00, 0x0},
	{0x1A, 0x00, 0x00, 0x0},
	{0x1B, 0x5C, 0x00, 0x0},
	{0x1C, 0xB0, 0x00, 0x0},
	{0x1D, 0x92, 0x00, 0x0},
	{0x1E, 0x50, 0x00, 0x0},
	{0x1F, 0x52, 0x00, 0x0},
	{0x20, 0x01, 0x00, 0x0},
	{0x21, 0x02, 0x00, 0x0},
	{0x22, 0x04, 0x00, 0x0},
	{0x23, 0x28, 0x00, 0x0},
	{0x24, 0xFF, 0x00, 0x0},
	{0x25, 0x1E, 0x00, 0x0},
	{0x26, 0x58, 0x00, 0x0}
};

void cam_actuator_i2c_modes_util_oem(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list)
{
	uint32_t value;
	if (i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x0204)
	{
		value = (i2c_list->i2c_settings.reg_setting[0].reg_data & 0xFF00) >> 8;
		i2c_list->i2c_settings.reg_setting[0].reg_data =
		((i2c_list->i2c_settings.reg_setting[0].reg_data & 0xFF) << 8) | value;
		CAM_DBG(CAM_ACTUATOR,"new value %d", i2c_list->i2c_settings.reg_setting[0].reg_data);
	}
}

void cam_actuator_poll_setting_update(struct cam_actuator_ctrl_t *a_ctrl) {

        struct i2c_settings_list *i2c_list = NULL;

        a_ctrl->is_actuator_ready = true;
        memset(&(a_ctrl->poll_register), 0, sizeof(struct cam_sensor_i2c_reg_array));
        list_for_each_entry(i2c_list,
                &(a_ctrl->i2c_data.init_settings.list_head), list) {
                if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
                        a_ctrl->poll_register.reg_addr = i2c_list->i2c_settings.reg_setting[0].reg_addr;
                        a_ctrl->poll_register.reg_data = i2c_list->i2c_settings.reg_setting[0].reg_data;
                        a_ctrl->poll_register.data_mask = i2c_list->i2c_settings.reg_setting[0].data_mask;
                        a_ctrl->poll_register.delay = 100; //i2c_list->i2c_settings.reg_setting[0].delay; // The max delay should be 100
                        a_ctrl->addr_type = i2c_list->i2c_settings.addr_type;
                        a_ctrl->data_type = i2c_list->i2c_settings.data_type;
                }
        }
}

void cam_actuator_poll_setting_apply(struct cam_actuator_ctrl_t *a_ctrl) {
        int ret = 0;
        if (!a_ctrl->is_actuator_ready) {
                if (a_ctrl->poll_register.reg_addr || a_ctrl->poll_register.reg_data) {
                        ret = camera_io_dev_poll(
                                &(a_ctrl->io_master_info),
                                a_ctrl->poll_register.reg_addr,
                                a_ctrl->poll_register.reg_data,
                                a_ctrl->poll_register.data_mask,
                                a_ctrl->addr_type,
                                a_ctrl->data_type,
                                a_ctrl->poll_register.delay);
                        if (ret < 0) {
                                CAM_ERR(CAM_ACTUATOR,"i2c poll apply setting Fail: %d, is_actuator_ready %d", ret, a_ctrl->is_actuator_ready);
                        } else {
                                CAM_DBG(CAM_ACTUATOR,"is_actuator_ready %d, ret %d", a_ctrl->is_actuator_ready, ret);
                        }
                        a_ctrl->is_actuator_ready = true; //Just poll one time
                }
        }
}

#define UPDATE_REG_SIZE 64
static uint32_t update_reg_arr[64][4] = {
	{0x00, 0x33, 0x00, 0x0},
	{0x01, 0x08, 0x00, 0x0},
	{0x02, 0x48, 0x00, 0x0},
	{0x03, 0x40, 0x00, 0x0},
	{0x04, 0x4B, 0x00, 0x0},
	{0x05, 0x60, 0x00, 0x0},
	{0x06, 0xEE, 0x00, 0x0},
	{0x07, 0x81, 0x00, 0x0},

	{0x08, 0x6F, 0x00, 0x0},
	{0x09, 0x8D, 0x00, 0x0},
	{0x0A, 0x14, 0x00, 0x0},
	{0x0B, 0x60, 0x00, 0x0},
	{0x0C, 0x56, 0x00, 0x0},
	{0x0D, 0x52, 0x00, 0x0},
	{0x0E, 0x51, 0x00, 0x0},
	{0x0F, 0x48, 0x00, 0x0},

	{0x10, 0x79, 0x00, 0x0},
	{0x11, 0xD0, 0x00, 0x0},
	{0x12, 0x5F, 0x00, 0x0},
	{0x13, 0x18, 0x00, 0x0},
	{0x14, 0x81, 0x00, 0x0},
	{0x15, 0xFF, 0x00, 0x0},
	{0x16, 0x78, 0x00, 0x0},
	{0x17, 0x10, 0x00, 0x0},

	{0x18, 0x00, 0x00, 0x0},
	{0x19, 0x07, 0x00, 0x0},
	{0x1A, 0x06, 0x00, 0x0},
	{0x1B, 0x31, 0x00, 0x0},
	{0x1C, 0xCB, 0x00, 0x0},
	{0x1D, 0x68, 0x00, 0x0},
	{0x1E, 0x7F, 0x00, 0x0},
	{0x1F, 0x7F, 0x00, 0x0},

	{0x20, 0x40, 0x00, 0x0},
	{0x21, 0x0C, 0x00, 0x0},
	{0x22, 0x00, 0x00, 0x0},
	{0x23, 0x22, 0x00, 0x0},
	{0x24, 0xE1, 0x00, 0x0},
	{0x25, 0x80, 0x00, 0x0},
	{0x26, 0x00, 0x00, 0x0},
	{0x27, 0x5B, 0x00, 0x0},

	{0x28, 0x45, 0x00, 0x0},
	{0x29, 0xF7, 0x00, 0x0},
	{0x2A, 0x2F, 0x00, 0x0},
	{0x2B, 0x51, 0x00, 0x0},
	{0x2C, 0xFE, 0x00, 0x0},
	{0x2D, 0xDD, 0x00, 0x0},
	{0x2E, 0xDE, 0x00, 0x0},
	{0x2F, 0xEE, 0x00, 0x0},

	{0x30, 0xFF, 0x00, 0x0},
	{0x31, 0xFF, 0x00, 0x0},
	{0x32, 0xFF, 0x00, 0x0},
	{0x33, 0xFF, 0x00, 0x0},
	{0x34, 0xFF, 0x00, 0x0},
	{0x35, 0xFF, 0x00, 0x0},
	{0x36, 0xFF, 0x00, 0x0},
	{0x37, 0xFF, 0x00, 0x0},

	{0x38, 0x7F, 0x00, 0x0},
	{0x39, 0xFF, 0x00, 0x0},
	{0x3A, 0xFF, 0x00, 0x0},
	{0x3B, 0xFF, 0x00, 0x0},
	{0x3C, 0x20, 0x00, 0x0},
	{0x3D, 0x11, 0x00, 0x0},
	{0x3E, 0x18, 0x00, 0x0},
	{0x3F, 0x01, 0x00, 0x0}

};

int RamWriteByte(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t addr, uint32_t data, unsigned short mdelay)
{
	int32_t rc = 0;
	int retry = 1;
	int i = 0;
	uint32_t read_data = 0;
	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = mdelay,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = mdelay,
	};
	if (a_ctrl == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	read_data = 0;
	rc = camera_io_dev_read(
		&(a_ctrl->io_master_info),
		addr, &read_data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0){
		CAM_ERR(CAM_ACTUATOR, "read error");
		//break;
	}
	CAM_ERR(CAM_ACTUATOR, "before write data addr:%x data:%x ,will write data is %x",addr,read_data,data);

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(a_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			break;
		}
	}

	read_data = 0;
	rc = camera_io_dev_read(
		&(a_ctrl->io_master_info),
		addr, &read_data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0){
		CAM_ERR(CAM_ACTUATOR, "write error");
		//break;
	}

	CAM_ERR(CAM_ACTUATOR, "after write data addr:%x data:%x",addr,read_data);
	return rc;
}

int32_t cam_actuator_update_pid(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	uint32_t IDSEL;
	uint32_t HallCal[5];
	uint32_t Linearity[8];
	int i;
	uint32_t temp = 0;
	uint32_t read_data = 0;

	CAM_ERR(CAM_ACTUATOR, "entry cam_actuator_update_pid");
	if (a_ctrl->need_check_pid &&
		0xE4 >> 1 == a_ctrl->io_master_info.cci_client->sid) {

		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;//read from eeporm
		msleep(10);

		for(i=0; i< UPDATE_REG_SIZE; i++)
		{
			rc = camera_io_dev_read(
					&(a_ctrl->io_master_info),
					update_reg_arr[i][0], &read_data,
					CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
			}
			CAM_ERR(CAM_ACTUATOR, " original addr:%x data:%x",update_reg_arr[i][0],read_data);
		}

		// backup Hall calibration data
		for(i=0; i<5; i++)
		{
			temp = i+0x1E;
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				temp, &HallCal[i],
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
			}
			CAM_ERR(CAM_ACTUATOR, "backup Hall calibration data addr:%x data:%x",temp,HallCal[i]);
		}


		// backup Linearity data
		rc = camera_io_dev_read(
			&(a_ctrl->io_master_info),
			0x06, &IDSEL,
			CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0){
			CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
		}
		CAM_ERR(CAM_ACTUATOR, "read IDSEL data addr:0x06 data:%x",IDSEL);


		if (IDSEL == 0xC3) // check bit1-2 = 0110b?
		{
			// Linearity correction area(128h - 12Fh)
			for(i=0; i<8; i++)
			{
				temp = i+0x28;
				rc = camera_io_dev_read(
					&(a_ctrl->io_master_info),
					temp, &Linearity[i],
					CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
				if (rc < 0){
					CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
				}
				CAM_ERR(CAM_ACTUATOR, "read data addr:%x data:%x",temp,Linearity[i]);
			}

		}else{
			// Linearity correction area(130h - 137h)   IDSEL == 0xC2 || IDSEL == 0xE0
			for(i=0; i<8; i++)
			{
				temp = i+0x30;
				rc = camera_io_dev_read(
					&(a_ctrl->io_master_info),
					temp, &Linearity[i],
					CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
				if (rc < 0){
					CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
				}
				CAM_ERR(CAM_ACTUATOR, "read data addr:%x data:%x",temp,Linearity[i]);
			}
		}

		a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;
		// Write all data to EEPROM
		RamWriteByte(a_ctrl, 0x98, 0xE2, 0);		// Release Write Protect
		RamWriteByte(a_ctrl, 0x99, 0xAF, 0);		// Release Write Protect


		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;

		for(i=0; i< UPDATE_REG_SIZE; i++)
		{
			RamWriteByte(a_ctrl, update_reg_arr[i][0], update_reg_arr[i][1], 20);
			// wait 20 msec
		}

		// recover Hall calibration data
		for(i=0; i<5; i++)
		{
			RamWriteByte(a_ctrl, i+0x1E, HallCal[i], 20);
			// wait 20 msec
		}

		// Linearity correction area(130h - 137h)
		for(i=0; i<8; i++)
		{
			RamWriteByte(a_ctrl, i+0x30, Linearity[i], 20);
			// wait 20 msec
		}

		if(IDSEL == 0XC3){
			RamWriteByte(a_ctrl, 0X06, 0xEF, 20);
		}else{
			RamWriteByte(a_ctrl, 0X06, 0xEE, 20);
		}

		a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;
		RamWriteByte(a_ctrl, 0x98, 0x00, 0);
		RamWriteByte(a_ctrl, 0x99, 0x00, 0);

		RamWriteByte(a_ctrl, 0xE0, 0x01, 0);

		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;

		for(i=0; i< UPDATE_REG_SIZE; i++)
		{
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				update_reg_arr[i][0], &read_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
			}
			CAM_ERR(CAM_ACTUATOR, " final addr:%x data:%x",update_reg_arr[i][0],read_data);
		}
	}

		a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;

	return rc;
}

int32_t cam_actuator_check_firmware(struct cam_actuator_ctrl_t *a_ctrl)
 {
	int32_t cnt = 0;
	int32_t rc = 0;
	uint32_t reg_data = 0;
	if (0xE4 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;
		for(cnt = 0; cnt < UPDATE_REG_SIZE; cnt++)
		{
		   rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				update_reg_arr[cnt][0], &reg_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "read PID data error in step %d:rc %d", cnt, rc);
				break;
			}
			if(update_reg_arr[cnt][0] >= 0x07 && update_reg_arr[cnt][0] <= 0x1d){
				if (reg_data != update_reg_arr[cnt][1]){
					CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, reg_data is %x", cnt, rc,reg_data);
					rc = -1;
					break;
				}
			}else if(update_reg_arr[cnt][0] >= 0x23 && update_reg_arr[cnt][0] <= 0x27){
				if (reg_data != update_reg_arr[cnt][1]){
					CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, reg_data is %x", cnt, rc,reg_data);
					rc = -1;
					break;
				}
			}else if(update_reg_arr[cnt][0] >= 0x38 && update_reg_arr[cnt][0] <= 0x3f){
				if (reg_data != update_reg_arr[cnt][1]){
					CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, reg_data is %x", cnt, rc,reg_data);
					rc = -1;
					break;
				}
			}
		}
	}

	a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;

	return rc;
 }
int32_t cam_AK7375c_actuator_check_pid(struct cam_actuator_ctrl_t *a_ctrl) {
	int32_t cnt = 0;
	int32_t rc = 0;
	uint32_t reg_data = 0;
	if (ak7375c_write_slave >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		a_ctrl->io_master_info.cci_client->sid = ak7375c_read_slave >> 1;
		for (cnt = 0; cnt < check_ak7375c_PID_size; cnt++)
		{
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				check_ak7375c_PID_arr[cnt][0], &reg_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "read PID data error in step %d:rc %d", cnt, rc);
				break;
			}
			if (reg_data != check_ak7375c_PID_arr[cnt][1])
			{
				CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, des_data=0x%x,curr_data=0x%x", cnt, rc,check_ak7375c_PID_arr[cnt][1],reg_data);
				rc = -1;
				break;
			}
		}
		a_ctrl->io_master_info.cci_client->sid = ak7375c_write_slave >> 1;
	}

	return rc;
}

int32_t cam_AK7375c_actuator_update_pid(struct cam_actuator_ctrl_t *a_ctrl) {
	int32_t cnt = 0;
	int32_t i = 0;
	int32_t rc = 0;
	int32_t retry = 2;
	uint32_t reg_data = 0xff;

	CAM_INFO(CAM_ACTUATOR, "entry cam_actuator_update_pid");

/*change to Setting mode*/
	rc = RamWriteByte(a_ctrl, change_ak7375c_setting_mode[0], change_ak7375c_setting_mode[1], change_ak7375c_setting_mode[2]);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "change to Setting mode failed,rc = %d",rc);
		return rc;
	}

/*write PID */
	//msleep(20);
	for (cnt = 0; cnt < update_ak7375c_PID_size; cnt++)
	{
		rc = RamWriteByte(a_ctrl, update_ak7375c_PID_arr[cnt][0], update_ak7375c_PID_arr[cnt][1], update_ak7375c_PID_arr[cnt][2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write PID data error addr0x%x,data0x%x",update_ak7375c_PID_arr[cnt][0],update_ak7375c_PID_arr[cnt][1]);
			break;
		}
	}
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "write PID failed,rc = %d", rc);
		return rc;
	}

/* store instruction for max 2 times*/
	for (i = 0; i < retry; i++)
	{
		rc = RamWriteByte(a_ctrl, store_ak7375c_arr[0], store_ak7375c_arr[1], store_ak7375c_arr[2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "store PID failed %d times", i);
			continue;
		}
		//msleep(200);

		/*check store instruction*/
		a_ctrl->io_master_info.cci_client->sid = ak7375c_read_slave >> 1;
		rc = camera_io_dev_read(
			&(a_ctrl->io_master_info),
			check_store_ak7375c_add, &reg_data,
			CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
		a_ctrl->io_master_info.cci_client->sid = ak7375c_write_slave >> 1;

		if (rc < 0){
			CAM_ERR(CAM_ACTUATOR, "read data error");
			continue;
		}
		/*the 2st bit of data is 0? */
		if (reg_data&0x04) {
			CAM_ERR(CAM_ACTUATOR, "stroe failed %d times ,read data=0x%x",i,reg_data);	//	stroe failed, try 2 tiams to write addr03h data 02h
			rc = -1;
			continue;
		} else {
			break;
		}

	}
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "store PID failed,rc = %d", rc);
		return rc;
	}

/* release setting mode*/
	rc = RamWriteByte(a_ctrl, release_ak7375c_setting_mode[0], release_ak7375c_setting_mode[1], release_ak7375c_setting_mode[2]);
	if (rc < 0){
		CAM_ERR(CAM_ACTUATOR, "release setting mode failed");
	}

	return rc;
}


int32_t oplus_cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl)
{
	int retry = 2;
	int re = 0;
	int rc = 0;
	CAM_INFO(CAM_ACTUATOR, "a_ctrl->io_master_info.cci_client->sid = %x",a_ctrl->io_master_info.cci_client->sid << 1);
	CAM_INFO(CAM_ACTUATOR, "need_check_pid = %d, pid_data_updated = %d",a_ctrl->need_check_pid,a_ctrl->pid_data_updated);

	if (a_ctrl->need_check_pid &&
		0xE4 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		for (re = 0;re < retry; re++){
			rc = cam_actuator_check_firmware(a_ctrl);
			if (rc < 0){
				//if rc is error ,update the pid eeprom
				CAM_ERR(CAM_ACTUATOR, "start store the pid data!");
				rc = cam_actuator_update_pid(a_ctrl);
			}
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "update the pid data error,check the io ctrl!");
			}else {
				break;
			}
		}
	}
	else if (a_ctrl->need_check_pid && !a_ctrl->pid_data_updated &&
		ak7375c_write_slave >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		for (re = 0; re < retry; re++){
			rc = cam_AK7375c_actuator_check_pid(a_ctrl);
			if (rc < 0){
				//if rc is error ,update the pid eeprom
				CAM_INFO(CAM_ACTUATOR, "start to update the pid data!");
				rc = cam_AK7375c_actuator_update_pid(a_ctrl);
				if (rc < 0){
					CAM_ERR(CAM_ACTUATOR, "updated the pid data error!");
				}else {
					CAM_INFO(CAM_ACTUATOR, "updated the pid data succeed!");
					a_ctrl->pid_data_updated = TRUE;
					break;
				}
			} else {
				CAM_INFO(CAM_ACTUATOR, "had updated the pid data succeed!");
				a_ctrl->pid_data_updated = TRUE;
				break;
			}
		}
	}
	return rc;
}

