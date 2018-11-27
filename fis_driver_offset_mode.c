#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#include <nx_type.h>
#include <cfg_type.h>
#include <linux/gpio.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/uaccess.h>

#include <linux/kernel.h>

typedef signed char     VINT8;
typedef unsigned char	VUINT8;
typedef signed short	VINT16;
typedef unsigned short	VUINT16;
typedef unsigned long	VUINT32;
typedef signed long	VINT32;

#define FIS_MODE_AG	           0x00
#define FIS_MODE_AE		    0x01
#define FIS_MODE_ACC	    0x02
#define FIS_MODE_GYR	    0x03
#define ONE_G (9.81f)
#define M_PI (3.14159265358979323846f)
#define AccUnit_ms2             0x00
#define GyrUnit_rads             0x00

static volatile int acc_unit = AccUnit_ms2;//acc_unit
static volatile int gyr_unit = GyrUnit_rads;//gyr_unit

#define _MMA7660_LOW_PASS_FILTER_
#define LowPassFactor 3
#define RawDataLength 4
#define MMA210X_REG_END  (20)

#define MMA7660_Sin30pos 10
#define MMA7660_Sin30neg -10
#define CFG_ETHER_GMAC_PHY_RST_NUM_SEN (PAD_GPIO_C + 10)
#define CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT1 (PAD_GPIO_C + 9)
#define CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT2 (PAD_GPIO_C + 12)
VINT8 Xraw[RawDataLength];
VINT8 Yraw[RawDataLength];
VINT8 Zraw[RawDataLength];

VINT8 RawDataPointer = 0;

VINT8 Xnew8, Ynew8, Znew8;
VINT8 Xavg8, Yavg8, Zavg8;
#ifdef _MMA7660_LOW_PASS_FILTER_
VINT8 Xflt8, Yflt8, Zflt8;
VINT8 Xrsdl, Yrsdl, Zrsdl;
#endif

enum FisImuRegister
{
	/*! \brief FIS device identifier register. */
	FisRegister_WhoAmI, // 0
	/*! \brief FIS hardware revision register. */
	FisRegister_Revision, // 1
	/*! \brief General and power management modes. */
	FisRegister_Ctrl1, // 2
	/*! \brief Accelerometer control. */
	FisRegister_Ctrl2, // 3
	/*! \brief Gyroscope control. */
	FisRegister_Ctrl3, // 4
	/*! \brief Magnetometer control. */
	FisRegister_Ctrl4, // 5
	/*! \brief Data processing settings. */
	FisRegister_Ctrl5, // 6
	/*! \brief AttitudeEngine control. */
	FisRegister_Ctrl6, // 7
	/*! \brief Sensor enabled status. */
	FisRegister_Ctrl7, // 8
	/*! \brief Reserved - do not write. */
	FisRegister_Ctrl8, // 9
	/*! \brief Host command register. */
	FisRegister_Ctrl9,
	/*! \brief Calibration register 1 least significant byte. */
	FisRegister_Cal1_L,
	/*! \brief Calibration register 1 most significant byte. */
	FisRegister_Cal1_H,
	/*! \brief Calibration register 2 least significant byte. */
	FisRegister_Cal2_L,
	/*! \brief Calibration register 2 most significant byte. */
	FisRegister_Cal2_H,
	/*! \brief Calibration register 3 least significant byte. */
	FisRegister_Cal3_L,
	/*! \brief Calibration register 3 most significant byte. */
	FisRegister_Cal3_H,
	/*! \brief Calibration register 4 least significant byte. */
	FisRegister_Cal4_L,
	/*! \brief Calibration register 4 most significant byte. */
	FisRegister_Cal4_H,
	/*! \brief FIFO control register. */
	FisRegister_FifoCtrl,
	/*! \brief FIFO data register. */
	FisRegister_FifoData,
	/*! \brief FIFO status register. */
	FisRegister_FifoStatus,
	/*! \brief Output data overrun and availability. */
	FisRegister_Status0,
	/*! \brief Miscellaneous status register. */
	FisRegister_Status1,
	/*! \brief Sample counter. */
	FisRegister_CountOut,
	/*! \brief Accelerometer X axis least significant byte. */
	FisRegister_Ax_L, //25 -xiaoping
	/*! \brief Accelerometer X axis most significant byte. */
	FisRegister_Ax_H,
	/*! \brief Accelerometer Y axis least significant byte. */
	FisRegister_Ay_L,
	/*! \brief Accelerometer Y axis most significant byte. */
	FisRegister_Ay_H,
	/*! \brief Accelerometer Z axis least significant byte. */
	FisRegister_Az_L,
	/*! \brief Accelerometer Z axis most significant byte. */
	FisRegister_Az_H,
	/*! \brief Gyroscope X axis least significant byte. */
	FisRegister_Gx_L,
	/*! \brief Gyroscope X axis most significant byte. */
	FisRegister_Gx_H,
	/*! \brief Gyroscope Y axis least significant byte. */
	FisRegister_Gy_L,
	/*! \brief Gyroscope Y axis most significant byte. */
	FisRegister_Gy_H,
	/*! \brief Gyroscope Z axis least significant byte. */
	FisRegister_Gz_L,
	/*! \brief Gyroscope Z axis most significant byte. */
	FisRegister_Gz_H,
	/*! \brief Magnetometer X axis least significant byte. */
	FisRegister_Mx_L,
	/*! \brief Magnetometer X axis most significant byte. */
	FisRegister_Mx_H,
	/*! \brief Magnetometer Y axis least significant byte. */
	FisRegister_My_L,
	/*! \brief Magnetometer Y axis most significant byte. */
	FisRegister_My_H,
	/*! \brief Magnetometer Z axis least significant byte. */
	FisRegister_Mz_L,
	/*! \brief Magnetometer Z axis most significant byte. */
	FisRegister_Mz_H,
	/*! \brief Quaternion increment W least significant byte. */
	FisRegister_Q1_L = 45,
	/*! \brief Quaternion increment W most significant byte. */
	FisRegister_Q1_H,
	/*! \brief Quaternion increment X least significant byte. */
	FisRegister_Q2_L,
	/*! \brief Quaternion increment X most significant byte. */
	FisRegister_Q2_H,
	/*! \brief Quaternion increment Y least significant byte. */
	FisRegister_Q3_L,
	/*! \brief Quaternion increment Y most significant byte. */
	FisRegister_Q3_H,
	/*! \brief Quaternion increment Z least significant byte. */
	FisRegister_Q4_L,
	/*! \brief Quaternion increment Z most significant byte. */
	FisRegister_Q4_H,
	/*! \brief Velocity increment X least significant byte. */
	FisRegister_Dvx_L,
	/*! \brief Velocity increment X most significant byte. */
	FisRegister_Dvx_H,
	/*! \brief Velocity increment Y least significant byte. */
	FisRegister_Dvy_L,
	/*! \brief Velocity increment Y most significant byte. */
	FisRegister_Dvy_H,
	/*! \brief Velocity increment Z least significant byte. */
	FisRegister_Dvz_L,
	/*! \brief Velocity increment Z most significant byte. */
	FisRegister_Dvz_H,
	/*! \brief Temperature output. */
	FisRegister_Temperature,
	/*! \brief AttitudeEngine clipping flags. */
	FisRegister_AeClipping,
	/*! \brief AttitudeEngine overflow flags. */
	FisRegister_AeOverflow,
};
enum FisImu_Interrupt
{
	/*! \brief FIS INT1 line. */
	Fis_Int1 = (0 << 6),
	/*! \brief FIS INT2 line. */
	Fis_Int2 = (1 << 6)
};

enum FisImu_InterruptEvent
{
	/*! \brief Positive edge (line went from low to high). */
	FisInt_positiveEdge,
	/*! \brief Line is high. */
	FisInt_high,
	/*! \brief Negative edge (line went from high to low). */
	FisInt_negativeEdge,
	/*! \brief Line is low. */
	FisInt_low
};

//extern void Xkf3_getEulerOrientation(struct Xkf3* xkf, float* euler, enum SensorFusionScenario scenario);
//extern u8 Xkf3_update(struct Xkf3* xkf, struct Xkf3Input const* input);

bool nucInt1Asserted(void)
{
	if (gpio_get_value(CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT1))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool nucInt2Asserted(void)
{
	if (gpio_get_value(CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT2))
	{
		return true;
	}
	else
	{
		return false;
	}
}
/*!
 * \brief Busy waiting implementation of FisImuHal.waitForEvent()
 *
 * This implementation busy waits polling either the FisImuHal.int1Asserted()
 * or FisImuHal.int2Asserted() function to implement the required
 * functionality.
 */
void FisImu_busyWaitForEvent(enum FisImu_Interrupt interrupt, enum FisImu_InterruptEvent event)
{
	bool (*readIntState)(void) = (interrupt == Fis_Int1) ? nucInt1Asserted : nucInt2Asserted;

	switch (event)
	{
		case FisInt_low:
		case FisInt_high:
			{
				const bool requiredState = (event == FisInt_high);
				while (true)
				{
					if (readIntState() == requiredState)
						return;
				}
			}

		case FisInt_positiveEdge:
		case FisInt_negativeEdge:
			{
				const bool requiredState = (event == FisInt_positiveEdge);
				bool prevState = readIntState();
				while (true)
				{
					bool state = readIntState();
					if ((state != prevState) && (state == requiredState))
						return;

					prevState = state;
				}
			}
	}
}

void XYZ_Filter(VINT8 *X, VINT8 *Y, VINT8 *Z);

void XYZ_Filter(VINT8 *X, VINT8 *Y, VINT8 *Z)
{
	VUINT8 i;
	VINT8 temp8_1, temp8_2;
	VINT16 temp16;

	Xnew8 = *X;
	Ynew8 = *Y;
	Znew8 = *Z;

	//printk(KERN_ERR "filting...\n");

#ifdef _MMA7660_LOW_PASS_FILTER_

	if((++RawDataPointer)>=RawDataLength) RawDataPointer = 0;

	Xraw[RawDataPointer] = Xnew8;
	Yraw[RawDataPointer] = Ynew8;
	Zraw[RawDataPointer] = Znew8;

	for(i=0, temp16=0;i<RawDataLength;i++)
	{
		temp16 += Xraw[i];
	}
	Xflt8 = (VINT8)(temp16/RawDataLength);
	for(i=0, temp16=0;i<RawDataLength;i++)
	{
		temp16 += Yraw[i];
	}
	Yflt8 = (VINT8)(temp16/RawDataLength);
	for(i=0, temp16=0;i<RawDataLength;i++)
	{
		temp16 += Zraw[i];
	}
	Zflt8 = (VINT8)(temp16/RawDataLength);

	temp8_1 = Xflt8 - Xavg8;
	temp8_2 = temp8_1 / (1 + LowPassFactor);
	Xavg8 += temp8_2;
	temp8_1 -= temp8_2 * (1 + LowPassFactor);   //Current Residual
	Xrsdl += temp8_1;                           //Overall Residual
	temp8_2 = Xrsdl / (1 + LowPassFactor);
	Xavg8 += temp8_2;
	Xrsdl -= temp8_2 * (1 + LowPassFactor);

	temp8_1 = Yflt8 - Yavg8;
	temp8_2 = temp8_1 / (1 + LowPassFactor);
	Yavg8 += temp8_2;
	temp8_1 -= temp8_2 * (1 + LowPassFactor);
	Yrsdl += temp8_1;
	temp8_2 = Yrsdl / (1 + LowPassFactor);
	Yavg8 += temp8_2;
	Yrsdl -= temp8_2 * (1 + LowPassFactor);

	temp8_1 = Zflt8 - Zavg8;
	temp8_2 = temp8_1 / (1 + LowPassFactor);
	Zavg8 += temp8_2;
	temp8_1 -= temp8_2 * (1 + LowPassFactor);
	Zrsdl += temp8_1;
	temp8_2 = Zrsdl / (1 + LowPassFactor);
	Zavg8 += temp8_2;
	Zrsdl -= temp8_2 * (1 + LowPassFactor);

	*X = Xavg8;
	*Y = Yavg8;
	*Z = Zavg8;

#else
	if((++RawDataPointer)>=RawDataLength) RawDataPointer = 0;

	Xraw[RawDataPointer] = Xnew8;
	Yraw[RawDataPointer] = Ynew8;
	Zraw[RawDataPointer] = Znew8;

	for(i=0, temp16=0;i<RawDataLength;i++)
	{
		temp16 += Xraw[i];
	}
	Xavg8 = (VINT8)(temp16/RawDataLength);
	for(i=0, temp16=0;i<RawDataLength;i++)
	{
		temp16 += Yraw[i];
	}
	Yavg8 = (VINT8)(temp16/RawDataLength);
	for(i=0, temp16=0;i<RawDataLength;i++)
	{
		temp16 += Zraw[i];
	}
	Zavg8 = (VINT8)(temp16/RawDataLength);

	*X = Xavg8;
	*Y = Yavg8;
	*Z = Zavg8;
#endif
}

// device info
#define SENSOR_NAME					"fis210x"
#define SENSOR_I2C_ADDR				0x6a
#define ABSMIN						       -32
#define ABSMAX						       31
#define FUZZ						              1
#define LSG						              21
#define MAX_DELAY					       300

static volatile int current_mode = FIS_MODE_AG;//FIS_MODE_AG


#define MMA2108_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define MMA2108_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


struct fis210_acc{
	s16    x;
	s16    y;
	s16    z;
} ;

struct fis210_gyr{
	s16    x;
	s16    y;
	s16    z;
} ;

static struct fis210_acc acc_raw;
static struct fis210_gyr gyr_raw;
static struct fis210_acc acc_offset;
static struct fis210_gyr gyr_offset;

struct fis210_data {
	struct i2c_client *fis210_client;
	struct input_dev *input;
	atomic_t delay;
	atomic_t enable;
	struct mutex enable_mutex;
	struct delayed_work work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	atomic_t position;
	atomic_t calibrated;
	struct fis210_acc offset;
	atomic_t fuzz;
};

// cfg data : 1 -- used
#define CFG_GSENSOR_USE_CONFIG  0

// calibration file path
#define CFG_GSENSOR_CALIBFILE   "/data/data/com.actions.sensor.calib/files/gsensor_calib.txt"

/*******************************************
 * for xml cfg
 *******************************************/
#define CFG_GSENSOR_ADAP_ID          "gsensor.i2c_adap_id"
#define CFG_GSENSOR_POSITION         "gsensor.position"
#define CFG_GSENSOR_CALIBRATION      "gsensor.calibration"

extern int get_config(const char *key, char *buff, int len);
/*******************************************
 * end for xml cfg
 *******************************************/

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fis210_early_suspend(struct early_suspend *h);
static void fis210_early_resume(struct early_suspend *h);
#endif

static int fis210_axis_remap(struct i2c_client *client, struct fis210_acc *acc);

static int fis210_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int fis210_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int fis_read_count(struct i2c_client *client)
{
      u8 count=0;
      u8 ret  =0;
      ret = fis210_smbus_read_byte(client,FisRegister_CountOut , &(count));

      return count;

}

static int fis210_read_data_AGX(struct i2c_client *client, struct fis210_acc *acc,struct fis210_gyr *gyr) 
{
	u8 tmp_data[12];
	u8 count =0;
	u8 i=0;
      int address=0;
	  
	FisImu_busyWaitForEvent(Fis_Int2, FisInt_positiveEdge);
	if(current_mode == FIS_MODE_AG)
	{
		address = FisRegister_Ax_L;//0x19
		address |= 0x00;

		// Set auto-increment bit if necessary
		address |= (12 > 1) ? 128 : 0;
		count = fis_read_count(client);
           
		if (i2c_smbus_read_i2c_block_data(client,address,12,tmp_data) < 12)
        {
			dev_err(&client->dev, "i2c block read failed\n");
			return -3;
		}
	}
	else if(current_mode == FIS_MODE_AE)
	{
		address = FisRegister_Q1_L;//0x2D;
		address |= 0x00;
		
		// Set auto-increment bit if necessary
		address |= (12 > 1) ? 128 : 0;
		count = fis_read_count(client);

		if (i2c_smbus_read_i2c_block_data(client,FisRegister_Ax_L,12,tmp_data) < 12) {
			dev_err(&client->dev, "i2c block read failed!\n");
			return -3;
		}
	}
        else if(current_mode == FIS_MODE_ACC)
        {
                address = FisRegister_Ax_L;
                address |= 0x00;

                // Set auto-increment bit if necessary
                address |= (6 > 1) ? 128 : 0;
                count = fis_read_count(client);

                if (i2c_smbus_read_i2c_block_data(client,FisRegister_Ax_L,6,tmp_data) < 6) {
                        dev_err(&client->dev, "i2c block read failed!\n");
                        return -3;
                }
        }
        else if(current_mode == FIS_MODE_GYR)
        {
                address = FisRegister_Gx_L;//0x1f;
                address |= 0x00;

                // Set auto-increment bit if necessary
                address |= (6 > 1) ? 128 : 0;
                count = fis_read_count(client);

                if (i2c_smbus_read_i2c_block_data(client,FisRegister_Gx_L,6,tmp_data) < 6) {
                        dev_err(&client->dev, "i2c block read failed!\n");
                        return -3;
                }
        }

	acc->x = ((tmp_data[1] << 8) & 0xff00) | tmp_data[0];
	acc->y = ((tmp_data[3] << 8) & 0xff00) | tmp_data[2];
	acc->z = ((tmp_data[5] << 8) & 0xff00) | tmp_data[4];

	gyr->x = ((tmp_data[7] << 8) & 0xff00) | tmp_data[6];
	gyr->y = ((tmp_data[9] << 8) & 0xff00) | tmp_data[8];
	gyr->z = ((tmp_data[11] << 8) & 0xff00) | tmp_data[10];

	//cal_data(acc);
	/*add cal gyr cal*/
	//printk(KERN_ERR"RAM DATA: %d,%d,%d %d,%d,%d!\n",acc.x,acc.y,acc.z,gyr.x,gyr.y,gyr.z);
	return 0;
}


static int fis210_set_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data = 0;
	
	printk(KERN_ERR "cueernt set mode is %d\n",mode);
	if(mode == FIS_MODE_AG)  //0X00
	{
		printk(KERN_ERR "cueernt mode is AE mode\n");
		current_mode = FIS_MODE_AG;
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl1, 0x00); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl2, 0x10);
		mdelay(10); 
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl3, 0x30); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl5, 0x0a); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl6, 0x00); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x03);
		mdelay(10);

	}
	else if(mode == FIS_MODE_AE) //0X01
	{
		printk(KERN_ERR "cueernt mode is AG mode\n");
		current_mode = FIS_MODE_AE;
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl1, 0x00); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl2, 0x10);
		mdelay(10); 
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl3, 0x30); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl5, 0x0a); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl6, 0x05); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x0f);
		mdelay(10);
	}
	else if(mode == FIS_MODE_ACC) //0X02
	{
		printk(KERN_ERR "cueernt mode is ACC mode\n");
		current_mode = FIS_MODE_ACC;
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl1, 0x00); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl2, 0x10);
		mdelay(10); 
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl3, 0x30); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl5, 0x0a); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl6, 0x00); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x01);
		mdelay(10);
	}
	else if(mode == FIS_MODE_GYR) //0X03
	{
		printk(KERN_ERR "cueernt mode is GYR mode\n");
		current_mode = FIS_MODE_GYR;
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl1, 0x00); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl2, 0x10);
		mdelay(10); 
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl3, 0x30); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl5, 0x0a); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl6, 0x00); 
		mdelay(10);
		i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x02);
		mdelay(10);
	}
	return comres;
}

static int fis210_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 1;
	unsigned char data = 0;

	//comres = fis210_smbus_read_byte(client, MMA2108_MODE_BIT__REG, &data);
	//*mode  = MMA2108_GET_BITSLICE(data, MMA2108_MODE_BIT);
	*mode = current_mode;

	return comres;
}

static int fis210_set_rate(struct i2c_client *client, unsigned char rate)
{
	int comres = 0;
	unsigned char data = 0;

	//comres += fis210_smbus_read_byte(client, MMA2108_RATE_BIT__REG, &data);
	//data  = MMA2108_SET_BITSLICE(data, MMA2108_RATE_BIT, rate);
	//comres += fis210_smbus_write_byte(client, MMA2108_RATE_BIT__REG, &data);

	return comres;
}

static int fis210_get_rate(struct i2c_client *client, unsigned char *rate)
{
	int comres = 0;
	unsigned char data = 0;

	//comres = fis210_smbus_read_byte(client, MMA2108_RATE_BIT__REG, &data);
	//*rate  = MMA2108_GET_BITSLICE(data, MMA2108_RATE_BIT);

	return comres;
}

static void FisImu_reset(void)
{
    gpio_request(CFG_ETHER_GMAC_PHY_RST_NUM_SEN,"sensor Rst pin");
   //add pin for read data status
    gpio_request(CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT1,"sensor Int1 pin");
    gpio_request(CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT2,"sensor Int2 pin");

    gpio_direction_input(CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT1);
    gpio_direction_input(CFG_ETHER_GMAC_PHY_RST_NUM_SEN_INT2);
    gpio_direction_output(CFG_ETHER_GMAC_PHY_RST_NUM_SEN, 1 );
    mdelay(100);
    gpio_direction_output(CFG_ETHER_GMAC_PHY_RST_NUM_SEN, 0 );
}

static void cal_acc_gyr_sum(struct i2c_client *client)
{
	u8 i=0;
	u8 check_num=10;
        struct fis210_acc acc_temp;
        struct fis210_acc gyr_temp;
	for(i=0;i<check_num;i++)
	{
		mdelay(2);
		//fis210_read_data(client,&acc_offset);	
		fis210_read_data_AGX(client,&acc_raw,&gyr_raw);	
		acc_temp.x+=acc_raw.x;
		acc_temp.y+=acc_raw.y;
		acc_temp.z+=acc_raw.z;
		gyr_temp.x+=gyr_raw.x;
		gyr_temp.y+=gyr_raw.y;
		gyr_temp.z+=gyr_raw.z;
	}
	acc_offset.x=acc_temp.x/(check_num);
	acc_offset.y=acc_temp.y/(check_num);
	acc_offset.z=acc_temp.z/(check_num);
	gyr_offset.x=gyr_temp.x/(check_num);
	gyr_offset.y=gyr_temp.y/(check_num);
	gyr_offset.z=gyr_temp.z/(check_num);
	
}

static int fis210_hw_init(struct i2c_client *client)
{
	int ret ;
	FisImu_reset();        
	    
	FisImu_busyWaitForEvent(Fis_Int1, FisInt_positiveEdge);
	printk("Int1 H->L\n");
	FisImu_busyWaitForEvent(Fis_Int1, FisInt_low);
	printk("Int1 Low\n");

	ret = i2c_smbus_read_byte_data(client,0x00);
	printk("%s:i2c-addr = 0x%x, Read ID value is :%d\n",__func__, client->addr, ret);
	mdelay(500);

  	//add fis210 sensor init regs
#if 1
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl1, 0x00); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl2, 0x10);
	mdelay(10); 
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl3, 0x30); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl5, 0x0a); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl6, 0x00); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x03);
	mdelay(10);
#else
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl1, 0x00); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl2, 0x10);
	mdelay(10); 
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl3, 0x30); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl5, 0x0a); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl6, 0x05); 
	mdelay(10);
	i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x0f);
	mdelay(10);

#endif
	// i2c_smbus_write_byte_data(client, FisRegister_Status1, 0x00); 
	mdelay(500);
	//cal_acc_gyr_sum(client);//cal_offset
	printk("####sensor config done###!\n");

	return ret;
}

float m_accScaleFactor = (1.0f / (1 << 12));//8g
float m_gyrScaleFactor = (1.0f / (1 << 4));//2048dps
#if 0
const float m_accScaleFactor = (acc_unit == AccUnit_ms2) ? ((ONE_G)*(1.0f / (1 << 12))):(1.0f / (1 << 12));//8g
const float m_gyrScaleFactor = (gyr_unit == GyrUnit_rads) ? ((M_PI / 180)*(1.0f / (1 << 4))) :(1.0f / (1 << 4));//2048dps
#endif
static void cal_data(struct fis210_acc *acc,struct fis210_gyr *gyr)
{
	struct fis210_acc qst_acc;
        struct fis210_gyr qst_gyr;
        	#if 0  
	VINT16 x, y, z;
                VINT16 x1, y1, z1;
         x = acc.x & 0xffff;
                y = acc.y & 0xffff;
                z = acc.z & 0xffff;

                //XYZ_Filter(&x,&y,&z);

                qst_acc.x = x * m_accScaleFactor;
                qst_acc.y = y * m_accScaleFactor;
                qst_acc.z = z * m_accScaleFactor;

               x1 = gyr.x & 0xffff;
               y1 = gyr.y & 0xffff;
               z1 = gyr.z & 0xffff;

                //XYZ_Filter(&x1,&y1,&z1);

                qst_gyr.x = x1 * m_gyrScaleFactor;
                qst_gyr.y = y1 * m_gyrScaleFactor;
                qst_gyr.z = z1 * m_gyrScaleFactor;

	
	#endif
   
	//printk(KERN_ERR "after calibrate:acc_gyro,%d,%d,%d!\n",(int)qst_acc[0],(int)qst_acc[1],(int)qst_acc[2]);
}
static int fis210_read_data(struct i2c_client *client, struct fis210_acc *acc) 
{
      
      u8 tmp_data[6];	
      u8 i=0;
      static u8 count=0;
      int address;
      address = 0x19;//0x20
      address |= 0x00;
//	FisImu_busyWaitForEvent(Fis_Int2, FisInt_positiveEdge);
	// Set auto-increment bit if necessary
	address |= (6 > 1) ? 128 : 0;
   
      count = fis_read_count(client);
    //i2c_smbus_read_i2c_block_data(client,address,7,tmp_data);        
	if (i2c_smbus_read_i2c_block_data(client,address,6,tmp_data) < 6)
       {
		dev_err(&client->dev, "i2c block read failed\n");
		return -3;
	}
#if 0
	acc->x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	acc->y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	acc->z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];
#else
	acc->x = ((tmp_data[1] << 8) & 0xff00) | tmp_data[0];
	acc->y = ((tmp_data[3] << 8) & 0xff00) | tmp_data[2];
	acc->z = ((tmp_data[5] << 8) & 0xff00) | tmp_data[4];
	
	for(i=0;i<6;i++)
	{
	     printk(KERN_ERR "RAM-BUFFER:  %d,!\n",tmp_data[i]);
	}	
#endif  
       //cal_data(acc);  //OFFSET VALUE
	printk(KERN_ERR "\\n");

	return 0;
}
static ssize_t fis210_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	sscanf(buf, "[0x%x]=0x%x", &address, &value);

	if (fis210_smbus_write_byte(fis210->fis210_client, (unsigned char)address,
				(unsigned char *)&value) < 0)
		return -EINVAL;

	return count;
}

static ssize_t fis210_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);
	size_t count = 0;
	u8 reg[MMA210X_REG_END];
	int i;

	for (i = 0 ; i < MMA210X_REG_END; i++) {
		fis210_smbus_read_byte(fis210->fis210_client, i, reg+i);
		count += sprintf(&buf[count], "0x%x: 0x%x\n", i, reg[i]);
	}

	return count;
}

static ssize_t fis210_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	if (fis210_get_mode(fis210->fis210_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t fis210_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (fis210_set_mode(fis210->fis210_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t fis210_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data=0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	if (fis210_get_rate(fis210->fis210_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t fis210_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (fis210_set_rate(fis210->fis210_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t fis210_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct fis210_data *fis210 = input_get_drvdata(input);
	struct fis210_acc acc;
	struct fis210_gyr gyr;

	fis210_read_data_AGX(fis210->fis210_client, &acc,&gyr);

	return sprintf(buf, "value show:%d %d %d %d %d %d\n", acc.x, acc.y, acc.z,gyr.x, gyr.y, gyr.z);
}

static ssize_t fis210_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&fis210->delay));

}

static ssize_t fis210_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	atomic_set(&fis210->delay, (unsigned int) data);

	return count;
}


static ssize_t fis210_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&fis210->enable));

}

static void fis210_do_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	if (enable) {
		//i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x03);
		//fis210_set_mode(fis210->fis210_client, MMA2108_MODE_ACTIVE);
		schedule_delayed_work(&fis210->work,
		msecs_to_jiffies(atomic_read(&fis210->delay)));
	} else {
		//fis210_set_mode(fis210->fis210_client, MMA2108_MODE_STANDBY);
        i2c_smbus_write_byte_data(client, FisRegister_Ctrl7, 0x00);
		cancel_delayed_work_sync(&fis210->work);
	}
}

static void fis210_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&fis210->enable);

	mutex_lock(&fis210->enable_mutex);
	if (enable != pre_enable) {
		fis210_do_enable(dev, enable);
		atomic_set(&fis210->enable, enable);
	}
	mutex_unlock(&fis210->enable_mutex);
}

static ssize_t fis210_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		fis210_set_enable(dev, data);

	return count;
}

static ssize_t fis210_fuzz_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&fis210->fuzz));

}

static ssize_t fis210_fuzz_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	atomic_set(&(fis210->fuzz), (int) data);

	if(fis210->input != NULL) {
		fis210->input->absinfo[ABS_X].fuzz = data;
		fis210->input->absinfo[ABS_Y].fuzz = data;
		fis210->input->absinfo[ABS_Z].fuzz = data;
	}

	return count;
}

static ssize_t fis210_board_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	data = atomic_read(&(fis210->position));

	return sprintf(buf, "%d\n", data);
}

static ssize_t fis210_board_position_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	error = strict_strtol(buf, 10, &data);
	if (error)
		return error;

	atomic_set(&(fis210->position), (int) data);

	return count;
}

static ssize_t fis210_calibration_run_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int cfg_calibration[3];
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);
	struct fis210_acc acc;

	fis210_read_data(fis210->fis210_client, &acc);

	fis210->offset.x = 0 - acc.x;
	fis210->offset.y = 0 - acc.y;
	if (atomic_read(&fis210->position) > 0) {
		fis210->offset.z = LSG - acc.z;
	} else {
		fis210->offset.z = (-LSG) - acc.z;
	}

	printk(KERN_INFO "fast calibration: %d %d %d\n", fis210->offset.x,
			fis210->offset.y, fis210->offset.z);

	cfg_calibration[0] = fis210->offset.x;
	cfg_calibration[1] = fis210->offset.y;
	cfg_calibration[2] = fis210->offset.z;

	printk(KERN_INFO "run fast calibration finished\n");
	return count;
}

static ssize_t fis210_calibration_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int cfg_calibration[3];
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	memset(&(fis210->offset), 0, sizeof(struct fis210_acc));
	memset(cfg_calibration, 0, sizeof(cfg_calibration));

	printk(KERN_INFO "reset fast calibration finished\n");
	return count;
}

static ssize_t fis210_calibration_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	return sprintf(buf, "%d %d %d\n", fis210->offset.x,
			fis210->offset.y, fis210->offset.z);
}

static ssize_t fis210_calibration_value_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int data[3];
	struct i2c_client *client = to_i2c_client(dev);
	struct fis210_data *fis210 = i2c_get_clientdata(client);

	sscanf(buf, "%d %d %d", &data[0], &data[1], &data[2]);
	fis210->offset.x = (signed short) data[0];
	fis210->offset.y = (signed short) data[1];
	fis210->offset.z = (signed short) data[2];

	printk(KERN_INFO "set fast calibration finished\n");
	return count;
}

#if (1)

static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_register_show, fis210_register_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_mode_show, fis210_mode_store);
static DEVICE_ATTR(rate, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_rate_show, fis210_rate_store);
static DEVICE_ATTR(value, S_IRUGO,
		fis210_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_delay_show, fis210_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_enable_show, fis210_enable_store);
static DEVICE_ATTR(fuzz, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_fuzz_show, fis210_fuzz_store);
static DEVICE_ATTR(board_position, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_board_position_show, fis210_board_position_store);
static DEVICE_ATTR(calibration_run, S_IWUSR|S_IWGRP,
		NULL, fis210_calibration_run_store);
static DEVICE_ATTR(calibration_reset, S_IWUSR|S_IWGRP,
		NULL, fis210_calibration_reset_store);
static DEVICE_ATTR(calibration_value, S_IRUGO|S_IWUSR|S_IWGRP,
		fis210_calibration_value_show,
		fis210_calibration_value_store);
#endif

static struct attribute *fis210_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_mode.attr,
	&dev_attr_rate.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_fuzz.attr,
	&dev_attr_board_position.attr,
	&dev_attr_calibration_run.attr,
	&dev_attr_calibration_reset.attr,
	&dev_attr_calibration_value.attr,
	NULL
};

static struct attribute_group fis210_attribute_group = {
	.attrs = fis210_attributes
};

static int fis210_read_file(char *path, char *buf, int size)
{
	struct file *filp;
	loff_t len, offset;
	int ret=0;
	mm_segment_t fs;

	filp = filp_open(path, O_RDWR, 0777);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		goto out;
	}

	len = vfs_llseek(filp, 0, SEEK_END);
	if (len > size) {
		len = size;
	}

	offset = vfs_llseek(filp, 0, SEEK_SET);

	fs=get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_read(filp, (char __user *)buf, (size_t)len, &(filp->f_pos));

	set_fs(fs);

	filp_close(filp, NULL);
out:
	return ret;
}

static int fis210_load_user_calibration(struct i2c_client *client)
{
	char buffer[16];
	int ret = 0;
	int data[3];
	struct fis210_data *fis210 = i2c_get_clientdata(client);
	int calibrated = atomic_read(&fis210->calibrated);

	// only calibrate once
	if (calibrated) {
		goto usr_calib_end;
	} else {
		atomic_set(&fis210->calibrated, 1);
	}

	ret = fis210_read_file(CFG_GSENSOR_CALIBFILE, buffer, sizeof(buffer));
	if (ret <= 0) {
		printk(KERN_ERR "gsensor calibration file not exist!\n");
		goto usr_calib_end;
	}

	sscanf(buffer, "%d %d %d", &data[0], &data[1], &data[2]);
	fis210->offset.x = (signed short) data[0];
	fis210->offset.y = (signed short) data[1];
	fis210->offset.z = (signed short) data[2];

	printk(KERN_INFO "user cfg_calibration: %d %d %d\n", data[0], data[1], data[2]);

usr_calib_end:
	return ret;
}

static int fis210_axis_remap(struct i2c_client *client, struct fis210_acc *acc)
{
	s16 swap;
	struct fis210_data *fis210 = i2c_get_clientdata(client);
	int position = atomic_read(&fis210->position);

	switch (abs(position)) {
		case 1:
			break;
		case 2:
			swap = acc->x;
			acc->x = acc->y;
			acc->y = -swap;
			break;
		case 3:
			acc->x = -(acc->x);
			acc->y = -(acc->y);
			break;
		case 4:
			swap = acc->x;
			acc->x = -acc->y;
			acc->y = swap;
			break;
	}
	if (position < 0) {
		acc->z = -(acc->z);
		acc->x = -(acc->x);
	}
	acc->x=-acc->x;
	return 0;
}
static void fis210_work_func(struct work_struct *work)
{
	struct fis210_data *fis210 = container_of((struct delayed_work *)work,
			struct fis210_data, work);
	static struct fis210_acc acc;
	static struct fis210_gyr gyr;
	struct fis210_acc qst_acc;
       struct fis210_gyr qst_gyr;
	   
	int result;
	unsigned long delay = msecs_to_jiffies(atomic_read(&fis210->delay));

       result = fis210_read_data_AGX(fis210->fis210_client, &acc,&gyr);

	//uint8_t status = Xkf3_update(g_xkf, &xkfInput); //update g_xkf
	//Xkf3_getEulerOrientation(g_xkf, euler, FusionScenario_Navigation); // cal Orientation output
             
	if (result == 0) {
	printk(KERN_ERR"RAM DATA: %d,%d,%d %d,%d,%d!\n",acc.x,acc.y,acc.z,gyr.x,gyr.y,gyr.z);
	//cal_data(acc,gyr) ;//data 
#if 1
	VINT16 x, y, z;
	VINT16 x1, y1, z1;
	x = acc.x & 0xffff;
	y = acc.y & 0xffff;
	z = acc.z & 0xffff;
#if 0
	if(acc_unit == AccUnit_ms2)
	{
	m_accScaleFactor = m_accScaleFactor*ONE_G;
	}
	else
	{
	m_accScaleFactor = m_accScaleFactor*1;
	}
	if(gyr_unit == GyrUnit_rads)
	{
	m_gyrScaleFactor = m_gyrScaleFactor*M_PI / 180;
	}
	else
	{
	m_gyrScaleFactor = m_gyrScaleFactor*1;
	}
	//XYZ_Filter(&x,&y,&z);

#endif
	qst_acc.x = x * m_accScaleFactor*ONE_G;
	qst_acc.y = y * m_accScaleFactor*ONE_G;
	qst_acc.z = z * m_accScaleFactor*ONE_G;

	x1 = gyr.x & 0xffff;
	y1 = gyr.y & 0xffff;
	z1 = gyr.z & 0xffff;

	//XYZ_Filter(&x1,&y1,&z1);

	qst_gyr.x = x1 * m_gyrScaleFactor*M_PI / 180;
	qst_gyr.y = y1 * m_gyrScaleFactor*M_PI / 180;
	qst_gyr.z = z1 * m_gyrScaleFactor*M_PI / 180;

#endif
			 
	printk(KERN_ERR"RAM DATA: %d,%d,%d %d,%d,%d!\n",qst_acc.x,qst_acc.y,qst_acc.z,qst_gyr.x,qst_gyr.y,qst_gyr.z);
	input_report_abs(fis210->input, ABS_X, acc.x);
	input_report_abs(fis210->input, ABS_Y, acc.y);
	input_report_abs(fis210->input, ABS_Z, acc.z);
	input_sync(fis210->input);
	}
	schedule_delayed_work(&fis210->work, delay);
}

static int fis210_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct fis210_data *data;
	struct input_dev *dev;
	int cfg_position;
	int cfg_calibration[3];
	struct fis210_private_data *pdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct fis210_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->fis210_client = client;
	mutex_init(&data->enable_mutex);

	INIT_DELAYED_WORK(&data->work, fis210_work_func);
	atomic_set(&data->delay, MAX_DELAY);
	atomic_set(&data->enable, 0);

#if CFG_GSENSOR_USE_CONFIG > 0
	/*get xml cfg*/
	err = get_config(CFG_GSENSOR_POSITION, (char *)(&cfg_position), sizeof(int));
	if (err != 0) {
		printk(KERN_ERR"get position %d fail\n", cfg_position);
		goto kfree_exit;
	}
#else
	pdata = client->dev.platform_data;
	//if(pdata)
	//	cfg_position = pdata->position;
	//else
		cfg_position = -3;
#endif
	atomic_set(&data->position, cfg_position);
	atomic_set(&data->calibrated, 0);
	atomic_set(&data->fuzz, FUZZ);

	//power on init regs
	err = fis210_hw_init(data->fis210_client);
	if (err < 0) {
		printk(KERN_ERR"fis210 probe fail! err:%d\n", err);
		goto kfree_exit;
	}

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, FUZZ, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, FUZZ, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, FUZZ, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		goto kfree_exit;
	}

	data->input = dev;

	err = sysfs_create_group(&data->input->dev.kobj,
			&fis210_attribute_group);
	if (err < 0)
		goto error_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = fis210_early_suspend;
	data->early_suspend.resume = fis210_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

#if CFG_GSENSOR_USE_CONFIG > 0
	/*get xml cfg*/
	err = get_config(CFG_GSENSOR_CALIBRATION, (char *)cfg_calibration, sizeof(cfg_calibration));
	if (err != 0) {
		printk(KERN_ERR"get calibration fail\n");
		goto error_sysfs;
	}
#else
	memset(cfg_calibration, 0, sizeof(cfg_calibration));
#endif

	data->offset.x = (signed short) cfg_calibration[0];
	data->offset.y = (signed short) cfg_calibration[1];
	data->offset.z = (signed short) cfg_calibration[2];

	/* default enable */
	fis210_do_enable(&client->dev, 1);

	return 0;

error_sysfs:
	input_unregister_device(data->input);

kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fis210_early_suspend(struct early_suspend *h)
{
	// sensor hal will disable when early suspend
}


static void fis210_early_resume(struct early_suspend *h)
{
	// sensor hal will enable when early resume
}
#endif

static int __devexit fis210_remove(struct i2c_client *client)
{
	struct fis210_data *data = i2c_get_clientdata(client);

	fis210_set_enable(&client->dev, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &fis210_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM

static int fis210_suspend(struct i2c_client *client, pm_message_t state)
{
	fis210_do_enable(&client->dev, 0);

	return 0;
}

static int fis210_resume(struct i2c_client *client)
{
	struct fis210_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	//power on init regs
	fis210_hw_init(data->fis210_client);
	fis210_do_enable(dev, atomic_read(&data->enable));

	return 0;
}

#else

#define fis210_suspend			NULL
#define fis210_resume			NULL

#endif /* CONFIG_PM */

/*
static const unsigned short  fis210_addresses[] = {
	SENSOR_I2C_ADDR,
	I2C_CLIENT_END,
};
*/

static const struct i2c_device_id fis210_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, fis210_id);

static struct i2c_driver fis210_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= SENSOR_NAME,
	},
	.class			= I2C_CLASS_HWMON,
	//.address_list    = fis210_addresses,
	.id_table		= fis210_id,
	.probe			= fis210_probe,
	.remove			= __devexit_p(fis210_remove),
	.suspend 	= fis210_suspend,
	.resume  	= fis210_resume,
};

#if CFG_GSENSOR_USE_CONFIG > 0
static struct i2c_board_info fis210_board_info={
	.type = SENSOR_NAME,
	.addr = SENSOR_I2C_ADDR,
};
#endif

#if CFG_GSENSOR_USE_CONFIG > 0
static struct i2c_client *fis210_client;
#endif

static int __init fis210_init(void)
{
#if CFG_GSENSOR_USE_CONFIG > 0
	struct i2c_adapter *i2c_adap;
	unsigned int cfg_i2c_adap_id;
	int ret;

	ret = get_config(CFG_GSENSOR_ADAP_ID, (char *)(&cfg_i2c_adap_id), sizeof(unsigned int));
	if (ret != 0) {
		printk(KERN_ERR"get i2c_adap_id %d fail\n", cfg_i2c_adap_id);
		return ret;
	}

	i2c_adap = i2c_get_adapter(cfg_i2c_adap_id);
	printk("cfg_i2c_adap_id = %d \n",cfg_i2c_adap_id);
	fis210_client = i2c_new_device(i2c_adap, &fis210_board_info);
	i2c_put_adapter(i2c_adap);
#endif
	return i2c_add_driver(&fis210_driver);
}

static void __exit fis210_exit(void)
{
#if CFG_GSENSOR_USE_CONFIG > 0
	i2c_unregister_device(fis210_client);
#endif
	i2c_del_driver(&fis210_driver);
}

MODULE_AUTHOR("duanzx <duanzx@qstcorp.com>");
MODULE_DESCRIPTION("FIS-sensor 6-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(fis210_init);
module_exit(fis210_exit);
