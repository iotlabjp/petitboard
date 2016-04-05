#include <stdio.h>
#include <math.h>
#include <mraa.h>
#include <unistd.h>

#define XM_ADDRESS 0x1D
#define G_ADDRESS  0x6B

#define WHO_AM_I_G 0x0F
#define CTRL_REG1_G 0x20
#define CTRL_REG2_G 0x21
#define CTRL_REG3_G 0x22
#define CTRL_REG4_G 0x23
#define CTRL_REG5_G 0x24
#define OUT_X_L_G 0x28

#define WHO_AM_I_XM 0x0F
#define CTRL_REG1_XM 0x20
#define CTRL_REG2_XM 0x21
#define CTRL_REG3_XM 0x22
#define CTRL_REG4_XM 0x23
#define CTRL_REG5_XM 0x24
#define CTRL_REG6_XM 0x25
#define CTRL_REG7_XM 0x26
#define OUT_X_L_A 0x28
#define OUT_X_L_M 0x08

typedef struct {
  float x;
  float y;
  float z;
} FTriplet;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} Triplet;

typedef enum {
  ACCEL_SCALE_2G = 0,
  ACCEL_SCALE_4G,
  ACCEL_SCALE_6G,
  ACCEL_SCALE_8G,
  ACCEL_SCALE_16G,
} AccelScale;

const float AccelScaleValue[] = {
  2  / 32768.0,
  4  / 32768.0,
  6  / 32768.0,
  8  / 32768.0,
  16 / 32768.0,
};

typedef enum {
  MAG_SCALE_2GS = 0,
  MAG_SCALE_4GS,
  MAG_SCALE_8GS,
  MAG_SCALE_12GS,
} MagScale;

const float MagScaleValue[] = {
  2  / 32768.0,
  4  / 32768.0,
  8  / 32768.0,
  12 / 32768.0,
};

typedef enum {
  GYRO_SCALE_245DPS = 0,
  GYRO_SCALE_500DPS,
  GYRO_SCALE_2000DPS,
} GyroScale;

const float GyroScaleValue[] = {
  245 / 32768.0,
  500 / 32768.0,
  2000 / 32768.0,
};

mraa_gpio_context gpio;
mraa_i2c_context i2c;

uint8_t read_byte(uint8_t i2c_address, uint8_t reg_address)
{
  mraa_i2c_address(i2c, i2c_address);
  return mraa_i2c_read_byte_data(i2c, reg_address);
}

void read_bytes(uint8_t i2c_address, uint8_t reg_address, uint8_t* dest, uint8_t count)
{
  while(count--) {
    *(dest++) = read_byte(i2c_address, reg_address++);
  }
}

void read_triplet(uint8_t i2c_address, uint8_t reg_address, Triplet* coords)
{
  uint8_t data[6];

  read_bytes(i2c_address, reg_address, data, 6);

  coords->x = (data[1] << 8) | data[0];
  coords->y = (data[3] << 8) | data[2];
  coords->z = (data[5] << 8) | data[4];

  //printf("%04X %04X %04X\n", coords->x, coords->y, coords->z);
}

void write_byte(uint8_t i2c_address, uint8_t reg_address, uint8_t data)
{
  mraa_i2c_address(i2c, i2c_address);
  mraa_i2c_write_byte_data(i2c, data, reg_address);
}

void init(void)
{
  mraa_gpio_context den_g = mraa_gpio_init(25);
  mraa_gpio_dir(den_g, MRAA_GPIO_OUT);
  mraa_gpio_write(den_g, 1);

  i2c = mraa_i2c_init(1);

  if(i2c == NULL) {
    printf("failed to initialize I2C\n");
    exit(1);
  }

  // read and verify WHO_AM_I register
  uint8_t g_id  = read_byte(G_ADDRESS,  WHO_AM_I_G);
  uint8_t xm_id = read_byte(XM_ADDRESS, WHO_AM_I_XM);
  if(!(g_id == 0xD4 && xm_id == 0x49)) {
    printf("failed to verify WHO_AM_I register\n");
    exit(1);
  }
}

void read_acc(AccelScale scale, FTriplet *grav)
{
  Triplet data;
  read_triplet(XM_ADDRESS, OUT_X_L_A, &data);
  grav->x = data.x * AccelScaleValue[scale];
  grav->y = data.y * AccelScaleValue[scale];
  grav->z = data.z * AccelScaleValue[scale];
}

void init_acc(AccelScale scale)
{
  write_byte(XM_ADDRESS, CTRL_REG1_XM, 0x57);

  uint8_t reg2_xm = read_byte(XM_ADDRESS, CTRL_REG2_XM);
  write_byte(XM_ADDRESS, CTRL_REG2_XM, (reg2_xm & 0xC7) | scale << 3);
}

void read_mag(MagScale scale, FTriplet *gauss)
{
  Triplet data;
  read_triplet(XM_ADDRESS, OUT_X_L_M, &data);
  gauss->x = data.x * MagScaleValue[scale];
  gauss->y = data.y * MagScaleValue[scale];
  gauss->z = data.z * MagScaleValue[scale];
}

void init_mag(MagScale scale)
{
  write_byte(XM_ADDRESS, CTRL_REG5_XM, 0x98);
  write_byte(XM_ADDRESS, CTRL_REG6_XM, scale << 5);
  write_byte(XM_ADDRESS, CTRL_REG7_XM, 0x00);
}

void read_gyro(GyroScale scale, FTriplet* dps)
{
  Triplet data;
  read_triplet(G_ADDRESS, OUT_X_L_G, &data);
  dps->x = data.x * GyroScaleValue[scale];
  dps->y = data.y * GyroScaleValue[scale];
  dps->z = data.z * GyroScaleValue[scale];
}

void init_gyro(GyroScale scale)
{
  write_byte(G_ADDRESS, CTRL_REG1_G, 0x0F);
  uint8_t reg4_g = read_byte(G_ADDRESS, CTRL_REG4_G);
  write_byte(G_ADDRESS, CTRL_REG4_G, (reg4_g & 0xCF) | scale << 4);
}

int main(void)
{
  init();
  init_acc(ACCEL_SCALE_2G);
  init_mag(MAG_SCALE_2GS);
  init_gyro(GYRO_SCALE_245DPS);

  for(int i = 0; i < 1000; i++) {
    FTriplet ftriplet;
    //read_acc(ACCEL_SCALE_2G, &ftriplet);
    //read_mag(MAG_SCALE_2GS, &ftriplet);
    read_gyro(GYRO_SCALE_245DPS, &ftriplet);
    float norm = sqrt(ftriplet.x * ftriplet.x + ftriplet.y * ftriplet.y + ftriplet.z * ftriplet.z);
    printf("%+9.4f %+9.4f %+9.4f %+9.4f\n", ftriplet.x, ftriplet.y, ftriplet.z, norm);
    usleep(1000 * 100);
  }

  return 0;
}
