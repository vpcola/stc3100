#include "stc3100.h"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define STC3100_REG_MODE        0  // R/W Mode register
#define STC3100_REG_CTRL        1  // R/W Control and status register
#define STC3100_REG_CHARGE      2  // R   Gas gauge charge data (16 bits)
#define STC3100_REG_COUNTER     4  // R   Number of conversions (16 bits)
#define STC3100_REG_CURRENT     6  // R   Battery current value (16 bits)
#define STC3100_REG_VOLTAGE     8  // R   Battery voltage value (16 bits)
#define STC3100_REG_TEMPERATURE 10 // R   Temperature value (16 bits)
#define STC3100_REG_ID          24 // R   Unique device ID (8 bytes)
#define STC3100_REG_RAM         32 // R/W User RAM (32 bytes)

// MODE register bits
#define SEL_EXT_CLK 0x01
#define GG_RES_14   0x00
#define GG_RES_13   0x02
#define GG_RES_12   0x04
#define GG_CAL      0x08
#define GG_RUN      0x10

// CTRL register bits
#define IO0DATA     0x01
#define GG_RST      0x02
#define GG_EOC      0x04
#define VTM_EOC     0x08
#define PORDET      0x10


/**
* @brief Test code to write esp-i2c-slave
*        Master device write data to slave(both esp32),
*        the data will be stored in slave buffer.
*        We can read them out from slave buffer.
*
* ___________________________________________________________________
* | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
* --------|---------------------------|----------------------|------|
*
*/
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_wr, size_t size)
{
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, ( slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
   i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
   i2c_master_stop(cmd);
   esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   return ret;
}


esp_err_t stc3100_init(i2c_port_t i2cnum)
{
    return ESP_OK; 
}


