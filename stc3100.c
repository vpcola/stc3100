#include "stc3100.h"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define STC3100_ADDR 0x70           // STC3100 7-bit address
//#define STC3100_ADDR 0xE0         // STC3100 8-bit address

/* ******************************************************************************** */
#define SENSERESISTOR  33    // sense resistor value in milliOhms (10min, 100max)
/*   TO BE DEFINED ACCORDING TO HARDWARE IMPLEMENTATION                             */
/* ******************************************************************************** */

#define STC3100_OK 0


/*Address of the STC3100 register --------------------------------------------*/
#define STC3100_REG_MODE                 0x00 /*Mode Register                 */
#define STC3100_REG_CTRL                 0x01 /*Control and Status Register   */
#define STC3100_REG_CHARGE_LOW           0x02 /*Gas Gauge Charge Data Bits 0-7*/
#define STC3100_REG_CHARGE_HIGH          0x03 /*Gas Gauge Charge Data Bits 8-15*/    
#define STC3100_REG_COUNTER_LOW          0x04 /*Number of Conversion Bits 0-7*/
#define STC3100_REG_COUNTER_HIGH         0x05 /*Number of Conversion Bits 8-15*/
#define STC3100_REG_CURRENT_LOW          0x06 /*Battery Current Value Bits 0-7*/
#define STC3100_REG_CURRENT_HIGH         0x07 /*Battery Current Value Bits 8-15*/
#define STC3100_REG_VOLTAGE_LOW          0x08 /*Battery Voltage Value Bits 0-7*/
#define STC3100_REG_VOLTAGE_HIGH         0x09 /*Battery Voltage Value Bits 8-15*/
#define STC3100_REG_TEMPERATURE_LOW      0x0A /*Temperature Values Bits 0-7) */
#define STC3100_REG_TEMPERATURE_HIGH     0x0B /*Temperature Values Bits 8-15)*/

/* Device ID registers Address 24 to 31 --------------------------------------*/
#define STC3100_REG_ID0                  0x18 /*Part Type ID 10h  */
#define STC3100_REG_ID1                  0x19 /*Unique Part ID Bits 0-7  */
#define STC3100_REG_ID2                  0x1A /*Unique Part ID Bits 8-15  */
#define STC3100_REG_ID3                  0x1B /*Unique Part ID Bits 16-23  */
#define STC3100_REG_ID4                  0x1C /*Unique Part ID Bits 24-31  */
#define STC3100_REG_ID5                  0x1D /*Unique Part ID Bits 32-39  */
#define STC3100_REG_ID6                  0x1E /*Unique Part ID Bits 40-47  */
#define STC3100_REG_ID7                  0x1F /*Device ID CRC  */

/*    General Purpose RAM Register Address 32-63     */
#define STC3100_RAM_SIZE      32  /*Total RAM register of STC3100*/

#define STC3100_REG_RAM0                               0x20  
#define STC3100_REG_RAM2                               0x22
#define STC3100_REG_RAM4                               0x24
#define STC3100_REG_RAM6                               0x26
#define STC3100_REG_RAM8                               0x28  
#define STC3100_REG_RAM12                              0x2C  
#define STC3100_REG_RAM14                              0x2E  
#define STC3100_REG_RAM16                              0x30  
#define STC3100_REG_RAM18                              0x32 
#define STC3100_REG_RAM20                              0x34 
#define STC3100_REG_RAM22                              0x36 
#define STC3100_REG_RAM24                              0x38 
#define STC3100_REG_RAM26                              0x3A 
#define STC3100_REG_RAM28                              0x3C 
#define STC3100_REG_RAM30                              0x3E 

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

static esp_err_t stc3100_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t * value, size_t len)
{
    int i;
    uint8_t data[65];

    if (len > 64) return ESP_FAIL;

    data[0] = reg;
    for(i = 0; i < len; i++) data[i + 1] = value[i];

    return i2c_master_write_slave(i2c_num, STC3100_ADDR, (uint8_t *) &data, len+1);
}

static esp_err_t stc3100_read_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t * value, size_t len)
{
    esp_err_t ret;

    // Send the device address first.
    ret = i2c_master_write_slave(i2c_num, STC3100_ADDR, (uint8_t *) &reg, 1);

    // Now read the data from the device
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (STC3100_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    // Ack except the last byte read
    if (len > 1)
    {
        i2c_master_read(cmd, (uint8_t *) value, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, (uint8_t *) (value + len - 1), I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


esp_err_t stc3100_init(i2c_port_t i2cnum)
{
    esp_err_t ret;
    uint8_t devid;
    uint8_t regs[2];

    // Read the device ids to determine the prescense of the STC3100 ic
    //
    ret = stc3100_read_reg(i2cnum, STC3100_REG_ID0, (uint8_t *) &devid, 1);
    // We only need the device type 10h to check the presence of the device
    if (devid != 0x10)
    {
        return ESP_FAIL;
    }

    // read the reg_ctrl to reset the GG_EOC and VTM_EOC bits
    stc3100_read_reg(i2cnum, STC3100_REG_CTRL, &regs[0], 1);

    // Write CTRL
    regs[0] = 0x02;
    stc3100_write_reg(i2cnum, STC3100_REG_CTRL, &regs[0], 1);
    // Write mode
    regs[0] = 0x10;
    stc3100_write_reg(i2cnum, STC3100_REG_MODE, &regs[0], 1);
    
    return ret; 
}


esp_err_t stc3100_read_device_id(i2c_port_t i2cnum, uint8_t * devid, size_t len)
{
   if (len < 8) return ESP_FAIL;

   return stc3100_read_reg(i2cnum, STC3100_REG_ID0, devid, len);
}


