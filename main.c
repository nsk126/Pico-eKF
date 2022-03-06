#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"

#define ADDR 0x68

/**
 * @brief LIST OF FUNCTIONS
 * gpio_init
 * gpio_set_dir
 * stdio_init_all
 * multicore_launch_core1
 */


static int addr = 0x68;

#ifdef i2c_default

static bool reserved_addr(uint8_t addr);

static void mpu6050_reset(uint8_t addr);

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

static void i2c_detect(uint8_t addr);

static void i2c_default_pins();

#endif

int16_t acceleration[3], gyro[3], temp;

void Core_2(){

    sleep_ms(3000);

    /**
     * @brief 
     * Gyro 8Khz
     * Accel 1Khz
     * 
     */
    

    while (true) {
        
        double ax = (double) (acceleration[0]/16384.0);
        double ay = (double) (acceleration[1]/16384.0);
        double az = (double) (acceleration[2]/16384.0);

        double gx = (double) (gyro[0]/131.0);
        double gy = (double) (gyro[1]/131.0);
        double gz = (double) (gyro[2]/131.0);

        


        // printf("| %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | \n", ax, ay, az, gx, gy, gz);
        
        // sleep_ms(200);
    }
    
    
}


int main() {

    
    // Initialize chosen serial port
    stdio_init_all();

    /**
     * @brief Multicore functions
     * Core 1 -> Core 0
     * Core 2 -> Core 1
     * 
     */


    // buffer time for MPU to boot up
    sleep_ms(2000);
    

    // INITIATE BUS WITH PRECAUTIONS

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else

    i2c_default_pins();

    // END OF BUS INITIATION
    
    mpu6050_reset(ADDR);

    // WHOAMI - Device ID check
    // uint8_t t_val = 0x1A;
    // uint8_t TEMP[1];
    // i2c_write_blocking(i2c_default,ADDR,&t_val,1,true);
    // i2c_read_blocking(i2c_default,ADDR,TEMP,1,false);
    // printf("Value: 0x%x", TEMP[0]);

    multicore_launch_core1(Core_2);
    


    while (1) {

        mpu6050_read_raw(acceleration, gyro, &temp);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information

        
        // printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        // printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        
    }

#endif


}


bool reserved_addr(uint8_t addr) {

    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;

}

static void mpu6050_reset(uint8_t addr) {

    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);

}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

static void i2c_detect(uint8_t addr){

    
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");


}

static void i2c_default_pins(){

    printf("Hello, MPU6050! Reading raw data from registers...\n");

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

}
