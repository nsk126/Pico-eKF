#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "hardware/dma.h"
#include "math.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#define ADDR 0x68

#define RAD_TO_DEG 57.295779513
#define DEG_TO_RAD 0.0174533

#ifdef i2c_default

static bool reserved_addr(uint8_t addr);
static void mpu6050_reset(uint8_t addr);
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
static void i2c_detect(uint8_t addr);
static void i2c_default_pins();

#endif

int16_t acceleration[3], gyro[3], temp;
static int addr = 0x68;

double f_phi = 0; 
double f_theta = 0; 

bool repeating_timer_callback(struct repeating_timer *t){

    double ax = (double) (acceleration[0]/16384.0);
    double ay = (double) (acceleration[1]/16384.0);
    double az = (double) (acceleration[2]/16384.0);

    double gx = (double) (gyro[0]/131.0);
    double gy = (double) (gyro[1]/131.0);
    double gz = (double) (gyro[2]/131.0);

    double a_phi = atan2(ay,az);
    double a_theta = atan2(-ax,sqrt(pow(ay,2) + pow(az,2)));
    // double theta2 = asin(ax/9.81) * RAD_TO_DEG; DO NOT USE


    // Body transformation test
    // double gx_cap = gx + sin(f_phi)*tan(f_theta)*gy + cos(f_phi)*tan(f_theta)*gz;
    // double gy_cap = cos(f_phi)*gy - sin(f_phi)*gz;

    double gy_phi = f_phi + (gx * 0.001); 
    double gy_theta = f_phi + (gy * 0.001);
    
    /**
     * @brief Complementary Filer
     * Alpha = 0.05
     * 
     */

    double alpha = 0.05;

    double f_phi = alpha * gy_phi + (1.0 - alpha) * a_phi;
    double f_theta = alpha * gy_theta + (1.0 - alpha) * a_theta;
    
    // printf("\033c"); // Character to clear terminal buffer
    // printf("%.2f,%.2f\n",a_phi,a_theta);
    // printf("%.2f,%.2f\n",gy_phi,gy_theta);
    printf("%.2f,%.2f\n",f_phi * RAD_TO_DEG,f_theta * RAD_TO_DEG);
      
    
    return true;
}

// Thread running on Core-0
// Blink thread with priority 2
void blink(void *param){

    while (1)
    {
        /* code */
        gpio_put(25,1);
        vTaskDelay(500);
        gpio_put(25,0);
        vTaskDelay(500);
    }   
}

// Thread running on Core-0
// Read MPU6050 thread with priority 1
void mpu_read(void *param){

    while (1)
    {
        /* code */
        mpu6050_read_raw(acceleration, gyro, &temp);

    }    
}


// Core 1 - Main thread
void Core_1(){

    struct repeating_timer timer;
    add_repeating_timer_ms(1, repeating_timer_callback, NULL, &timer);

    while (1)
    {
        tight_loop_contents();
    }   
    

}

// Core 0 - Main thread
int main() 
{
    // USB printing ENABLE
    stdio_init_all();

    // buffer time for MPU to boot up
    sleep_ms(2000);
    

    // INITIATE BUS WITH PRECAUTIONS
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else

    // Init I2C Pins
    i2c_default_pins();

    // END OF BUS INITIATION

    // SET PWR managment registers in MPU6050
    mpu6050_reset(ADDR);

    // Blink LED
    gpio_init(25);
    gpio_set_dir(25,GPIO_OUT);

    // Core-1 Init
    multicore_launch_core1(Core_1);

    // FreeRTOS Threads
    TaskHandle_t gLED = NULL;
    TaskHandle_t mpu_I2C_read = NULL;
    
    // uint32_t status = xTaskCreate(blink, "B", 256, NULL, tskIDLE_PRIORITY, &gLED);
    // status = xTaskCreate(usb_fb, "A", 256, NULL, 1, &usb_task);

    uint32_t status = xTaskCreate(blink, "A", 1024, NULL, 2, &gLED);
    status = xTaskCreate(mpu_read, "B", 1024, NULL, 1, &mpu_I2C_read);

    vTaskStartScheduler();

    while(1){};

#endif

    return 0;
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
