 /***************************************************************************/
 #ifndef PLATFORM_DRIVERS_H_
 #define PLATFORM_DRIVERS_H_

#include "main.h"

 /******************************************************************************/
 /********************** Macros and Constants Definitions **********************/
 /******************************************************************************/

// #define SUCCESS     0
// #define FAILURE     -1
enum {
	FAILURE = -1
};

 #define SPI_CPHA    0x01
 #define SPI_CPOL    0x02

 #define GPIO_OUT    0x01
 #define GPIO_IN     0x00

 #define GPIO_HIGH   0x01
 #define GPIO_LOW    0x00

 /******************************************************************************/
 /*************************** Types Declarations *******************************/
 /******************************************************************************/

 typedef enum {
     GENERIC_I2C
 } i2c_type;

// typedef struct {
//     i2c_type    type;
//     uint32_t    id;
//     char        *pathname;
//     uint32_t    max_speed_hz;
//     uint8_t     slave_address;
// } i2c_init_param;

 typedef struct {
//     i2c_type    type;
//     uint32_t    id;
//     int     fd;
//     uint32_t    max_speed_hz;
     uint8_t     slave_address;
 } i2c_desc;

 typedef enum {
     GENERIC_SPI
 } spi_type;

 typedef enum {
     SPI_MODE_0 = (0 | 0),
     SPI_MODE_1 = (0 | SPI_CPHA),
     SPI_MODE_2 = (SPI_CPOL | 0),
     SPI_MODE_3 = (SPI_CPOL | SPI_CPHA)
 } spi_mode;

// typedef struct {
//     spi_type    type;
//     uint32_t    id;
//     char        *pathname;
//     uint32_t    max_speed_hz;
//     spi_mode    mode;
//     uint8_t     chip_select;
// } spi_init_param;

typedef struct {
//     spi_type    type;
//     uint32_t    id;
//     int     fd;
//     uint32_t    max_speed_hz;
//     spi_mode    mode;
//     uint8_t     chip_select;
} spi_desc;

 typedef enum {
     GENERIC_GPIO
 } gpio_type;

 typedef struct {
     gpio_type   type;
     uint32_t    id;
     uint8_t     number;
 } gpio_desc;

 /******************************************************************************/
 /************************ Functions Declarations ******************************/
 /******************************************************************************/

// /* Initialize the I2C communication peripheral. */
// int32_t i2c_init(i2c_desc **desc,
//          const i2c_init_param *param);
//
// /* Free the resources allocated by i2c_init(). */
// int32_t i2c_remove(i2c_desc *desc);

 /* Write data to a slave device. */
 int32_t i2c_write(i2c_desc *desc,
           uint8_t *data,
           uint8_t bytes_number,
           uint8_t option);

 /* Read data from a slave device. */
 int32_t i2c_read(i2c_desc *desc,
          uint8_t *data,
          uint8_t bytes_number,
          uint8_t option);

// /* Initialize the SPI communication peripheral. */
// int32_t spi_init(spi_desc **desc,
//          const spi_init_param *param);
//
// /* Free the resources allocated by spi_init() */
// int32_t spi_remove(spi_desc *desc);

 /* Write and read data to/from SPI. */
 int32_t spi_write_and_read(spi_desc *desc,
                uint8_t *data,
                uint8_t bytes_number);

 /* Obtain the GPIO decriptor. */
 int32_t gpio_get(gpio_desc **desc,
          uint8_t gpio_number);

 /* Free the resources allocated by gpio_get() */
 int32_t gpio_remove(gpio_desc *desc);

 /* Enable the input direction of the specified GPIO. */
 int32_t gpio_direction_input(gpio_desc *desc);

 /* Enable the output direction of the specified GPIO. */
 int32_t gpio_direction_output(gpio_desc *desc,
                   uint8_t value);

 /* Get the direction of the specified GPIO. */
 int32_t gpio_get_direction(gpio_desc *desc,
                uint8_t *direction);

 /* Set the value of the specified GPIO. */
 int32_t gpio_set_value(gpio_desc *desc,
                uint8_t value);

 /* Get the value of the specified GPIO. */
 int32_t gpio_get_value(gpio_desc *desc,
                uint8_t *value);

 /* Generate microseconds delay. */
 void udelay(uint32_t usecs);

 /* Generate miliseconds delay. */
 void mdelay(uint32_t msecs);

 #endif // PLATFORM_DRIVERS_H_
