#define USER_SETUP_ID 70
#define ILI9341_DRIVER

// THE STABLE P-51 PINS:
#define TFT_MISO 9   // Even if unused, define it to clear the pin
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10
#define TFT_DC   13
#define TFT_RST  14
#define USE_HSPI_PORT
#define TFT_S3_OPTIMIZED

#define LOAD_GLCD
#define LOAD_FONT2
#define SPI_FREQUENCY  27000000