#ifndef PRM_H
#define	PRM_H

#include "config.h"

/*----------------------------------------------------------
   Dial design
-----------------------------------------------------------*/
#if F_MAIN_OUTSIDE == 1
  #define MAINDIAL_IS_OUTSIDE //Maindial is inside
#endif

#define F_REV 0

#if F_REV == 1                //Dial Spins normally
#else
 #define REV_DIAL             //Dial spin in reverse
#endif

#define MODE0 // MODE0, MODE1, MODE2



//Dial number size ------------------------------------------------
//Font size of numbers
#define font_sizex_main DIAL_FONT
#define font_sizey_main (DIAL_FONT * 0.9)
#define font_sizex_sub  font_sizex_main
#define font_sizey_sub  font_sizey_main

#define line_start     DP_POS         //dial pointer extends beyond dial by this amount


//Frequency per main dial scale  ----------------------------------------------------------
#define  freq_tick_main FREQ_TICK_MAIN // Frequency per tick of Main dial: 10000(10kHz) or 100000(100kHz)
#define  freq_tick_sub  FREQ_TICK_SUB  // Frequency per tick of Sub  dial: 1000(1kHz) or 10000(10kHz)



//Dial display position------------------------------------------------
//Position of dial display
#define center_offset  DISP_L/2
#define top_position   55
#define top_margin     DISP_TM


//dial radius---------------------------------------------------
#define dial_radius D_R  //Dial radius (if 10000, Linear scale)


//Spacing between main and sub dials ------------------------------
#define dial_space  DIAL_SPACE  // Space bitween Main and Sub dial


//Scale interval---------------------------------------------
#define tick_pitch_main  TICK_PITCH_MAIN/2  // Tick pitch of main dial
#define tick_pitch_sub   TICK_PITCH_SUB/2   // Tick pitch of sub dial


//Scale thickness----------------------------------------
#define tw_main1   TICK_WIDTH1*0.66  // Tick width of main(1)
#define tw_main5   TICK_WIDTH2*0.9   // Tick width of main(5)
#define tw_main10  TICK_WIDTH3*0.9   // Tick width of main(10)
#define tw_sub1    TICK_WIDTH1*0.66  // Tick width of sub(1)
#define tw_sub5    TICK_WIDTH2*0.9   // Tick width of sub(5)
#define tw_sub10   TICK_WIDTH3*0.9   // Tick width of sub(10)

//Scale length----------------------------------------
#define tl_main1   TICK_MAIN1*0.9   // Tick length of main(1)
#define tl_main5   TICK_MAIN5*0.8   // Tick length of main(5)
#define tl_main10  TICK_MAIN10*0.7  // Tick length of main(10)
#define tl_sub1    TICK_SUB1*0.9    // Tick length of sub(1)
#define tl_sub5    TICK_SUB5*0.8    // Tick length of sub(5)
#define tl_sub10   TICK_SUB10*0.7   // Tick length of sub(10)


//scale/space between numbers-------------------------------------------
#define TNCL_main  TNCL_MAIN*0.7  // Space bitween Number and Tick (Main dial)
#define TNCL_sub   TNCL_SUB*0.7  // Space bitween Number and Tick (Sub dial)


// Pointer width, length-------------------------
#define DP_width   DP_WIDTH  // Needle width
#define DP_len     DP_LEN    // Needle length

// Dial Colors -------------------------
#define TickMainCol CL_TICK_MAIN
#define TickSubCol  CL_TICK_SUB
#define NumMainCol  CL_NUM_MAIN
#define NumSubCol   CL_NUM_SUB 
#define PointerCol  CL_POINTER
#define BGCol       TFT_BLACK //0x80FF80U
//#define DialBGCol   BGCol
#define TFT_BLACK2  0x0020  //opaque black

// Micro Controller SPI Configuration -------------------------
//#if MC_TYPE == S3MINI || MC_TYPE == S2MINI || MC_TYPE == S3ZERO || MC_TYPE == C3FH4 || MC_TYPE == C3MINI
  //#define SPIHOST SPI2_HOST  // SPI2_HOST or SPI3_HOST 
//#else if MC_TYPE == WROVER
  //#define SPIHOST HSPI_HOST // VSPI_HOST or HSPI_HOST
//#endif

//#define LGFX_ESP32_S2

#define SPIHOST SPI2_HOST

#if MC_TYPE == R2040
 #pragma once
 #define LGFX_USE_V1
#endif

#include <LovyanGFX.hpp>
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7735S _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
#if MC_TYPE == R2040
    lgfx::Light_PWM     _light_instance;
  #endif
public:
  LGFX(void)
  {
    #if MC_TYPE != R2040
    { // // SPI
      auto cfg = _bus_instance.config(); // Get the structure for bus settings.
      cfg.spi_host = SPIHOST;      // Select the SPI to use ESP32-S2,C3: SPI2_HOST or SPI3_HOST / ESP32: VSPI_HOST or HSPI_HOST
      // Due to the ESP-IDF version update, the description of VSPI_HOST and HSPI_HOST will be deprecated, so if an error occurs, please use SPI2_HOST and SPI3_HOST instead.
      cfg.spi_mode = 0;              // Set SPI communication mode (0 ~ 3)
      cfg.freq_write = 40000000;     // SPI clock when transmitting (maximum 80MHz, rounded to 80MHz divided by an integer)
      cfg.freq_read  = 16000000;     // SPI clock when receiving
      cfg.spi_3wire  = false;        // Set true if receiving is done using the MOSI pin.
      cfg.use_lock   = true;         //  Set true to use transaction locking
      cfg.dma_channel = SPI_DMA_CH_AUTO; // Set the DMA channel to use (0=DMA not used / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=automatic setting)
      // * With the ESP-IDF version upgrade, SPI_DMA_CH_AUTO (automatic setting) is recommended for the DMA channel. Specifying 1ch or 2ch is not recommended.
      cfg.pin_sclk = TFT_SCLK;       // Set SPI SCLK pin number
      cfg.pin_mosi = TFT_MOSI;       // Set SPI MOSI pin number
      cfg.pin_miso = -1;             // Set SPI MISO pin number (-1 = disable)
      cfg.pin_dc   = TFT_DC;         // Set SPI D/C pin number (-1 = disable)
      _bus_instance.config(cfg);     // Reflects the setting value on the bus.
      _panel_instance.setBus(&_bus_instance);      // Place the bus on the panel.
    }

    { // Configure display panel control settings.
      auto cfg = _panel_instance.config(); // Gets the structure for display panel settings.
      cfg.pin_cs           =  TFT_CS; // in number to which CS is connected (-1 = disable)
      cfg.pin_rst          =  TFT_RST;// Pin number to which RST is connected (-1 = disable)
      cfg.pin_busy         =    -1;   // Pin number to which BUSY is connected (-1 = disable)

      // * The following setting values ​​are general default values ​​set for each panel, so please comment out any items you are unsure of and try again.
      cfg.panel_width      =   128;  // Actual displayable width / Width actually used (set the same value or smaller value than memory_width)
      cfg.panel_height     =   160;  // Actual display height / Height actually used (set the same value or smaller value than memory_height)
      cfg.offset_x         =     0;  // Panel X direction offset amount / Amount to shift the display position to the right (initial value 0)
      cfg.offset_y         =     0;  // Panel Y direction offset amount / Amount to shift the display position downward (initial value 0)
      cfg.offset_rotation  =     3;  // Offset of value in rotation direction 0~7 (4~7 are upside down)
      cfg.dummy_read_pixel =     8;  // Number of dummy read bits before pixel readout
      cfg.dummy_read_bits  =     1;  // Number of bits for dummy read before reading data other than pixels
      cfg.readable         = false;  // Set to true if display data reading is possible
      cfg.invert           = false;  // Set to true if the brightness and darkness of the panel is reversed.
      cfg.rgb_order        = true;   // Set to true if the red and blue colors of the panel are swapped.
      cfg.dlen_16bit       = false;  // For panels that transmit data length in 16-bit units using 16-bit parallel or SPI, set to true, false: 8bit
      cfg.bus_shared       = false;  // Set to true when sharing the bus with the SD card (control the bus using drawJpgFile, etc.)

      // Typically you specify the same value for memory_width and panel_width and use offset_x = 0. ;
      // If you want to prevent the display at the edge of the screen from being hidden outside the screen, 
      // set the panel_width value smaller than memory_width and adjust the horizontal position using offset_x.
      // For example, if panel_width is set to a value 32 smaller than memory_width, setting offset_x to 16 will center the left and right positions.
      // Similarly, adjust the vertical direction (memory_height, panel_height, offset_y) as necessary.
      // Please set the following only if the display is misaligned with a variable pixel number driver such as ST7735 or ILI9163.
      cfg.memory_width     =   128;  // Maximum width supported by driver IC / Output resolution width
      cfg.memory_height    =   160;  // Maximum height supported by driver IC / Output resolution height


      _panel_instance.config(cfg);
    } 
    #endif
    #if MC_TYPE == R2040
        {
        auto cfg = _bus_instance.config();
        cfg.spi_host   = 0;
        cfg.spi_mode   = 0;
        cfg.freq_write = 80000000;
        cfg.pin_sclk   = TFT_SCLK;
        cfg.pin_miso   = TFT_MISO;
        cfg.pin_mosi   = TFT_MOSI;
        cfg.pin_dc     = TFT_DC;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
        }
     
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs       = TFT_CS;
      cfg.pin_rst      = TFT_RST;
      cfg.panel_width  = 128;
      cfg.panel_height = 160;
      cfg.offset_x     = 0;
      cfg.offset_y     = 2;
      cfg.invert       = false;
      cfg.rgb_order    = true;
      cfg.offset_rotation = 3;
      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl      = TFT_PI;
      cfg.pwm_channel = 1;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }
    #endif
    setPanel(&_panel_instance); // Set the panel to be used.
  }
};

#endif
