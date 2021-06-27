/**
 * Copyright Nikita Bulaev 2017
 *
 * STM32 HAL libriary for LCD display based on HITACHI HD44780U chip.
 *
 * ===========================================================================
 * WARNING!
 *
 * YOU MUST INCLUDE CORRECT STM32 HAL LIB HEAR. THIS LIB WAS TESTED ON STM32F3
 * PLEASE, INCLUDE CORRECT ONE!
 * ===========================================================================
 *
 * THIS SOFTWARE IS PROVIDED "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LCD_HD44780_I2C_H
#define LCD_HD44780_I2C_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LCD_BIT_RS ((uint8_t)0x01U)
#define LCD_BIT_RW ((uint8_t)0x02U)
#define LCD_BIT_E ((uint8_t)0x04U)
#define LCD_BIT_BACKIGHT_ON ((uint8_t)0x08U)
#define LCD_BIT_BACKIGHT_OFF ((uint8_t)0x00U)

#define LCD_MODE_4BITS ((uint8_t)0x02U)
#define LCD_BIT_1LINE ((uint8_t)0x00U)
#define LCD_BIT_2LINE ((uint8_t)0x08U)
#define LCD_BIT_4LINE LCD_BIT_2LINE
#define LCD_BIT_5x8DOTS ((uint8_t)0x00U)
#define LCD_BIT_5x10DOTS ((uint8_t)0x04U)
#define LCD_BIT_SETCGRAMADDR ((uint8_t)0x40U)
#define LCD_BIT_SETDDRAMADDR ((uint8_t)0x80U)

#define LCD_BIT_DISPLAY_CONTROL ((uint8_t)0x08U)
#define LCD_BIT_DISPLAY_ON ((uint8_t)0x04U)
#define LCD_BIT_CURSOR_ON ((uint8_t)0x02U)
#define LCD_BIT_CURSOR_OFF ((uint8_t)0x00U)
#define LCD_BIT_BLINK_ON ((uint8_t)0x01U)
#define LCD_BIT_BLINK_OFF ((uint8_t)0x00U)

#define LCD_BIT_DISP_CLEAR ((uint8_t)0x01U)
#define LCD_BIT_CURSOR_HOME ((uint8_t)0x02U)

#define LCD_BIT_ENTRY_MODE ((uint8_t)0x04U)
#define LCD_BIT_CURSOR_DIR_RIGHT ((uint8_t)0x02U)
#define LCD_BIT_CURSOR_DIR_LEFT ((uint8_t)0x00U)
#define LCD_BIT_DISPLAY_SHIFT ((uint8_t)0x01U)

// TODO: Update commands with this defines
#define LCD_BIT_CURSOR_SHIFT_MODE ((uint8_t)0x10U)
#define LCD_BIT_CURSOR_DISP_SHIFT ((uint8_t)0x08U)
#define LCD_BIT_CURSOR_MOVE ((uint8_t)0x00U)
#define LCD_BIT_CURSOR_SHIFT_DIR_R ((uint8_t)0x40U)
#define LCD_BIT_CURSOR_SHIFT_DIR_L ((uint8_t)0x00U)

/* Function defines */

#ifndef bool
typedef enum {
    false,
    true
} bool;
#endif

typedef int (*lcd_hd44780_transmit_t)(uint8_t address, uint8_t* buffer, int size);
typedef void (*lcd_hd44780_delay_t)(int ms);

typedef struct lcd_hd44780 {
    lcd_hd44780_params lcd_params;
    lcd_hd44780_transmit_t transmit;
    lcd_hd44780_delay_t delay;
    uint8_t buffer[6];
} lcd_hd44780_t;

typedef struct {
    uint8_t lines; // Lines of the display
    uint8_t columns; // Columns
    uint8_t address; // I2C address shifted left by 1
    uint8_t backlight; // Backlight
    uint8_t mode_bits; // Display on/off control bits
    uint8_t entry_bits; // Entry mode set bits
} lcd_hd44780_params;

typedef enum {
    LCD_PARAM_UNSET = 0,
    LCD_PARAM_SET
} lcd_hd44780_params_action;

typedef enum {
    LCD_BACKLIGHT = 0,
    LCD_DISPLAY,
    LCD_CLEAR,
    LCD_CURSOR,
    LCD_CURSOR_BLINK,
    LCD_CURSOR_HOME,
    LCD_CURSOR_DIR_LEFT,
    LCD_CURSOR_DIR_RIGHT,
    LCD_DISPLAY_SHIFT
} lcd_hd44780_commands;

bool lcd_hd44780_init(lcd_hd44780_t* lcd, uint8_t address, uint8_t lines, uint8_t rows);
bool lcd_hd44780_command(lcd_hd44780_t* lcd, lcd_hd44780_commands command, lcd_hd44780_params_action action);
bool lcd_hd44780_backlight(lcd_hd44780_t* lcd, uint8_t command);
bool lcd_hd44780_set_cursor_position(lcd_hd44780_t* lcd, uint8_t line, uint8_t row);
bool lcd_hd44780_print_str(lcd_hd44780_t* lcd, uint8_t* data, uint8_t length);
bool lcd_hd44780_print_char(lcd_hd44780_t* lcd, uint8_t data);
bool lcd_hd44780_load_custom_char(lcd_hd44780_t* lcd, uint8_t cell, uint8_t* char_map);

bool lcd_hd44780_backlight_on(lcd_hd44780_t* lcd);
bool lcd_hd44780_backlight_off(lcd_hd44780_t* lcd);
bool lcd_hd44780_autoscroll_on(lcd_hd44780_t* lcd);
bool lcd_hd44780_autoscroll_off(lcd_hd44780_t* lcd);
bool lcd_hd44780_display_clear(lcd_hd44780_t* lcd);
bool lcd_hd44780_display_on(lcd_hd44780_t* lcd);
bool lcd_hd44780_display_off(lcd_hd44780_t* lcd);
bool lcd_hd44780_cursor_on(lcd_hd44780_t* lcd);
bool lcd_hd44780_cursor_off(lcd_hd44780_t* lcd);
bool lcd_hd44780_blink_on(lcd_hd44780_t* lcd);
bool lcd_hd44780_blink_off(lcd_hd44780_t* lcd);
bool lcd_hd44780_cursor_dir_to_right(lcd_hd44780_t* lcd);
bool lcd_hd44780_cursor_dir_to_left(lcd_hd44780_t* lcd);
bool lcd_hd44780_cursor_home(lcd_hd44780_t* lcd);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
