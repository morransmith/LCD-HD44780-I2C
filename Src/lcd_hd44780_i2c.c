/**
 * Copyright Nikita Bulaev 2017
 *
 * STM32 HAL libriary for LCD display based on HITACHI HD44780U chip.
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

/*
    Here are some cyrillic symbols that you can use in your code

    uint8_t symD[8]   = { 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 }; // Д
    uint8_t symZH[8]  = { 0x11, 0x15, 0x15, 0x0E, 0x15, 0x15, 0x11 }; // Ж
    uint8_t symI[8]   = { 0x11, 0x11, 0x13, 0x15, 0x19, 0x11, 0x11 }; // И
    uint8_t symL[8]   = { 0x0F, 0x09, 0x09, 0x09, 0x09, 0x11, 0x11 }; // Л
    uint8_t symP[8]   = { 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }; // П
    uint8_t symSHi[8] = { 0x10, 0x15, 0x15, 0x15, 0x15, 0x1F, 0x03 }; // Щ
    uint8_t symJU[8]  = { 0x12, 0x15, 0x15, 0x1D, 0x15, 0x15, 0x12 }; // Ю
    uint8_t symJA[8]  = { 0x0F, 0x11, 0x11, 0x0F, 0x05, 0x09, 0x11 }; // Я


 */

#include "lcd_hd44780_i2c.h"
#include <stdlib.h>
#include <string.h>

bool lcd_hd44780_backlight_on(lcd_hd44780_t* lcd) { return lcd_hd44780_backlight(lcd, LCD_BIT_BACKIGHT_ON); }
bool lcd_hd44780_backlight_off(lcd_hd44780_t* lcd) { return lcd_hd44780_backlight(lcd, LCD_BIT_BACKIGHT_OFF); }
bool lcd_hd44780_autoscroll_on(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_DISPLAY_SHIFT, LCD_PARAM_SET); }
bool lcd_hd44780_autoscroll_off(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_DISPLAY_SHIFT, LCD_PARAM_UNSET); }
bool lcd_hd44780_display_clear(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CLEAR, LCD_PARAM_SET); }
bool lcd_hd44780_display_on(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_DISPLAY, LCD_PARAM_SET); }
bool lcd_hd44780_display_off(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_DISPLAY, LCD_PARAM_UNSET); }
bool lcd_hd44780_cursor_on(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CURSOR, LCD_PARAM_SET); }
bool lcd_hd44780_cursor_off(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CURSOR, LCD_PARAM_UNSET); }
bool lcd_hd44780_blink_on(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CURSOR_BLINK, LCD_PARAM_SET); }
bool lcd_hd44780_blink_off(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CURSOR_BLINK, LCD_PARAM_UNSET); }
bool lcd_hd44780_cursor_dir_to_right(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CURSOR_DIR_RIGHT, LCD_PARAM_SET); }
bool lcd_hd44780_cursor_dir_to_left(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CURSOR_DIR_LEFT, LCD_PARAM_SET); }
bool lcd_hd44780_cursor_home(lcd_hd44780_t* lcd) { return lcd_hd44780_command(lcd, LCD_CURSOR_HOME, LCD_PARAM_SET); }

static bool lcd_hd44780_write_byte(lcd_hd44780_t* lcd, uint8_t rs_rw_bits, uint8_t* data);

/**
 * @brief  Turn display on and init it params
 * @note   We gonna make init steps according to datasheep page 46.
 *         There are 4 steps to turn 4-bits mode on,
 *         then we send initial params.
 * @param  hi2c    I2C struct to which display is connected
 * @param  address Display I2C 7-bit address
 * @param  lines   Number of lines of display
 * @param  columns Number of colums
 * @return         true if success
 */
bool lcd_hd44780_init(lcd_hd44780_t* lcd, uint8_t address, uint8_t lines, uint8_t columns)
{

    uint8_t lcd_data = LCD_BIT_5x8DOTS;

    lcd->lcd_params.address = address << 1;
    lcd->lcd_params.lines = lines;
    lcd->lcd_params.columns = columns;
    lcd->lcd_params.backlight = LCD_BIT_BACKIGHT_ON;

    memset(lcd->buffer, 0, sizeof(lcd->buffer));

    lcd->buffer[0] = LCD_BIT_E | (0x03 << 4);
    lcd->buffer[1] = lcd->buffer[0];
    lcd->buffer[2] = (0x03 << 4);

    /* First 3 steps of init cycles. They are the same. */
    for (uint8_t i = 0; i < 3; ++i) {
        if (lcd->transmit(lcd->lcd_params.address, (uint8_t*)lcd->buffer, 3))
            return false;

        if (i != 2)
            // For first 2 cycles delay is less then 5ms (4100us by datasheet)
            lcd->delay(5);
        else
            // For the last cycle delay is less then 1 ms (100us by datasheet)
            lcd->delay(1);
    }

    /* Lets turn to 4-bit at least */
    lcd->buffer[0] = LCD_BIT_BACKIGHT_ON | LCD_BIT_E | (LCD_MODE_4BITS << 4);
    lcd->buffer[1] = lcd->buffer[0];
    lcd->buffer[2] = LCD_BIT_BACKIGHT_ON | (LCD_MODE_4BITS << 4);
    if (lcd->transmit(lcd->lcd_params.address, (uint8_t*)lcd->buffer, 3))
        return false;

    /* Lets set display params */
    /* First of all lets set display size */
    lcd_data |= LCD_MODE_4BITS;

    if (lcd->lcd_params.lines > 1) {
        lcd_data |= LCD_BIT_2LINE;
    }

    lcd_hd44780_write_byte(lcd, 0x00, &lcd_data); // TODO: Make 5x10 dots font usable for some 1-line display

    /* Now lets set display, cursor and blink all on */
    lcd_hd44780_display_on(lcd);

    /* Set cursor moving to the right */
    lcd_hd44780_cursor_dir_to_right(lcd);

    /* Clear display and Set cursor at Home */
    lcd_hd44780_display_clear(lcd);
    lcd_hd44780_cursor_home(lcd);

    return true;
}

/**
 * @brief  Send command to display
 * @param  command  One of listed in lcd_hd44780_commands enum
 * @param  action   LCD_PARAM_SET or LCD_PARAM_UNSET
 * @return          true if success
 */
bool lcd_hd44780_command(lcd_hd44780_t* lcd, lcd_hd44780_commands command, lcd_hd44780_params_action action)
{
    uint8_t lcd_data = 0x00;

    /* First of all lest store the command */
    switch (action) {
    case LCD_PARAM_SET:
        switch (command) {
        case LCD_DISPLAY:
            lcd->lcd_params.mode_bits |= LCD_BIT_DISPLAY_ON;
            break;

        case LCD_CURSOR:
            lcd->lcd_params.mode_bits |= LCD_BIT_CURSOR_ON;
            break;

        case LCD_CURSOR_BLINK:
            lcd->lcd_params.mode_bits |= LCD_BIT_BLINK_ON;
            break;

        case LCD_CLEAR:
            lcd_data = LCD_BIT_DISP_CLEAR;

            if (!lcd_hd44780_write_byte(lcd, 0x00, &lcd_data))
                return false;

            lcd->delay(2);
            return true;

        case LCD_CURSOR_HOME:
            lcd_data = LCD_BIT_CURSOR_HOME;

            if (!lcd_hd44780_write_byte(lcd, 0x00, &lcd_data))
                return false;

            lcd->delay(2);
            return true;

        case LCD_CURSOR_DIR_RIGHT:
            lcd->lcd_params.entry_bits |= LCD_BIT_CURSOR_DIR_RIGHT;
            break;

        case LCD_CURSOR_DIR_LEFT:
            lcd->lcd_params.entry_bits |= LCD_BIT_CURSOR_DIR_LEFT;
            break;

        case LCD_DISPLAY_SHIFT:
            lcd->lcd_params.entry_bits |= LCD_BIT_DISPLAY_SHIFT;
            break;

        default:
            return false;
        }

        break;

    case LCD_PARAM_UNSET:
        switch (command) {
        case LCD_DISPLAY:
            lcd->lcd_params.mode_bits &= ~LCD_BIT_DISPLAY_ON;
            break;

        case LCD_CURSOR:
            lcd->lcd_params.mode_bits &= ~LCD_BIT_CURSOR_ON;
            break;

        case LCD_CURSOR_BLINK:
            lcd->lcd_params.mode_bits &= ~LCD_BIT_BLINK_ON;
            break;

        case LCD_CURSOR_DIR_RIGHT:
            lcd->lcd_params.entry_bits &= ~LCD_BIT_CURSOR_DIR_RIGHT;
            break;

        case LCD_CURSOR_DIR_LEFT:
            lcd->lcd_params.entry_bits &= ~LCD_BIT_CURSOR_DIR_LEFT;
            break;

        case LCD_DISPLAY_SHIFT:
            lcd->lcd_params.entry_bits &= ~LCD_BIT_DISPLAY_SHIFT;
            break;

        default:
            return false;
        }

        break;

    default:
        return false;
    }

    /* Now lets send the command */
    switch (command) {
    case LCD_DISPLAY:
    case LCD_CURSOR:
    case LCD_CURSOR_BLINK:
        lcd_data = LCD_BIT_DISPLAY_CONTROL | lcd->lcd_params.mode_bits;
        break;

    case LCD_CURSOR_DIR_RIGHT:
    case LCD_CURSOR_DIR_LEFT:
    case LCD_DISPLAY_SHIFT:
        lcd_data = LCD_BIT_ENTRY_MODE | lcd->lcd_params.entry_bits;
        break;

    default:
        break;
    }

    return lcd_hd44780_write_byte(lcd, 0x00, &lcd_data);
}

/**
 * @brief  Turn display's Backlight On or Off
 * @param  command LCD_BIT_BACKIGHT_ON to turn display On
 *                 LCD_BIT_BACKIGHT_OFF (or 0x00) to turn display Off
 * @return         true if success
 */
bool lcd_hd44780_backlight(lcd_hd44780_t* lcd, uint8_t command)
{
    lcd->lcd_params.backlight = command;

    if (lcd->transmit(lcd->lcd_params.address, &lcd->lcd_params.backlight, 1))
        return false;

    return true;
}

/**
 * @brief  Set cursor position on the display
 * @param  column counting from 0
 * @param  line   counting from 0
 * @return        true if success
 */
bool lcd_hd44780_set_cursor_position(lcd_hd44780_t* lcd, uint8_t column, uint8_t line)
{
    // We will setup offsets for 4 lines maximum
    static const uint8_t line_offsets[4] = { 0x00, 0x40, 0x14, 0x54 };

    if (line >= lcd->lcd_params.lines)
        line = lcd->lcd_params.lines - 1;

    uint8_t lcd_command = LCD_BIT_SETDDRAMADDR | (column + line_offsets[line]);

    return lcd_hd44780_write_byte(lcd, 0x00, &lcd_command);
}

/**
 * @brief  Print string from cursor position
 * @param  data   Pointer to string
 * @param  length Number of symbols to print
 * @return        true if success
 */
bool lcd_hd44780_print_str(lcd_hd44780_t* lcd, uint8_t* data, uint8_t length)
{
    for (uint8_t i = 0; i < length; ++i)
        if (!lcd_hd44780_write_byte(lcd, LCD_BIT_RS, &data[i]))
            return false;

    return true;
}

/**
 * @brief  Print single char at cursor position
 * @param  data Symbol to print
 * @return      true if success
 */
bool lcd_hd44780_print_char(lcd_hd44780_t* lcd, uint8_t data)
{
    return lcd_hd44780_write_byte(lcd, LCD_BIT_RS, &data);
}

/**
 * @brief Loading custom Chars to one of the 8 cells in CGRAM
 * @note  You can create your custom chars according to
 *        documentation page 15.
 *        It consists of array of 8 bytes.
 *        Each byte is line of dots. Lower bits are dots.
 * @param  cell     Number of cell from 0 to 7 where to upload
 * @param  char_map  Pointer to Array of dots
 *                  Example: { 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 }
 * @return          true if success
 */
bool lcd_hd44780_load_custom_char(lcd_hd44780_t* lcd, uint8_t cell, uint8_t* char_map)
{

    // Stop, if trying to load to incorrect cell
    if (cell > 7)
        return false;

    uint8_t lcd_command = LCD_BIT_SETCGRAMADDR | (cell << 3);

    if (!lcd_hd44780_write_byte(lcd, 0x00, &lcd_command))
        return false;

    for (uint8_t i = 0; i < 8; ++i)
        if (!lcd_hd44780_write_byte(lcd, LCD_BIT_RS, &char_map[i]) == false)
            return false;

    return true;
}

/**
 * @brief  Local function to send data to display
 * @param  rs_rw_bits State of RS and R/W bits
 * @param  data     Pointer to byte to send
 * @return          true if success
 */
static bool lcd_hd44780_write_byte(lcd_hd44780_t* lcd, uint8_t rs_rw_bits, uint8_t* data)
{

    /* Higher 4 bits*/
    lcd->buffer[0] = rs_rw_bits | LCD_BIT_E | lcd->lcd_params.backlight | (*data & 0xF0); // Send data and set strobe
    lcd->buffer[1] = lcd->buffer[0]; // Strobe turned on
    lcd->buffer[2] = rs_rw_bits | lcd->lcd_params.backlight | (*data & 0xF0); // Turning strobe off

    /* Lower 4 bits*/
    lcd->buffer[3] = rs_rw_bits | LCD_BIT_E | lcd->lcd_params.backlight | ((*data << 4) & 0xF0); // Send data and set strobe
    lcd->buffer[4] = lcd->buffer[3]; // Strobe turned on
    lcd->buffer[5] = rs_rw_bits | lcd->lcd_params.backlight | ((*data << 4) & 0xF0); // Turning strobe off

    if (lcd->transmit(lcd->lcd_params.address, (uint8_t*)lcd->buffer, 6))
        return false;

    lcd->delay(1);

    return true;
}
