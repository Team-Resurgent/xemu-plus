/*
 * SmartXX character LCD
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_XBOX_LCD_SMARTXX_H
#define HW_XBOX_LCD_SMARTXX_H

#include <stdint.h>

/* The backlight register (0xF701) belongs to the modchip, whose reads of it
 * return the chip id, so the modchip passes writes on here, scaled to 0-127
 * for the chip's own backlight width. A no-op when no panel is attached. */
void lcd_smartxx_backlight_write(uint8_t val);

#endif
