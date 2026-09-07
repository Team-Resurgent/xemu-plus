/*
 * SmartXX / Aladdin character LCD
 *
 * The SmartXX and Aladdin XT modchips drive an HD44780 in 4-bit mode through
 * one register at I/O port 0xF700. The data nibble is scattered over bits
 * 6:3 (D6, D7, D4, D5), bit 2 is E, bit 1 is RS. The backlight level is a
 * write to 0xF701, a register the modchip owns because a read of it returns
 * the chip id, so the modchip hands those writes over through
 * lcd_smartxx_backlight_write(). There is no contrast control.
 *
 * Like the Xecuter LCD this is its own device, overlaid on the modchip's port
 * range, so it can be attached independently of the modchip model.
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

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "qapi/error.h"
#include "hw/xbox/lcd_hd44780.h"
#include "hw/xbox/lcd_smartxx.h"

#define LCD_SMARTXX_BASE        0xF700
#define LCD_SMARTXX_SIZE        1

#define LCD_SMARTXX_RS          0x02
#define LCD_SMARTXX_E           0x04

#define LCD_SMARTXX_IO_PRIORITY 1

typedef struct LcdSmartxxState {
    ISADevice dev;
    MemoryRegion io;
    HD44780State panel;

    /* Last value written, so reads see something plausible. */
    uint8_t reg;

    /* Nibble pairing, as in the Xecuter LCD: an HD44780 powers on in 8-bit
     * mode, and we assume 8-bit whenever the phase is unknown, so the host's
     * next wake-up strobes resynchronise us. */
    bool dl_8bit;
    bool lo_pending;
    uint8_t nibble_hi;
    bool latched_rs;
    uint8_t init_run;
} LcdSmartxxState;

#define TYPE_LCD_SMARTXX "lcd-smartxx"
#define LCD_SMARTXX(obj) \
    OBJECT_CHECK(LcdSmartxxState, (obj), TYPE_LCD_SMARTXX)

/* The one attached panel, if any; only one display can be fitted. */
static LcdSmartxxState *lcd_smartxx_attached;

void lcd_smartxx_backlight_write(uint8_t val)
{
    if (lcd_smartxx_attached) {
        /* Already scaled to 0-127 by the modchip, as the UI expects. */
        hd44780_set_backlight(&lcd_smartxx_attached->panel, val & 0x7F);
    }
}

/* Register bits 6:3 carry D6, D7, D4, D5 (PrometheOS writeValue:
 * (v >> 2) & 0x28 | v & 0x50). */
static uint8_t lcd_smartxx_nibble(uint8_t reg)
{
    return ((reg >> 5) & 1) << 3 |
           ((reg >> 6) & 1) << 2 |
           ((reg >> 3) & 1) << 1 |
           ((reg >> 4) & 1);
}

static void lcd_smartxx_byte(LcdSmartxxState *s, bool rs, uint8_t byte)
{
    /* A function set carries the interface width, and is the point at which
     * a host switching to 4-bit mode realigns our pairing. */
    if (!rs && (byte & 0xE0) == 0x20) {
        s->dl_8bit = (byte & 0x10) != 0;
        s->lo_pending = false;
    }
    hd44780_write_byte(&s->panel, rs, byte);
}

static void lcd_smartxx_strobe(LcdSmartxxState *s, uint8_t reg)
{
    uint8_t nibble = lcd_smartxx_nibble(reg);
    bool rs = (reg & LCD_SMARTXX_RS) != 0;

    /* Wake-up detector: three 0x3 command nibbles mean the host is resetting
     * the panel, whatever mode it believes it is in; the 0x2 that follows
     * realigns us. */
    if (!rs && nibble == 0x3) {
        if (s->init_run < 3) {
            s->init_run++;
        }
        if (s->init_run == 3) {
            s->dl_8bit = true;
            s->lo_pending = false;
        }
    } else {
        s->init_run = 0;
    }

    if (s->dl_8bit) {
        if (!rs) {
            lcd_smartxx_byte(s, false, nibble << 4);
        }
        return;
    }

    if (!s->lo_pending) {
        s->nibble_hi = nibble;
        s->latched_rs = rs;
        s->lo_pending = true;
        return;
    }
    s->lo_pending = false;
    lcd_smartxx_byte(s, s->latched_rs, (s->nibble_hi << 4) | nibble);
}

static void lcd_smartxx_io_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned int size)
{
    LcdSmartxxState *s = opaque;
    bool e_was_high = (s->reg & LCD_SMARTXX_E) != 0;
    bool e_is_high = (val & LCD_SMARTXX_E) != 0;

    s->reg = val;
    if (!e_was_high && e_is_high) {
        lcd_smartxx_strobe(s, val);
    }
}

static uint64_t lcd_smartxx_io_read(void *opaque, hwaddr addr,
                                    unsigned int size)
{
    LcdSmartxxState *s = opaque;

    return s->reg;
}

static const MemoryRegionOps lcd_smartxx_io_ops = {
    .read = lcd_smartxx_io_read,
    .write = lcd_smartxx_io_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void lcd_smartxx_realize(DeviceState *dev, Error **errp)
{
    LcdSmartxxState *s = LCD_SMARTXX(dev);
    ISADevice *isa = ISA_DEVICE(dev);

    hd44780_reset(&s->panel);
    hd44780_register_debug(&s->panel);
    /* Lit until the host sets a level, so an untouched panel is readable. */
    hd44780_set_backlight(&s->panel, 127);
    s->dl_8bit = true;
    lcd_smartxx_attached = s;

    memory_region_init_io(&s->io, OBJECT(s), &lcd_smartxx_io_ops, s,
                          "lcd-smartxx.io", LCD_SMARTXX_SIZE);
    memory_region_add_subregion_overlap(isa_address_space_io(isa),
                                        LCD_SMARTXX_BASE, &s->io,
                                        LCD_SMARTXX_IO_PRIORITY);
}

static void lcd_smartxx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "SmartXX / Aladdin character LCD";
    dc->realize = lcd_smartxx_realize;
}

static const TypeInfo lcd_smartxx_type_info = {
    .name          = TYPE_LCD_SMARTXX,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(LcdSmartxxState),
    .class_init    = lcd_smartxx_class_init,
};

static void lcd_smartxx_register_types(void)
{
    type_register_static(&lcd_smartxx_type_info);
}

type_init(lcd_smartxx_register_types)
