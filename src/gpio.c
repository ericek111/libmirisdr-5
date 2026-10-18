/*
 * Copyright (C) 2026 by Bertold Van den Bergh <vandenbergh@bertold.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The four GPIO pins share register 0x08 with the band plan: val[15:12] is the
 * direction nibble, one bit per pin and 1 meaning output, and val[11:8] the
 * level driven when a pin is an output.  Every band plan word drives all four
 * as outputs, and the plan is rewritten on each retune, so a pin the
 * application wants for itself has to be held against it.  That is what the
 * override mask does: the plan still supplies the rest of the register.
 *
 * Pads are read back through the register read window, which is a separate
 * decode from the write registers - index 6 is the GPIO input nibble, and it
 * reports the pad, not the output latch, so a pin driven by something else can
 * be read while the plan still lists it as an output.
 */

/* Which pin the bias tee control drives, from the commit that added it for the
   "purple dongle" - one board's wiring, not something the part defines. */
#define BIAS_GPIO 3

/* The RSPdx-R2's direct sampling switch: high puts the antenna on the I converter */
#define DS_GPIO 2

/* The word before the pins the application holds: the plan's, with the bias and the
   direct sampling switch driven high. A raw GPIO word from the tune decides these too */
static uint32_t mirisdr_reg8_plan (mirisdr_dev_t *p)
{
    return p->reg8 | ((p->fe_raw & MIRISDR_FRONTEND_RAW_GPIO) ? 0 :
           ((p->bias && !MIRISDR_HW_EXPANDER(p->hw_flavour)) ? (1 << (BIAS_GPIO + 8)) : 0) |
           (p->ds ? (1 << (DS_GPIO + 8)) : 0));
}

static int update_reg_8 (mirisdr_dev_t *p)
{
    uint32_t val = mirisdr_reg8_plan(p);
    unsigned int pin;

    for (pin = 0; pin < MIRISDR_GPIO_COUNT; pin++)
    {
        if (!(p->gpio_mask & (1 << pin))) continue;

        val&= ~((1 << (pin + 12)) | (1 << (pin + 8)));
        if (p->gpio_dir & (1 << pin)) val|= 1 << (pin + 12);
        if (p->gpio_val & (1 << pin)) val|= 1 << (pin + 8);
    }

    if (p->reg8_valid && (p->reg8_sent == val)) return 0;

    return (mirisdr_write_reg(p, 0x08, val) < 0) ? -1 : 0;
}

static void mirisdr_gpio_hold_input (mirisdr_dev_t *p, uint8_t mask)
{
    if (!p || !mask) return;

    mask&= (1 << MIRISDR_GPIO_COUNT) - 1;

    p->gpio_hold|= mask;
    p->gpio_mask|= mask;
    p->gpio_dir&= ~mask;
    p->gpio_val&= ~mask;
}

/* from what the pin does now, so taking it over changes nothing yet */
static void mirisdr_gpio_claim (mirisdr_dev_t *p, unsigned int pin)
{
    uint32_t w = mirisdr_reg8_plan(p);

    if (p->gpio_mask & (1 << pin)) return;

    p->gpio_mask|= 1 << pin;
    if (w & (1 << (pin + 12))) p->gpio_dir|= 1 << pin;
    if (w & (1 << (pin + 8)))  p->gpio_val|= 1 << pin;
}

int mirisdr_set_gpio_direction (mirisdr_dev_t *p, unsigned int pin, int output)
{
    if (!p) return -1;
    if (pin >= MIRISDR_GPIO_COUNT) return -1;

    mirisdr_gpio_claim(p, pin);

    if (output) p->gpio_dir|= 1 << pin;
    else p->gpio_dir&= ~(1 << pin);

    return update_reg_8(p);
}

int mirisdr_get_gpio_direction (mirisdr_dev_t *p, unsigned int pin)
{
    if (!p) return -1;
    if (pin >= MIRISDR_GPIO_COUNT) return -1;

    if (p->gpio_mask & (1 << pin)) return (p->gpio_dir >> pin) & 1;

    return (mirisdr_reg8_plan(p) >> (pin + 12)) & 1;
}

int mirisdr_set_gpio_outputs (mirisdr_dev_t *p, unsigned int mask, unsigned int levels)
{
    unsigned int pin;

    if (!p) return -1;
    if (mask >> MIRISDR_GPIO_COUNT) return -1;

    for (pin = 0; pin < MIRISDR_GPIO_COUNT; pin++)
    {
        if (!(mask & (1 << pin))) continue;

        mirisdr_gpio_claim(p, pin);

        if (levels & (1 << pin)) p->gpio_val|= 1 << pin;
        else p->gpio_val&= ~(1 << pin);
    }

    return update_reg_8(p);
}

int mirisdr_set_gpio_output (mirisdr_dev_t *p, unsigned int pin, int high)
{
    if (pin >= MIRISDR_GPIO_COUNT) return -1;

    return mirisdr_set_gpio_outputs(p, 1 << pin, high ? 1 << pin : 0);
}

int mirisdr_get_gpio_output (mirisdr_dev_t *p, unsigned int pin)
{
    if (!p) return -1;
    if (pin >= MIRISDR_GPIO_COUNT) return -1;

    if (p->gpio_mask & (1 << pin)) return (p->gpio_val >> pin) & 1;

    return (mirisdr_reg8_plan(p) >> (pin + 8)) & 1;
}

int mirisdr_get_gpio_inputs (mirisdr_dev_t *p)
{
    uint8_t buf[4];

    if (!p) return -1;

    if (mirisdr_read_reg(p, 6, buf, sizeof(buf)) != (int) sizeof(buf)) return -1;

    return buf[0] & ((1 << MIRISDR_GPIO_COUNT) - 1);
}

int mirisdr_get_gpio_input (mirisdr_dev_t *p, unsigned int pin)
{
    int v;

    if (pin >= MIRISDR_GPIO_COUNT) return -1;

    v = mirisdr_get_gpio_inputs(p);

    return (v < 0) ? -1 : ((v >> pin) & 1);
}

int mirisdr_release_gpio (mirisdr_dev_t *p, unsigned int pin)
{
    if (!p) return -1;
    if (pin >= MIRISDR_GPIO_COUNT) return -1;

    p->gpio_mask&= ~(1 << pin);
    p->gpio_dir&= ~(1 << pin);
    p->gpio_val&= ~(1 << pin);

    return update_reg_8(p);
}
