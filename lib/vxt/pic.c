// Copyright (c) 2019-2023 Andreas T Jonsson <mail@andreasjonsson.se>
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software in
//    a product, an acknowledgment (see the following) in the product
//    documentation is required.
//
//    Portions Copyright (c) 2019-2023 Andreas T Jonsson <mail@andreasjonsson.se>
//
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.

#include "vxt/vxt.h"
#include <vxt/vxtu.h>

struct pic {
	vxt_byte mask_reg;
    vxt_byte request_reg;
    vxt_byte service_reg;
    vxt_byte icw_step;
    vxt_byte read_mode;
	vxt_byte icw[5];

	vxt_word base;
	struct vxt_pirepheral *slave;
};

static vxt_byte in(struct pic *c, vxt_word port) {
	if (port == c->base)
        return c->read_mode ? c->service_reg : c->request_reg;
    return c->mask_reg;
}

static void out(struct pic *c, vxt_word port, vxt_byte data) {
	if (port == c->base) {
		if (data & 0x10) {
			c->icw_step = 1;
			c->mask_reg = 0;
			c->icw[c->icw_step++] = data;
			return;
		}

		if (((data & 0x98) == 8) && (data & 2))
			c->read_mode = data & 2;

		if (data & 0x20) {
			for (int i = 0; i < 8; i++) {
				if ((c->service_reg >> i) & 1) {
					c->service_reg ^= (1 << i);
					return;
				}
			}
		}
	} else {
		if ((c->icw_step == 3) && (c->icw[1] & 2))
			c->icw_step = 4;

		if (c->icw_step < 5) {
			c->icw[c->icw_step++] = data;
			return;
		}
		c->mask_reg = data;
	}
}

static int next(struct pic *c) {
    vxt_byte has = c->request_reg & (~c->mask_reg);

    for (vxt_byte i = 0; i < 8; i++) {
        vxt_byte mask = 1 << i;
        if (!(has & mask))
            continue;

        if ((c->request_reg & mask) && !(c->service_reg & mask)) {
            c->request_reg ^= mask;
            c->service_reg |= mask;

            if (!(c->icw[4] & 2)) // Not auto EOI?
                c->service_reg |= mask;

			if ((i == 2) && c->slave)
				return c->slave->pic.next(c->slave) + 8;
			else
			 	return (int)c->icw[2] + i;
        }
    }
    return -1;
}

static void irq(struct pic *c, int n) {
	if ((n > 7) && c->slave) {
		c->slave->pic.irq(c->slave, n);
		n = 2;
	}
    c->request_reg |= (vxt_byte)(1 << n);
}

static vxt_error install(struct pic *c, vxt_system *s) {
	struct vxt_pirepheral *p = VXT_GET_PIREPHERAL(c);
    vxt_system_install_io(s, p, c->base, c->base + 1);

	// Assume PC AT
	if (c->base == 0x20) {
		for (int i = 0; i < VXT_MAX_PIREPHERALS; i++) {
        	struct vxt_pirepheral *ip = vxt_system_pirepheral(s, (vxt_byte)i);
			if (!ip || (ip == p))
				continue;

        	if (vxt_pirepheral_class(ip) == VXT_PCLASS_PIC) {
				c->slave = ip;
				break;
			}
		}
	}
    return VXT_NO_ERROR;
}

static vxt_error reset(struct pic *c) {
    vxt_memclear(c, sizeof(struct pic));
    return VXT_NO_ERROR;
}

static enum vxt_pclass pclass(struct pic *c) {
    (void)c; return VXT_PCLASS_PIC;
}

static const char *name(struct pic *c) {
    (void)c; return "PIC (Intel 8259)";
}

VXT_API struct vxt_pirepheral *vxtu_pic_create(vxt_allocator *alloc, vxt_word base) VXT_PIREPHERAL_CREATE(alloc, pic, {
    DEVICE->base = base;

	PIREPHERAL->install = &install;
    PIREPHERAL->reset = &reset;
    PIREPHERAL->name = &name;
    PIREPHERAL->pclass = &pclass;
    PIREPHERAL->io.in = &in;
    PIREPHERAL->io.out = &out;
    PIREPHERAL->pic.next = &next;
    PIREPHERAL->pic.irq = &irq;
})
