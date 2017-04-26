/*
 * QEMU RIVA128 VGA Emulator.
 *
 * Copyright (c) 2003 Fabrice Bellard
 * Copyright (c) 2017 Melissa Goad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "hw/hw.h"
#include "ui/console.h"
#include "hw/pci/pci.h"
#include "vga_int.h"
#include "ui/pixel_ops.h"
#include "qemu/timer.h"
#include "hw/loader.h"

#define PCI_RIVA128_IOPORT_OFFSET 0x400
#define PCI_RIVA128_IOPORT_SIZE   (0x3e0 - 0x3c0)
#define PCI_RIVA128_MMIO_SIZE     0x1000000

typedef struct RIVA128State {
    PCIDevice dev;
    VGACommonState vga;
    uint32_t flags;
    MemoryRegion mmio;
    MemoryRegion mrs[3];

    struct
    {
        struct
        {
            bool sda, scl; //Basic I2C pins. Yes, the hardware actually works like this.
        } i2c;
    } riva128;
} PCIRIVA128State;

#define TYPE_PCI_RIVA128 "pci-riva128"
#define PCI_RIVA128(obj) OBJECT_CHECK(PCIRIVA128State, (obj), TYPE_PCI_RIVA128)

static const VMStateDescription vmstate_riva128_pci = {
    .name = "riva128",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, PCIRIVA128State),
        VMSTATE_STRUCT(riva128, PCIRIVA128State, 0, vmstate_riva128_common, RIVA128CommonState),
        VMSTATE_END_OF_LIST()
    }
};

static uint64_t riva128_ioport_read(PCIRIVA128State *s, hwaddr addr)
{
    uint64_t ret = 0;

    switch (addr) {
    case 0x3c0: case 0x3c1: case 0x3c2: case 0x3c3: case 0x3c4: case 0x3c5: case 0x3c6: case 0x3c7:
    case 0x3c8: case 0x3c9: case 0x3ca: case 0x3cc: case 0x3ce: case 0x3cf: case 0x3d4: case 0x3da:
        ret = vga_ioport_read(&s->vga, addr);
        break;
    case 0x3d5:
        switch (s->vga.cr_index) {
        case 0x3e:
            //RIVA 128 I2C Read register
            ret = (s->riva128.i2c.sda << 3) | (s->riva128.i2c.scl << 2);
            break;
        default:
            ret = vga_ioport_read(&s->vga, addr);
            break;
        }
        break;
    }
    return ret;
}

static uint64_t riva128_ioport_write(PCIRIVA128State *s, hwaddr addr, uint8_t val)
{
    switch (addr) {
    case 0x3c0: case 0x3c1: case 0x3c2: case 0x3c3: case 0x3c4: case 0x3c5: case 0x3c6: case 0x3c7:
    case 0x3c8: case 0x3c9: case 0x3ca: case 0x3cc: case 0x3ce: case 0x3cf: case 0x3d4: case 0x3da:
        ret = vga_ioport_write(&s->vga, addr, val);
        break;
    case 0x3d5:
        switch (s->vga.cr_index) {
        case 0x3f:
            //RIVA 128 I2C Write register
            s->riva128.i2c.scl = val & 0x20;
            s->riva128.i2c.sda = val & 0x10;
            break;
        default:
            vga_ioport_write(&s->vga, addr, val);
            break;
        }
        break;
    }
    return ret;
}

static uint64_t pci_riva128_ioport_read(void *ptr, hwaddr addr,
                                    unsigned size)
{
    PCIRIVA128State *s = ptr;
    uint64_t ret = 0;

    switch (size) {
    case 1:
        ret = riva128_ioport_read(s, addr + 0x3c0);
        break;
    case 2:
        ret  = riva128_ioport_read(s, addr + 0x3c0);
        ret |= riva128_ioport_read(s, addr + 0x3c1) << 8;
        break;
    }
    return ret;
}

static void pci_riva128_ioport_write(void *ptr, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    PCIRIVA128State *s = ptr;

    switch (size) {
    case 1:
        riva128_ioport_write(s, addr + 0x3c0, val);
        break;
    case 2:
        /*
         * Update bytes in little endian order.  Allows to update
         * indexed registers with a single word write because the
         * index byte is updated first.
         */
        riva128_ioport_write(s, addr + 0x3c0, val & 0xff);
        riva128_ioport_write(s, addr + 0x3c1, (val >> 8) & 0xff);
        break;
    }
}

static const MemoryRegionOps pci_riva128_ioport_ops = {
    .read = pci_riva128_ioport_read,
    .write = pci_riva128_ioport_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static bool riva128_get_big_endian_fb(Object *obj, Error **errp)
{
    PCIRIVA128State *d = PCI_VGA(PCI_DEVICE(obj));

    return d->riva128.big_endian_fb;
}

static void riva128_set_big_endian_fb(Object *obj, bool value, Error **errp)
{
    PCIRIVA128State *d = PCI_VGA(PCI_DEVICE(obj));

    d->riva128.big_endian_fb = value;
}

void pci_std_riva128_mmio_region_init(VGACommonState *s,
                                  MemoryRegion *parent,
                                  MemoryRegion *subs)
{
    memory_region_init_io(&subs[0], NULL, &pci_riva128_ioport_ops, s,
                          "riva128 ioports remapped", PCI_RIVA128_IOPORT_SIZE);
    memory_region_add_subregion(parent, PCI_RIVA128_IOPORT_OFFSET,
                                &subs[0]);
}

static void pci_std_riva128_realize(PCIDevice *dev, Error **errp)
{
    PCIRIVA128State *d = PCI_RIVA128(dev);
    VGACommonState *s = &d->riva128;
    bool qext = false;

    /* riva128 + console init */
    vga_common_init(s, OBJECT(dev), true);
    vga_init(s, OBJECT(dev), pci_address_space(dev), pci_address_space_io(dev),
             true);

    s->con = graphic_console_init(DEVICE(dev), 0, s->hw_ops, s);

    /* XXX: VGA_RAM_SIZE must be a power of two */
    pci_register_bar(&d->dev, 0, PCI_BASE_ADDRESS_MEM_PREFETCH, &s->vram);

    /* mmio bar for riva128 register access */
    if (d->flags & (1 << PCI_RIVA128_FLAG_ENABLE_MMIO)) {
        memory_region_init(&d->mmio, NULL, "riva128.mmio", PCI_RIVA128_MMIO_SIZE);

        pci_std_riva128_mmio_region_init(s, &d->mmio, d->mrs);

        pci_register_bar(&d->dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);
    }

    if (!dev->rom_bar) {
        /* compatibility with pc-0.13 and older */
        vga_init_vbe(s, OBJECT(dev), pci_address_space(dev));
    }
}

static void pci_std_riva128_init(Object *obj)
{
    /* Expose framebuffer byteorder via QOM */
    object_property_add_bool(obj, "big-endian-framebuffer",
                             riva128_get_big_endian_fb, riva128_set_big_endian_fb, NULL);
}

static void pci_secondary_riva128_realize(PCIDevice *dev, Error **errp)
{
    PCIRIVA128State *d = PCI_VGA(dev);
    VGACommonState *s = &d->riva128;
    bool qext = false;

    /* riva128 + console init */
    vga_common_init(s, OBJECT(dev), false);
    s->con = graphic_console_init(DEVICE(dev), 0, s->hw_ops, s);

    /* mmio bar */
    memory_region_init(&d->mmio, OBJECT(dev), "riva128.mmio", PCI_RIVA128_MMIO_SIZE);

    pci_std_riva128_mmio_region_init(s, &d->mmio, d->mrs);

    pci_register_bar(&d->dev, 0, PCI_BASE_ADDRESS_MEM_PREFETCH, &s->vram);
    pci_register_bar(&d->dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);
}

static void pci_secondary_riva128_init(Object *obj)
{
    /* Expose framebuffer byteorder via QOM */
    object_property_add_bool(obj, "big-endian-framebuffer",
                             riva128_get_big_endian_fb, riva128_set_big_endian_fb, NULL);
}

static void pci_secondary_riva128_reset(DeviceState *dev)
{
    PCIRIVA128State *d = PCI_VGA(PCI_DEVICE(dev));
    vga_common_reset(&d->vga);
}

static Property riva128_pci_properties[] = {
    DEFINE_PROP_UINT32("riva128mem_mb", PCIRIVA128State, riva128.vram_size_mb, 4),
    DEFINE_PROP_BIT("mmio", PCIRIVA128State, flags, PCI_VGA_FLAG_ENABLE_MMIO, true),
    DEFINE_PROP_END_OF_LIST(),
};

static Property secondary_pci_properties[] = {
    DEFINE_PROP_UINT32("riva128mem_mb", PCIRIVA128State, riva128.vram_size_mb, 4),
    DEFINE_PROP_END_OF_LIST(),
};

static void riva128_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->vendor_id = PCI_VENDOR_ID_NVIDIA_SGS;
    k->device_id = PCI_DEVICE_ID_NVIDIA_SGS_RIVA128;
    dc->vmsd = &vmstate_riva128_pci;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo riva128_pci_type_info = {
    .name = TYPE_PCI_VGA,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCIRIVA128State),
    .abstract = true,
    .class_init = riva128_pci_class_init,
};

static void riva128_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = pci_std_riva128_realize;
    k->romfile = "riva128bios.bin";
    k->class_id = PCI_CLASS_DISPLAY_VGA;
    dc->props = riva128_pci_properties;
    dc->hotpluggable = false;
}

static void secondary_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = pci_secondary_riva128_realize;
    k->class_id = PCI_CLASS_DISPLAY_OTHER;
    dc->props = secondary_pci_properties;
    dc->reset = pci_secondary_riva128_reset;
}

static const TypeInfo riva128_info = {
    .name          = "riva128",
    .parent        = TYPE_PCI_RIVA128,
    .instance_init = pci_std_riva128_init,
    .class_init    = riva128_class_init,
};

static const TypeInfo secondary_info = {
    .name          = "secondary-riva128",
    .parent        = TYPE_PCI_RIVA128,
    .instance_init = pci_secondary_riva128_init,
    .class_init    = secondary_class_init,
};

static void riva128_register_types(void)
{
    type_register_static(&riva128_pci_type_info);
    type_register_static(&riva128_info);
    type_register_static(&secondary_info);
}

type_init(riva128_register_types)
