#[macro_use]
extern crate bitfield;
#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate log;

use num::FromPrimitive;
use serde::{Deserialize, Serialize};

const TIMING_VBLANK: u16 = 1;
const TIMING_HBLANK: u16 = 2;

pub trait DmaNotifer {
    fn notify(&mut self, timing: u16);
}

use self::consts::*;
use self::interrupt::{Interrupt, InterruptConnect, SharedInterruptFlags};
use self::sched::{EventType, FutureEvent, GpuEvent, Scheduler};

macro_rules! index2d {
    ($x:expr, $y:expr, $w:expr) => {
        $w * $y + $x
    };
    ($t:ty, $x:expr, $y:expr, $w:expr) => {
        (($w as $t) * ($y as $t) + ($x as $t)) as $t
    };
}

use render::Point;

mod interrupt;
mod layer;
mod render;
mod rgb15;
mod sched;
mod sfx;
mod window;

use rgb15::Rgb15;
use window::*;

mod regs;
use regs::*;

mod consts {
    pub const PALRAM_ADDR: u32 = 0x0500_0000;
    pub const VRAM_ADDR: u32 = 0x0600_0000;
    pub const OAM_ADDR: u32 = 0x0700_0000;

    pub const PAGE_PALRAM: usize = (PALRAM_ADDR >> 24) as usize;
    pub const PAGE_VRAM: usize = (VRAM_ADDR >> 24) as usize;
    pub const PAGE_OAM: usize = (OAM_ADDR >> 24) as usize;

    pub const VIDEO_RAM_SIZE: usize = 128 * 1024;
    pub const PALETTE_RAM_SIZE: usize = 1024;
    pub const OAM_SIZE: usize = 1024;

    pub const DISPLAY_WIDTH: usize = 240;
    pub const DISPLAY_HEIGHT: usize = 160;
    pub const VBLANK_LINES: usize = 68;

    pub const CYCLES_HDRAW: usize = 960 + 46;
    pub const CYCLES_HBLANK: usize = 272 - 46;

    pub const TILE_SIZE: u32 = 0x20;

    pub const VRAM_OBJ_TILES_START_TEXT: u32 = 0x1_0000;
    pub const VRAM_OBJ_TILES_START_BITMAP: u32 = 0x1_4000;
}

use debug_stub_derive::DebugStub;
use enum_primitive_derive::Primitive;
use memory::{Addr, BusIO};

#[derive(Debug, Primitive, Copy, Clone)]
enum PixelFormat {
    BPP4 = 0,
    BPP8 = 1,
}

#[derive(Debug, Default, Copy, Clone)]
struct AffineMatrix {
    pa: i32,
    pb: i32,
    pc: i32,
    pd: i32,
}

#[derive(Serialize, Deserialize, Debug, Default, Copy, Clone)]
pub struct BgAffine {
    pub pa: i16, // dx
    pub pb: i16, // dmx
    pub pc: i16, // dy
    pub pd: i16, // dmy
    pub x: i32,
    pub y: i32,
    pub internal_x: i32,
    pub internal_y: i32,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
struct ObjBufferEntry {
    window: bool,
    alpha: bool,
    color: Rgb15,
    priority: u16,
}

impl Default for ObjBufferEntry {
    fn default() -> ObjBufferEntry {
        ObjBufferEntry {
            window: false,
            alpha: false,
            color: Rgb15::TRANSPARENT,
            priority: 4,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, DebugStub)]
pub struct Gpu {
    interrupt_flags: SharedInterruptFlags,

    // registers
    pub vcount: usize, // VCOUNT
    pub dispcnt: DisplayControl,
    pub dispstat: DisplayStatus,
    pub bgcnt: [BgControl; 4],
    pub bg_vofs: [u16; 4],
    pub bg_hofs: [u16; 4],
    pub bg_aff: [BgAffine; 2],
    pub win0: Window,
    pub win1: Window,
    pub winout_flags: WindowFlags,
    pub winobj_flags: WindowFlags,
    pub mosaic: RegMosaic,
    pub bldcnt: BlendControl,
    pub bldalpha: BlendAlpha,
    pub bldy: u16,

    palette_ram: Box<[u8]>,
    vram: Box<[u8]>,
    oam: Box<[u8]>,

    vram_obj_tiles_start: u32,
    obj_buffer: Box<[ObjBufferEntry]>,
    frame_buffer: Box<[u32]>,
    bg_line: [Box<[Rgb15]>; 4],
}

impl InterruptConnect for Gpu {
    fn connect_irq(&mut self, interrupt_flags: SharedInterruptFlags) {
        self.interrupt_flags = interrupt_flags;
    }
}

type FutureGpuEvent = (GpuEvent, usize);

impl Gpu {
    pub fn new(sched: &mut Scheduler, interrupt_flags: SharedInterruptFlags) -> Gpu {
        sched.schedule((EventType::Gpu(GpuEvent::HDraw), CYCLES_HDRAW));

        fn alloc_scanline_buffer() -> Box<[Rgb15]> {
            vec![Rgb15::TRANSPARENT; DISPLAY_WIDTH].into_boxed_slice()
        }

        Gpu {
            interrupt_flags,
            dispcnt: DisplayControl::from(0x80),
            dispstat: Default::default(),
            bgcnt: Default::default(),
            bg_vofs: [0; 4],
            bg_hofs: [0; 4],
            bg_aff: [BgAffine::default(); 2],
            win0: Window::default(),
            win1: Window::default(),
            winout_flags: WindowFlags::from(0),
            winobj_flags: WindowFlags::from(0),
            mosaic: RegMosaic(0),
            bldcnt: BlendControl::default(),
            bldalpha: BlendAlpha::default(),
            bldy: 0,

            vcount: 0,
            palette_ram: vec![0; PALETTE_RAM_SIZE].into_boxed_slice(),
            vram: vec![0; VIDEO_RAM_SIZE].into_boxed_slice(),
            oam: vec![0; OAM_SIZE].into_boxed_slice(),
            obj_buffer: vec![Default::default(); DISPLAY_WIDTH * DISPLAY_HEIGHT].into_boxed_slice(),
            frame_buffer: vec![0; DISPLAY_WIDTH * DISPLAY_HEIGHT].into_boxed_slice(),
            bg_line: [
                alloc_scanline_buffer(),
                alloc_scanline_buffer(),
                alloc_scanline_buffer(),
                alloc_scanline_buffer(),
            ],
            vram_obj_tiles_start: VRAM_OBJ_TILES_START_TEXT,
        }
    }

    #[inline]
    pub fn write_dispcnt(&mut self, value: u16) {
        let old_mode = self.dispcnt.mode;
        self.dispcnt.write(value);
        let new_mode = self.dispcnt.mode;
        if old_mode != new_mode {
            debug!("[GPU] Display mode changed! {} -> {}", old_mode, new_mode);
            self.vram_obj_tiles_start = if new_mode >= 3 {
                VRAM_OBJ_TILES_START_BITMAP
            } else {
                VRAM_OBJ_TILES_START_TEXT
            };
        }
    }

    pub fn skip_bios(&mut self) {
        for i in 0..2 {
            self.bg_aff[i].pa = 0x100;
            self.bg_aff[i].pb = 0;
            self.bg_aff[i].pc = 0;
            self.bg_aff[i].pd = 0x100;
        }
    }

    /// helper method that reads the palette index from a base address and x + y
    fn read_pixel_index(&mut self, addr: u32, x: u32, y: u32, format: PixelFormat) -> usize {
        match format {
            PixelFormat::BPP4 => self.read_pixel_index_bpp4(addr, x, y),
            PixelFormat::BPP8 => self.read_pixel_index_bpp8(addr, x, y),
        }
    }

    #[inline]
    fn read_pixel_index_bpp4(&mut self, addr: u32, x: u32, y: u32) -> usize {
        let ofs = addr + index2d!(u32, x / 2, y, 4);
        let ofs = ofs as usize;
        let byte = self.vram.read_8(ofs as u32);
        if x & 1 != 0 {
            (byte >> 4) as usize
        } else {
            (byte & 0xf) as usize
        }
    }

    #[inline]
    fn read_pixel_index_bpp8(&mut self, addr: u32, x: u32, y: u32) -> usize {
        let ofs = addr;
        self.vram.read_8(ofs + index2d!(u32, x, y, 8)) as usize
    }

    #[inline(always)]
    fn get_palette_color(&mut self, index: u32, palette_bank: u32, offset: u32) -> Rgb15 {
        if index == 0 || (palette_bank != 0 && index % 16 == 0) {
            return Rgb15::TRANSPARENT;
        }
        let value = self
            .palette_ram
            .read_16(offset + 2 * index + 0x20 * palette_bank);

        // top bit is ignored
        Rgb15(value & 0x7FFF)
    }

    #[inline]
    fn obj_buffer_get(&self, x: usize, y: usize) -> &ObjBufferEntry {
        &self.obj_buffer[index2d!(x, y, DISPLAY_WIDTH)]
    }

    #[inline]
    fn obj_buffer_get_mut(&mut self, x: usize, y: usize) -> &mut ObjBufferEntry {
        &mut self.obj_buffer[index2d!(x, y, DISPLAY_WIDTH)]
    }

    fn get_ref_point(&self, bg: usize) -> Point {
        assert!(bg == 2 || bg == 3);
        (
            self.bg_aff[bg - 2].internal_x,
            self.bg_aff[bg - 2].internal_y,
        )
    }

    fn render_scanline(&mut self) {
        if self.dispcnt.force_blank {
            for x in self.frame_buffer[self.vcount * DISPLAY_WIDTH..]
                .iter_mut()
                .take(DISPLAY_WIDTH)
            {
                *x = 0xf8f8f8;
            }
            return;
        }

        if self.dispcnt.enable_obj {
            self.render_objs();
        }
        match self.dispcnt.mode {
            0 => {
                for bg in 0..=3 {
                    if self.dispcnt.enable_bg[bg] {
                        self.render_reg_bg(bg);
                    }
                }
                self.finalize_scanline(0, 3);
            }
            1 => {
                if self.dispcnt.enable_bg[2] {
                    self.render_aff_bg(2);
                }
                if self.dispcnt.enable_bg[1] {
                    self.render_reg_bg(1);
                }
                if self.dispcnt.enable_bg[0] {
                    self.render_reg_bg(0);
                }
                self.finalize_scanline(0, 2);
            }
            2 => {
                if self.dispcnt.enable_bg[3] {
                    self.render_aff_bg(3);
                }
                if self.dispcnt.enable_bg[2] {
                    self.render_aff_bg(2);
                }
                self.finalize_scanline(2, 3);
            }
            3 => {
                self.render_mode3(2);
                self.finalize_scanline(2, 2);
            }
            4 => {
                self.render_mode4(2);
                self.finalize_scanline(2, 2);
            }
            5 => {
                self.render_mode5(2);
                self.finalize_scanline(2, 2);
            }
            _ => panic!("{:?} not supported", self.dispcnt.mode),
        }
    }

    /// Clears the gpu obj buffer
    fn obj_buffer_reset(&mut self) {
        for x in self.obj_buffer.iter_mut() {
            *x = Default::default();
        }
    }

    pub fn get_frame_buffer(&self) -> &[u32] {
        &self.frame_buffer
    }

    #[inline]
    fn update_vcount(&mut self, value: usize) {
        self.vcount = value;
        let vcount_setting = self.dispstat.vcount_setting;
        self.dispstat.vcount_flag = vcount_setting == self.vcount;

        if self.dispstat.vcount_irq_enable && self.dispstat.vcount_flag {
            interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_VCounterMatch);
        }
    }

    #[inline]
    fn handle_hdraw_end<D: DmaNotifer>(&mut self, dma_notifier: &mut D) -> FutureGpuEvent {
        self.dispstat.hblank_flag = true;
        if self.dispstat.hblank_irq_enable {
            interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_HBlank);
        };
        dma_notifier.notify(TIMING_HBLANK);

        // Next event
        (GpuEvent::HBlank, CYCLES_HBLANK)
    }

    fn handle_hblank_end<D: DmaNotifer>(&mut self, dma_notifier: &mut D) -> FutureGpuEvent {
        self.update_vcount(self.vcount + 1);

        if self.vcount < DISPLAY_HEIGHT {
            self.dispstat.hblank_flag = false;
            self.render_scanline();
            // update BG2/3 reference points on the end of a scanline
            for i in 0..2 {
                self.bg_aff[i].internal_x += self.bg_aff[i].pb as i32;
                self.bg_aff[i].internal_y += self.bg_aff[i].pd as i32;
            }

            (GpuEvent::HDraw, CYCLES_HDRAW)
        } else {
            // latch BG2/3 reference points on vblank
            for i in 0..2 {
                self.bg_aff[i].internal_x = self.bg_aff[i].x;
                self.bg_aff[i].internal_y = self.bg_aff[i].y;
            }

            self.dispstat.vblank_flag = true;
            self.dispstat.hblank_flag = false;
            if self.dispstat.vblank_irq_enable {
                interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_VBlank);
            };

            dma_notifier.notify(TIMING_VBLANK);

            self.obj_buffer_reset();

            (GpuEvent::VBlankHDraw, CYCLES_HDRAW)
        }
    }

    fn handle_vblank_hdraw_end(&mut self) -> FutureGpuEvent {
        self.dispstat.hblank_flag = true;
        if self.dispstat.hblank_irq_enable {
            interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_HBlank);
        };
        (GpuEvent::VBlankHBlank, CYCLES_HBLANK)
    }

    fn handle_vblank_hblank_end(&mut self) -> FutureGpuEvent {
        if self.vcount < DISPLAY_HEIGHT + VBLANK_LINES - 1 {
            self.update_vcount(self.vcount + 1);
            self.dispstat.hblank_flag = false;
            (GpuEvent::VBlankHDraw, CYCLES_HDRAW)
        } else {
            self.update_vcount(0);
            self.dispstat.vblank_flag = false;
            self.dispstat.hblank_flag = false;
            self.render_scanline();
            (GpuEvent::HDraw, CYCLES_HDRAW)
        }
    }

    pub fn on_event<D>(&mut self, event: GpuEvent, dma_notifier: &mut D) -> FutureEvent
    where
        D: DmaNotifer,
    {
        let (event, when) = match event {
            GpuEvent::HDraw => self.handle_hdraw_end(dma_notifier),
            GpuEvent::HBlank => self.handle_hblank_end(dma_notifier),
            GpuEvent::VBlankHDraw => self.handle_vblank_hdraw_end(),
            GpuEvent::VBlankHBlank => self.handle_vblank_hblank_end(),
        };
        (EventType::Gpu(event), when)
    }
}

impl BusIO for Gpu {
    fn read_8(&mut self, addr: Addr) -> u8 {
        let page = (addr >> 24) as usize;
        match page {
            PAGE_PALRAM => self.palette_ram.read_8(addr & 0x3ff),
            PAGE_VRAM => {
                // complicated
                let mut ofs = addr & ((VIDEO_RAM_SIZE as u32) - 1);
                if ofs > 0x18000 {
                    ofs -= 0x8000;
                }
                self.vram.read_8(ofs)
            }
            PAGE_OAM => self.oam.read_8(addr & 0x3ff),
            _ => unreachable!(),
        }
    }

    fn write_16(&mut self, addr: Addr, value: u16) {
        let page = (addr >> 24) as usize;
        match page {
            PAGE_PALRAM => self.palette_ram.write_16(addr & 0x3fe, value),
            PAGE_VRAM => {
                let mut ofs = addr & ((VIDEO_RAM_SIZE as u32) - 1);
                if ofs > 0x18000 {
                    ofs -= 0x8000;
                }
                self.vram.write_16(ofs, value)
            }
            PAGE_OAM => self.oam.write_16(addr & 0x3fe, value),
            _ => unreachable!(),
        }
    }

    fn write_8(&mut self, addr: Addr, value: u8) {
        fn expand_value(value: u8) -> u16 {
            (value as u16) * 0x101
        }

        let page = (addr >> 24) as usize;
        match page {
            PAGE_PALRAM => self.palette_ram.write_16(addr & 0x3fe, expand_value(value)),
            PAGE_VRAM => {
                let mut ofs = addr & ((VIDEO_RAM_SIZE as u32) - 1);
                if ofs > 0x18000 {
                    ofs -= 0x8000;
                }
                if ofs < self.vram_obj_tiles_start {
                    self.vram.write_16(ofs & !1, expand_value(value));
                }
            }
            PAGE_OAM => { /* OAM can't be written with 8bit store */ }
            _ => unreachable!(),
        };
    }
}

mod memory {
    pub type Addr = u32;

    pub trait BusIO {
        fn read_32(&mut self, addr: Addr) -> u32 {
            self.read_16(addr) as u32 | (self.read_16(addr + 2) as u32) << 16
        }

        fn read_16(&mut self, addr: Addr) -> u16 {
            self.default_read_16(addr)
        }

        #[inline(always)]
        fn default_read_16(&mut self, addr: Addr) -> u16 {
            self.read_8(addr) as u16 | (self.read_8(addr + 1) as u16) << 8
        }

        fn read_8(&mut self, addr: Addr) -> u8;

        fn write_32(&mut self, addr: Addr, value: u32) {
            self.write_16(addr, (value & 0xffff) as u16);
            self.write_16(addr + 2, (value >> 16) as u16);
        }

        fn write_16(&mut self, addr: Addr, value: u16) {
            self.default_write_16(addr, value)
        }

        #[inline(always)]
        fn default_write_16(&mut self, addr: Addr, value: u16) {
            self.write_8(addr, (value & 0xff) as u8);
            self.write_8(addr + 1, ((value >> 8) & 0xff) as u8);
        }

        fn write_8(&mut self, addr: Addr, value: u8);

        fn get_bytes(&mut self, range: std::ops::Range<u32>) -> Vec<u8> {
            let mut bytes = Vec::new();
            for b in range {
                bytes.push(self.read_8(b));
            }
            bytes
        }
    }

    impl BusIO for Box<[u8]> {
        #[inline]
        fn read_8(&mut self, addr: Addr) -> u8 {
            self[addr as usize]
        }

        #[inline]
        fn write_8(&mut self, addr: Addr, value: u8) {
            self[addr as usize] = value
        }
    }
}
