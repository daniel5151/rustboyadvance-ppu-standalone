#[macro_use]
extern crate bitfield;
#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate log;

macro_rules! index2d {
    ($x:expr, $y:expr, $w:expr) => {
        $w * $y + $x
    };
    ($t:ty, $x:expr, $y:expr, $w:expr) => {
        (($w as $t) * ($y as $t) + ($x as $t)) as $t
    };
}

mod layer;
mod regs;
mod render;
mod rgb15;
mod sfx;
mod window;

mod consts {
    pub const VRAM_ADDR: u32 = 0x0600_0000;

    pub const VIDEO_RAM_SIZE: usize = 128 * 1024;
    pub const PALETTE_RAM_SIZE: usize = 1024;
    pub const OAM_SIZE: usize = 1024;

    pub const DISPLAY_WIDTH: usize = 240;
    pub const DISPLAY_HEIGHT: usize = 160;
    pub const VBLANK_LINES: usize = 68;

    pub const TILE_SIZE: u32 = 0x20;

    pub const VRAM_OBJ_TILES_START_TEXT: u32 = 0x1_0000;
    pub const VRAM_OBJ_TILES_START_BITMAP: u32 = 0x1_4000;

    pub const REG_DISPCNT: u32 = 0x0400_0000; //  2    R/W    LCD Control
    pub const REG_DISPSTAT: u32 = 0x0400_0004; //  2    R/W    General LCD Status (STAT,LYC)
    pub const REG_BG0CNT: u32 = 0x0400_0008; //  2    R/W    BG0 Control
    pub const REG_BG1CNT: u32 = 0x0400_000A; //  2    R/W    BG1 Control
    pub const REG_BG2CNT: u32 = 0x0400_000C; //  2    R/W    BG2 Control
    pub const REG_BG3CNT: u32 = 0x0400_000E; //  2    R/W    BG3 Control
    pub const REG_BG0HOFS: u32 = 0x0400_0010; //  2    W      BG0 X-Offset
    pub const REG_BG0VOFS: u32 = 0x0400_0012; //  2    W      BG0 Y-Offset
    pub const REG_BG1HOFS: u32 = 0x0400_0014; //  2    W      BG1 X-Offset
    pub const REG_BG1VOFS: u32 = 0x0400_0016; //  2    W      BG1 Y-Offset
    pub const REG_BG2HOFS: u32 = 0x0400_0018; //  2    W      BG2 X-Offset
    pub const REG_BG2VOFS: u32 = 0x0400_001A; //  2    W      BG2 Y-Offset
    pub const REG_BG3HOFS: u32 = 0x0400_001C; //  2    W      BG3 X-Offset
    pub const REG_BG3VOFS: u32 = 0x0400_001E; //  2    W      BG3 Y-Offset
    pub const REG_BG2PA: u32 = 0x0400_0020; //  2    W      BG2 Rotation/Scaling Parameter A (dx)
    pub const REG_BG2PB: u32 = 0x0400_0022; //  2    W      BG2 Rotation/Scaling Parameter B (dmx)
    pub const REG_BG2PC: u32 = 0x0400_0024; //  2    W      BG2 Rotation/Scaling Parameter C (dy)
    pub const REG_BG2PD: u32 = 0x0400_0026; //  2    W      BG2 Rotation/Scaling Parameter D (dmy)
    pub const REG_BG2X_L: u32 = 0x0400_0028; //  4    W      BG2 Reference Point X-Coordinate, lower 16 bit
    pub const REG_BG2X_H: u32 = 0x0400_002A; //  4    W      BG2 Reference Point X-Coordinate, upper 16 bit
    pub const REG_BG2Y_L: u32 = 0x0400_002C; //  4    W      BG2 Reference Point Y-Coordinate, lower 16 bit
    pub const REG_BG2Y_H: u32 = 0x0400_002E; //  4    W      BG2 Reference Point Y-Coordinate, upper 16 bit
    pub const REG_BG3PA: u32 = 0x0400_0030; //  2    W      BG3 Rotation/Scaling Parameter A (dx)
    pub const REG_BG3PB: u32 = 0x0400_0032; //  2    W      BG3 Rotation/Scaling Parameter B (dmx)
    pub const REG_BG3PC: u32 = 0x0400_0034; //  2    W      BG3 Rotation/Scaling Parameter C (dy)
    pub const REG_BG3PD: u32 = 0x0400_0036; //  2    W      BG3 Rotation/Scaling Parameter D (dmy)
    pub const REG_BG3X_L: u32 = 0x0400_0038; //  4    W      BG3 Reference Point X-Coordinate, lower 16 bit
    pub const REG_BG3X_H: u32 = 0x0400_003A; //  4    W      BG3 Reference Point X-Coordinate, upper 16 bit
    pub const REG_BG3Y_L: u32 = 0x0400_003C; //  4    W      BG3 Reference Point Y-Coordinate, lower 16 bit
    pub const REG_BG3Y_H: u32 = 0x0400_003E; //  4    W      BG3 Reference Point Y-Coordinate, upper 16 bit
    pub const REG_WIN0H: u32 = 0x0400_0040; //  2    W      Window 0 Horizontal Dimensions
    pub const REG_WIN1H: u32 = 0x0400_0042; //  2    W      Window 1 Horizontal Dimensions
    pub const REG_WIN0V: u32 = 0x0400_0044; //  2    W      Window 0 Vertical Dimensions
    pub const REG_WIN1V: u32 = 0x0400_0046; //  2    W      Window 1 Vertical Dimensions
    pub const REG_WININ: u32 = 0x0400_0048; //  2    R/W    Inside of Window 0 and 1
    pub const REG_WINOUT: u32 = 0x0400_004A; //  2    R/W    Inside of OBJ Window & Outside of Windows
    pub const REG_MOSAIC: u32 = 0x0400_004C; //  2    W      Mosaic Size
    pub const REG_BLDCNT: u32 = 0x0400_0050; //  2    R/W    Color Special Effects Selection
    pub const REG_BLDALPHA: u32 = 0x0400_0052; //  2    R/W    Alpha Blending Coefficients
    pub const REG_BLDY: u32 = 0x0400_0054; //  2    W      Brightness (Fade-In/Out) Coefficient
}

pub use self::consts::OAM_SIZE;
pub use self::consts::PALETTE_RAM_SIZE;
pub use self::consts::VIDEO_RAM_SIZE;

use self::consts::*;
use self::regs::*;
use self::render::Point;
use self::rgb15::Rgb15;
use self::window::*;
use debug_stub_derive::DebugStub;
use enum_primitive_derive::Primitive;
use memory::BusIO;
use num::FromPrimitive;

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

#[derive(Debug, Default, Copy, Clone)]
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

#[derive(Debug, Copy, Clone)]
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

#[derive(Clone, DebugStub)]
pub struct GbaPpu {
    // registers
    vcount: usize, // VCOUNT
    dispcnt: DisplayControl,
    dispstat: DisplayStatus,
    bgcnt: [BgControl; 4],
    bg_vofs: [u16; 4],
    bg_hofs: [u16; 4],
    bg_aff: [BgAffine; 2],
    win0: Window,
    win1: Window,
    winout_flags: WindowFlags,
    winobj_flags: WindowFlags,
    mosaic: RegMosaic,
    bldcnt: BlendControl,
    bldalpha: BlendAlpha,
    bldy: u16,

    palette_ram: &'static [u8],
    vram: &'static [u8],
    oam: &'static [u8],

    vram_obj_tiles_start: u32,
    obj_buffer: Box<[ObjBufferEntry]>,
    frame_buffer: Box<[u32]>,
    bg_line: [Box<[Rgb15]>; 4],
}

impl GbaPpu {
    pub fn new(palette_ram: &'static [u8], vram: &'static [u8], oam: &'static [u8]) -> GbaPpu {
        fn alloc_scanline_buffer() -> Box<[Rgb15]> {
            vec![Rgb15::TRANSPARENT; DISPLAY_WIDTH].into_boxed_slice()
        }

        GbaPpu {
            vcount: 0,
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

            palette_ram,
            vram,
            oam,

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
    fn write_dispcnt(&mut self, value: u16) {
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

    pub fn register_write(&mut self, addr: u32, value: u16) {
        fn sign_extend_i32(value: i32, size: u32) -> i32 {
            let shift = 32 - size;
            (value << shift) >> shift
        }

        macro_rules! write_reference_point {
            (low bg $coord:ident $internal:ident) => {{
                let i = ((addr - REG_BG2X_L) / 0x10) as usize;
                let t = self.bg_aff[i].$coord as u32;
                self.bg_aff[i].$coord = ((t & 0xffff0000) + (value as u32)) as i32;
                let new_value = ((t & 0xffff0000) + (value as u32)) as i32;
                self.bg_aff[i].$coord = new_value;
                self.bg_aff[i].$internal = new_value;
            }};
            (high bg $coord:ident $internal:ident) => {{
                let i = ((addr - REG_BG2X_L) / 0x10) as usize;
                let t = self.bg_aff[i].$coord;
                let new_value =
                    (t & 0xffff) | ((sign_extend_i32((value & 0xfff) as i32, 12)) << 16);
                self.bg_aff[i].$coord = new_value;
                self.bg_aff[i].$internal = new_value;
            }};
        }

        match addr {
            REG_DISPCNT => self.write_dispcnt(value),
            REG_DISPSTAT => self.dispstat.write(value),
            REG_BG0CNT => self.bgcnt[0].write(value),
            REG_BG1CNT => self.bgcnt[1].write(value),
            REG_BG2CNT => self.bgcnt[2].write(value),
            REG_BG3CNT => self.bgcnt[3].write(value),
            REG_BG0HOFS => self.bg_hofs[0] = value & 0x1ff,
            REG_BG0VOFS => self.bg_vofs[0] = value & 0x1ff,
            REG_BG1HOFS => self.bg_hofs[1] = value & 0x1ff,
            REG_BG1VOFS => self.bg_vofs[1] = value & 0x1ff,
            REG_BG2HOFS => self.bg_hofs[2] = value & 0x1ff,
            REG_BG2VOFS => self.bg_vofs[2] = value & 0x1ff,
            REG_BG3HOFS => self.bg_hofs[3] = value & 0x1ff,
            REG_BG3VOFS => self.bg_vofs[3] = value & 0x1ff,
            REG_BG2X_L | REG_BG3X_L => write_reference_point!(low bg x internal_x),
            REG_BG2Y_L | REG_BG3Y_L => write_reference_point!(low bg y internal_y),
            REG_BG2X_H | REG_BG3X_H => write_reference_point!(high bg x internal_x),
            REG_BG2Y_H | REG_BG3Y_H => write_reference_point!(high bg y internal_y),
            REG_BG2PA => self.bg_aff[0].pa = value as i16,
            REG_BG2PB => self.bg_aff[0].pb = value as i16,
            REG_BG2PC => self.bg_aff[0].pc = value as i16,
            REG_BG2PD => self.bg_aff[0].pd = value as i16,
            REG_BG3PA => self.bg_aff[1].pa = value as i16,
            REG_BG3PB => self.bg_aff[1].pb = value as i16,
            REG_BG3PC => self.bg_aff[1].pc = value as i16,
            REG_BG3PD => self.bg_aff[1].pd = value as i16,
            REG_WIN0H => {
                let right = value & 0xff;
                let left = value >> 8;
                self.win0.right = right as u8;
                self.win0.left = left as u8;
            }
            REG_WIN1H => {
                let right = value & 0xff;
                let left = value >> 8;
                self.win1.right = right as u8;
                self.win1.left = left as u8;
            }
            REG_WIN0V => {
                let bottom = value & 0xff;
                let top = value >> 8;
                self.win0.bottom = bottom as u8;
                self.win0.top = top as u8;
            }
            REG_WIN1V => {
                let bottom = value & 0xff;
                let top = value >> 8;
                self.win1.bottom = bottom as u8;
                self.win1.top = top as u8;
            }
            REG_WININ => {
                let value = value & !0xc0c0;
                self.win0.flags = WindowFlags::from(value & 0xff);
                self.win1.flags = WindowFlags::from(value >> 8);
            }
            REG_WINOUT => {
                let value = value & !0xc0c0;
                self.winout_flags = WindowFlags::from(value & 0xff);
                self.winobj_flags = WindowFlags::from(value >> 8);
            }
            REG_MOSAIC => self.mosaic.0 = value,
            REG_BLDCNT => self.bldcnt.write(value),
            REG_BLDALPHA => self.bldalpha.write(value),
            REG_BLDY => self.bldy = core::cmp::min(value & 0b11111, 16),
            _ => debug!("unexpected register write: {:x?} <-- {:x?}", addr, value),
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
            // interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_VCounterMatch);
        }
    }

    #[inline]
    pub fn handle_hdraw_end(&mut self) {
        self.dispstat.hblank_flag = true;
        if self.dispstat.hblank_irq_enable {
            // interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_HBlank);
        };
    }

    pub fn handle_hblank_end(&mut self) {
        self.update_vcount(self.vcount + 1);

        if self.vcount < DISPLAY_HEIGHT {
            self.dispstat.hblank_flag = false;
            self.render_scanline();
            // update BG2/3 reference points on the end of a scanline
            for i in 0..2 {
                self.bg_aff[i].internal_x += self.bg_aff[i].pb as i32;
                self.bg_aff[i].internal_y += self.bg_aff[i].pd as i32;
            }
        } else {
            // latch BG2/3 reference points on vblank
            for i in 0..2 {
                self.bg_aff[i].internal_x = self.bg_aff[i].x;
                self.bg_aff[i].internal_y = self.bg_aff[i].y;
            }

            self.dispstat.vblank_flag = true;
            self.dispstat.hblank_flag = false;
            if self.dispstat.vblank_irq_enable {
                // interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_VBlank);
            };

            self.obj_buffer_reset();
        }
    }

    pub fn handle_vblank_hdraw_end(&mut self) {
        self.dispstat.hblank_flag = true;
        if self.dispstat.hblank_irq_enable {
            // interrupt::signal_irq(&self.interrupt_flags, Interrupt::LCD_HBlank);
        };
    }

    pub fn handle_vblank_hblank_end(&mut self) {
        if self.vcount < DISPLAY_HEIGHT + VBLANK_LINES - 1 {
            self.update_vcount(self.vcount + 1);
            self.dispstat.hblank_flag = false;
        } else {
            self.update_vcount(0);
            self.dispstat.vblank_flag = false;
            self.dispstat.hblank_flag = false;
            self.render_scanline();
        }
    }
}

mod memory {
    pub trait BusIO {
        fn read_32(&mut self, addr: u32) -> u32 {
            self.read_16(addr) as u32 | (self.read_16(addr + 2) as u32) << 16
        }

        fn read_16(&mut self, addr: u32) -> u16 {
            self.read_8(addr) as u16 | (self.read_8(addr + 1) as u16) << 8
        }

        fn read_8(&mut self, addr: u32) -> u8;
    }

    impl<const N: usize> BusIO for &'static [u8; N] {
        #[inline]
        fn read_8(&mut self, addr: u32) -> u8 {
            self[addr as usize]
        }
    }

    impl BusIO for &'static [u8] {
        #[inline]
        fn read_8(&mut self, addr: u32) -> u8 {
            self[addr as usize]
        }
    }
}
