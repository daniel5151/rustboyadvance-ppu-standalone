# rustboyadvance-ppu-standalone

A quick-and-dirty extraction and simplification of the PPU from
[RustBoyAdvance-NG](https://github.com/michelhe/rustboyadvance-ng), hoisted into its own
crate.

Tailored to the needs of my other project:
[wide-libretro](https://github.com/daniel5151/wide-libretro).

Check out the `wide-libretro` repo for more details...

## Changes

- code is clippy clean
- minimal `pub` API
- removed all built-in scheduling + interrupt triggers
  - i.e: this code is a "pure" function of RAM+Register -> Framebuffer
- removed serde
  - tho it might make sense to re-add it later...
- externally managed VRAM, OAM, and Pallette RAM
  - GBA PPU doesn't write-back to RAM at all (thankfully)
