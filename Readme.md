# NEC PC-FX core for MiSTer

NEC PC-FX for MiSTer. The successor to the TG16, known for a lot of FMV games.

## Setup

There's not much to do. Make sure PCFX Bios version 1.00 or 1.00 is renamed boot.rom in your PCFX games folder, which is as you may have guess named `PCFX`.

You may want to create and mount one memory card with the menu option in the core. You likely will never need more than one.

## Accuracy

The v810 isn't super well understood, but the CPU passes the tests that exists and is more accurate than other representations of it have been. The chips shared with the tg16 are already pretty accurate. The rainbow chip is the hardest to characterize, because it's poorly studied and not easy to test either with hardware test roms or probes, because it has internally isoteric behavior decoding videos. Otherwise, the system to the best of my knowledge runs the full library accurately. The DMA or King RAM writes may be slightly too fast because the contention behavior is only partially understood, but it's not super far off.