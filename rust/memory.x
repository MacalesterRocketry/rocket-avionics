/*
 * RP2350 memory layout. Mirrors the official Pico 2 / Feather RP2350 default
 * since that's what embassy-rp's `rp235xa` feature assumes.
 *
 * RP2350 has:
 *   - 520 KiB of on-chip SRAM (4 banks × 128 KiB striped + 2 × 4 KiB scratch)
 *   - External QSPI flash, board-dependent (Pico 2 = 4 MiB, Feather = 8 MiB)
 *
 * We dedicate the last 1 MiB of flash to FLASH_DATA (datalog scratch — not yet
 * used; see PORTING_PLAN.md "logging" section for the SD-vs-flash decision)
 * and keep the rest for the firmware image. The embassy-rp boot2 sits at the
 * very start of FLASH and is provided by the `binary-info` feature.
 */

MEMORY {
    /* RP2350 boot2 + image start at 0x10000000. Picotool/UF2 expects this. */
    BOOT2    : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH    : ORIGIN = 0x10000100, LENGTH = 7M - 0x100
    /* Optional reserved region for on-flash datalog scratch (1 MiB). */
    FLASH_DATA : ORIGIN = 0x10700000, LENGTH = 1M

    /* Stripe SRAM (banks 0-3) as one contiguous region for general use. */
    RAM       : ORIGIN = 0x20000000, LENGTH = 512K
    /* Core-local scratch banks. We use SCRATCH_X for core1 stack (set in main.rs). */
    SCRATCH_X : ORIGIN = 0x20080000, LENGTH = 4K
    SCRATCH_Y : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    .flash_data (NOLOAD) : ALIGN(4) { *(.flash_data .flash_data.*) } > FLASH_DATA

    /* Pin core1's stack into SCRATCH_X — this keeps it off the striped SRAM
     * banks where core0 lives, eliminating bus contention during the hot AHRS
     * loop. embassy-rp's `multicore::spawn_core1` reads the address+size from
     * a static slice we provide. */
    .core1_stack (NOLOAD) : ALIGN(8) {
        *(.core1_stack)
    } > SCRATCH_X
}
