/* Flat V5 user program at the slot base; the header must come first. */
ENTRY(_start)
SECTIONS
{
    . = 0x03800000;
    .boot : { KEEP(*(.boot)) }
    .text : { *(.text*) }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    /* VEXos loads only the flat .bin and does not zero or map .bss, so keep
       .bss inside the image: make it a loaded (zero-filled) section by giving
       it contents and anchoring a word past its end so objcopy emits the
       zeros rather than truncating a trailing NOBITS section. */
    .bss : { *(.bss*) *(COMMON) . = ALIGN(4); }
    .bss_tail : { LONG(0) }
    /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
