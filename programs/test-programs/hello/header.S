/*
 * VEXos user program header (system_0 checks it at 0x03800000; entry is
 * always 0x03800020): magic 'XVX5', type, owner, options, then flag words.
 */
    .section .boot, "ax"
    .arm
    .global _start
    .word 0x35585658        /* V5_SIG_MAGIC */
    .word 0                 /* type: user */
    .word 2                 /* owner: partner */
    .word 0                 /* options */
    .word 0, 0, 0, 0        /* +0x10..+0x1F */
_start:                     /* 0x03800020, called by system_0 */
    b user_main
