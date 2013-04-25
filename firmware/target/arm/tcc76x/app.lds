#include "config.h"

ENTRY(start)
OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)
STARTUP(target/arm/tcc76x/crt0.o)

#define PLUGINSIZE PLUGIN_BUFFER_SIZE
#define CODECSIZE CODEC_SIZE

#define DRAMSIZE (MEMORYSIZE * 0x100000) - PLUGINSIZE - CODECSIZE

#define DRAMORIG 0x20000000
#define IRAMORIG 0x00000000
#define IRAMSIZE 64K


/* End of the audio buffer, where the codec buffer starts */
#define ENDAUDIOADDR  (DRAMORIG + DRAMSIZE)

/* Where the codec buffer ends, and the plugin buffer starts */
#define ENDADDR (ENDAUDIOADDR + CODECSIZE)


MEMORY
{
#ifdef TCCBOOT
   DRAM : ORIGIN = DRAMORIG + DRAMSIZE - 0x100000, LENGTH = 0x100000
#else
   DRAM : ORIGIN = DRAMORIG, LENGTH = DRAMSIZE
#endif
   IRAM : ORIGIN = IRAMORIG, LENGTH = IRAMSIZE
}

SECTIONS
{
    .text : {
        loadaddress = .;
        _loadaddress = .;
        . = ALIGN(0x200);
        *(.init.text)
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
    } > DRAM

    .data : {
        *(.data*)
        *(.rodata.*)
        *(.rodata)
        . = ALIGN(0x4);
        _dataend = . ;
    } > DRAM

    /* IRAM is mapped to 0 and vectors are at start of IRAM */
    .vectors 0x0 :
    {
        _vectorsstart = .;
        KEEP(*(.vectors));
        _vectorsend = .;
    } > IRAM AT> DRAM

    _vectorscopy = LOADADDR(.vectors);
    _noloaddram  = LOADADDR(.vectors);

    .ibss (NOLOAD) :
    {
        _iedata = .;
        *(.qharray)
        *(.ibss)
        . = ALIGN(0x4);
        _iend = .;
    } > IRAM

    .iram _iend :
    {
        _iramstart = .;
        *(.icode)
        *(.irodata)
        *(.idata)
        . = ALIGN(0x4);
        _iramend = .;
    } > IRAM AT> DRAM

    _iramcopy = LOADADDR(.iram);

    .init ENDAUDIOADDR :
    {
        . = ALIGN(4);
        _initstart = .;
        *(.init*)
        _initend = .;
    } AT> DRAM

    _initcopy = LOADADDR(.init);

    .stack (NOLOAD) :
    {
       *(.stack)
       stackbegin = .;
       . += 0x2000;
       stackend = .;
    } > IRAM

    .bss _noloaddram : {
         _edata = .;
         *(.bss*);
         *(COMMON)
         _end = .;
    } > DRAM

    .audiobuf ALIGN(4) :
    {
        _audiobuffer = .;
        audiobuffer = .;
    } > DRAM

    .audiobufend ENDAUDIOADDR:
    {
        audiobufend = .;
        _audiobufend = .;
    } > DRAM

    .codec ENDAUDIOADDR:
    {
        codecbuf = .;
        _codecbuf = .;
    }

    .plugin ENDADDR:
    {
        _pluginbuf = .;
        pluginbuf = .;
    }
}
