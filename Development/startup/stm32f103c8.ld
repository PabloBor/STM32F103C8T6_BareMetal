ENTRY(reset_handler)

MEMORY {
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS {
    /* Aseguramos que la tabla de vectores esté alineada a 4 bytes */
    .vector_table : {
        . = ALIGN(4);                /* Alineación a 4 bytes */
        KEEP(*(.vector_table))       /* Tabla de vectores */
    } > FLASH

    .text : {
        . = ALIGN(4);                /* Alineación a 4 bytes */
        *(.text*)                    /* Código */
        *(.rodata*)                  /* Datos de solo lectura */
    } > FLASH

    .data : {
        . = ALIGN(4);                /* Alineación a 4 bytes */
        *(.data*)                    /* Variables inicializadas */
    } > RAM AT> FLASH

    .bss : {
        . = ALIGN(4);                /* Alineación a 4 bytes */
        *(.bss*)                     /* Variables no inicializadas */
    } > RAM

    _stack_top = ORIGIN(RAM) + LENGTH(RAM); /* Dirección del tope de la pila */
}
