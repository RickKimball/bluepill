ENTRY(_start)

PROVIDE ( _estack = 0x20005000 );

MEMORY
{
	flash(wrx) : ORIGIN = 0x08000000, LENGTH = 64K
	sram(rx)   : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
	.isr_vectors :
	{
	KEEP(*(.isr_vectors))
	} > flash

	.text :
	{
	. = ALIGN(4);
	*(.text)
	*(.text*)
	*(.text*)
	} > flash

	.rodata : 
	{
	. = ALIGN(4);
	*(.rodata)
	*(.rodata*)
	. = ALIGN(4);
	} > flash

	_sidata = LOADADDR(.data);

	.data :
	{
	. = ALIGN(4);
	_sdata = .;

	*(.data)
	*(.data*)

	. = ALIGN(4);

	*(.ramfunc)
	*(.ramfunc*)

	. = ALIGN(4);

	_edata = .;
	} > sram AT > flash

	.bss :
        {
	. = ALIGN(4);
	_sbss = .;
	*(.bss)
	*(.bss*)
	*(COMMON)
	. = ALIGN(4);
	_ebss = .;
	_end = .;
	end = .;
	} > sram
}

