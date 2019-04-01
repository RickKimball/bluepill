/*	vim: set ts=4 sw=4 noexpandtab:

	stm32f103c8.ld - gcc ldscript 

*/

ENTRY(Reset_Handler)

PROVIDE ( _estack = 0x20005000 );

MEMORY
{
	FLASH(wrx) : ORIGIN = 0x08000000, LENGTH = 64K
	SRAM(rx)   : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
	.isr_vectors :
	{
		KEEP(*(.isr_vectors))
	} > FLASH

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text*)
		*(.text*)
	} > FLASH

	.rodata : 
	{
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} > FLASH

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
	} > SRAM AT > FLASH

	.bss :
	{
	. = ALIGN(4);
		__bss_start__ = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
		_end = .;
		end = .;
	} > SRAM
}

