/* Linker script to configure memory regions. */

MEMORY
{
  FLASH (rwx) : ORIGIN = 0x0 + 152K, LENGTH = 512K - 152K - 4K /* 152 kB is taken by s132, 4 kB is taken by bootloader settings */
  RAM (rwx) :  ORIGIN = 0x200098a8, LENGTH = 0x6758 /* worst case taken from example apps. this can be significantly reduced if needed */
  /*bootloader_settings_page (r) : ORIGIN = 0x0 + 512K - 4K, LENGTH = 4K
  uicr_bootloader_start_address (r) : ORIGIN = 0x10001014, LENGTH = 0x4*/
}

