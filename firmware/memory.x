/* nRF52840 memory layout with S140 SoftDevice v7.x
 *
 * Generous RAM allocation for SoftDevice — give it 64K to be safe.
 * Can be reduced later once we know the exact requirement.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00027000, LENGTH = 868K
  RAM   : ORIGIN = 0x20010000, LENGTH = 192K
}
