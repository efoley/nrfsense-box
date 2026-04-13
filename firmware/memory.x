/* nRF52840 memory layout with S140 SoftDevice v7.x
 *
 * SoftDevice reports needing RAM up to 0x20002300 (~9 KB) with our config
 * (1 peripheral connection, ATT MTU 23). We round up to 0x20002400 (9.25 KB)
 * for alignment headroom.
 *
 * If the SoftDevice config changes (more connections, larger MTU, etc.),
 * the required RAM will increase. Check the RTT log on boot — the SD
 * prints the minimum required address if it's insufficient.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00027000, LENGTH = 868K
  RAM   : ORIGIN = 0x20002400, LENGTH = 247K
}
