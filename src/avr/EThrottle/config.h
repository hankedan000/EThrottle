#ifndef CONFIG_H_
#define CONFIG_H_

#define PID_SAMPLE_RATE_MS 10// 10ms

// define this if you have the mcp-can-boot bootloader installed
//  * allows for remote reflashing via CAN
//  * watchdog reset support (regular Arduino bootloader doesn't support)
#undef MCP_CAN_BOOT_BL

// toggle for watchdog support
#define WATCHDOG_SUPPORT 0
#ifdef MCP_CAN_BOOT_BL
#define WATCHDOG_SUPPORT 1// auto enable watchdog if using mcp-can-boot bootloader
#endif

#endif