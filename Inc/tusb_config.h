#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

// MCU configuration
#define CFG_TUSB_MCU            OPT_MCU_STM32H5

// RHPort number used for device can be 0 or 1
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE

// This MCU uses the native USB Full-Speed peripheral.
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

// Device mode configuration
#define CFG_TUD_ENABLED         1

// CDC virtual serial port configuration
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  2048
#define CFG_TUD_CDC_TX_BUFSIZE  4096

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
