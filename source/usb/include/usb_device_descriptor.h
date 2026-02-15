/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_DESCRIPTOR_H__
#define __USB_DEVICE_DESCRIPTOR_H__
#include "deviceProperties.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USB_DEVICE_SPECIFIC_BCD_VERSION (0x0200U)
#define USB_DEVICE_DEMO_BCD_VERSION (0x0101U)
   
#define USB_DEVICE_DESCRIPTOR_SIZE      18
#define USB_CFG_DESCRIPTOR_SIZE         141   
#define USB_STRING_0_HOBART_SIZE         4 
#define USB_STRING_1_HOBART_SIZE        49 
#define USB_STRING_2_HOBART_SIZE        50 
   
   
      /* hobart */
#define USB_DEVICE_VID (0x128CU)
#define USB_DEVICE_PID (0x1000U)

      /* avery */
#define USB_DEVICE_AV_VID (0x4296U)   
#define USB_DEVICE_AV_PID (0x7573U)   
#define USB_DEVICE_CONFIG_CDC_ACM (2U)    
#define USB_CDC_VCOM_DIC_ENDPOINT_COUNT (2U)   
#define USB_CDC_VCOM_CIC_ENDPOINT_COUNT (1U)


      /* hobart */    
#define USB_DEVICE_CLASS (0x00U)
#define USB_DEVICE_SUBCLASS (0x00U)
#define USB_DEVICE_PROTOCOL (0x00U)

#define USB_DEVICE_MAX_POWER (0x32U)

#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL (sizeof(g_UsbDeviceConfigurationDescriptor))
#define USB_DESCRIPTOR_LENGTH_STRING0 (sizeof(g_UsbDeviceString0))
#define USB_DESCRIPTOR_LENGTH_STRING1 (sizeof(g_UsbDeviceString1))
#define USB_DESCRIPTOR_LENGTH_STRING2 (sizeof(g_UsbDeviceString2))

#define USB_DEVICE_CONFIGURATION_COUNT (1U)
#define USB_DEVICE_STRING_COUNT (3U)
#define USB_DEVICE_LANGUAGE_COUNT (1U)

#define USB_COMPOSITE_CONFIGURE_INDEX (1U)

/* #define USB_PRINTER_CONFIGURE_INDEX (1U) */
#define USB_PRINTER_INTERFACE_COUNT (1U)

#define USB_PRINTER_INTERFACE_INDEX (0U)
#define USB_PRINTER_INTERFACE_ALTERNATE_COUNT (1U)
#define USB_PRINTER_INTERFACE_ALTERNATE_0 (0U)
#define USB_PRINTER_ENDPOINT_COUNT (6U)   /* was 4 */  
#define USB_PRINTER_INTERRUPT_ENDPOINT_OUT (1U)
#define USB_PRINTER_INTERRUPT_ENDPOINT_IN (2U)
#define USB_PRINTER_BULK_ENDPOINT_OUT (3U)
#define USB_PRINTER_BULK_ENDPOINT_IN (4U)

#define USB_PRINTER_CLASS (0x00U)
#define USB_PRINTER_SUBCLASS (0x00U)
#define USB_PRINTER_PROTOCOL (0x00U)

#define HS_PRINTER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define HS_PRINTER_INTERRUPT_OUT_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_PRINTER_INTERRUPT_OUT_INTERVAL (0x04U)
#define HS_PRINTER_INTERRUPT_IN_PACKET_SIZE (64U)
#define FS_PRINTER_INTERRUPT_IN_PACKET_SIZE (64U)
#define HS_PRINTER_INTERRUPT_IN_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_PRINTER_INTERRUPT_IN_INTERVAL (0x04U)

#define HS_PRINTER_BULK_OUT_PACKET_SIZE (512U)  
#define FS_PRINTER_BULK_OUT_PACKET_SIZE (512U)  
#define HS_PRINTER_BULK_IN_PACKET_SIZE (512U)   
#define FS_PRINTER_BULK_IN_PACKET_SIZE (512U)   
#define HS_PRINTER_BULK_OUT_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_PRINTER_BULK_OUT_INTERVAL (0x04U)
#define HS_PRINTER_BULK_IN_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_PRINTER_BULK_IN_INTERVAL (0x04U)

#define USB_WEIGHER_CLASS (0x00U)
#define USB_WEIGHER_SUBCLASS (0x00U)
#define USB_WEIGHER_PROTOCOL (0x00U)

#define USB_WEIGHER_INTERFACE_COUNT (2U)
#define USB_WEIGHER_INTERFACE_INDEX (1U)
#define USB_WEIGHER_INTERFACE_ALTERNATE_COUNT (1U)
#define USB_WEIGHER_INTERFACE_ALTERNATE_0 (0U)
#define USB_WEIGHER_IN_BUFFER_LENGTH (8U)
#define USB_WEIGHER_ENDPOINT_COUNT (2U)
#define USB_WEIGHER_INTERRUPT_ENDPOINT_OUT (5U)
#define USB_WEIGHER_INTERRUPT_ENDPOINT_IN (6U)

#define HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE (64U)
#define HS_WEIGHER_INTERRUPT_OUT_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_WEIGHER_INTERRUPT_OUT_INTERVAL (0x04U)
#define HS_WEIGHER_INTERRUPT_IN_PACKET_SIZE (64U)
#define FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE (64U)
#define HS_WEIGHER_INTERRUPT_IN_INTERVAL (0x06U) /* 2^(6-1) = 4ms */
#define FS_WEIGHER_INTERRUPT_IN_INTERVAL (0x04U)

#define USB_COMPOSITE_INTERFACE_COUNT (USB_PRINTER_INTERFACE_COUNT + USB_WEIGHER_INTERFACE_COUNT)
   
   
/* avery */
#define HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE         (16U)
#define FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE         (16U)   
#define HS_CDC_VCOM_INTERRUPT_IN_INTERVAL            (0x07)
#define FS_CDC_VCOM_INTERRUPT_IN_INTERVAL            (0x08)
#define USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_1     (3)
#define USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_2     (1)   
#define HS_CDC_VCOM_BULK_IN_PACKET_SIZE              (512)
#define HS_CDC_VCOM_BULK_OUT_PACKET_SIZE             (512)
#define FS_CDC_VCOM_BULK_IN_PACKET_SIZE              (64)
#define FS_CDC_VCOM_BULK_OUT_PACKET_SIZE             (64)   
   
#define USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_1          (4)
#define USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_1         (4)
#define USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_2          (2)
#define USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_2         (2)
#define USB_AV_INTERFACE_COUNT                       (2 * USB_DEVICE_CONFIG_CDC_ACM)

#define USB_IAD_DESC_SIZE                            (8)
#define USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC        (5)
#define USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG         (5)
#define USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT           (4)
#define USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC         (5)

#define USB_CDC_VCOM_CIC_CLASS                       (0x02)     /* Communications and CDC Control */
#define USB_CDC_VCOM_CIC_SUBCLASS                    (0x02)     /* Abstract (modem) */
#define USB_CDC_VCOM_CIC_PROTOCOL                    (0x00)     /* No protocol (Virtual COM) */
#define USB_CDC_VCOM_DIC_CLASS                       (0x0A)     /* CDC data device */
#define USB_CDC_VCOM_DIC_SUBCLASS                    (0x00)
#define USB_CDC_VCOM_DIC_PROTOCOL                    (0x00)
#define USB_CDC_VCOM_INTERFACE_COUNT                 (2)
#define USB_CDC_VCOM_CIC_ENDPOINT_COUNT              (1)
#define USB_CDC_VCOM_CIC_INTERFACE_ALTERNATE_COUNT   (1)
#define USB_CDC_VCOM_CIC_INTERFACE_ALTERNATE_0       (0)
#define USB_CDC_VCOM_DIC_ENDPOINT_COUNT              (2)
#define USB_CDC_VCOM_DIC_INTERFACE_ALTERNATE_COUNT   (1)
#define USB_CDC_VCOM_DIC_INTERFACE_ALTERNATE_0       (0)
#define USB_CDC_VCOM_CIC_INTERFACE_INDEX_1           (0)
#define USB_CDC_VCOM_DIC_INTERFACE_INDEX_1           (1)
#define USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_1     (1)
#define USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_1          (2)
#define USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_1         (2)
#define USB_CDC_VCOM_CIC_INTERFACE_INDEX_2           (2)
#define USB_CDC_VCOM_DIC_INTERFACE_INDEX_2           (3)
#define USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_2     (3)
#define USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_2          (4)
#define USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_2         (4)
#define USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE         (0x24)
#define USB_DESCRIPTOR_TYPE_CDC_CS_ENDPOINT          (0x25)

#define USB_CDC_HEADER_FUNC_DESC                     (0x00)
#define USB_CDC_CALL_MANAGEMENT_FUNC_DESC            (0x01)
#define USB_CDC_ABSTRACT_CONTROL_FUNC_DESC           (0x02)
#define USB_CDC_UNION_FUNC_DESC                      (0x06)
/*******************************************************************************
 * API
 ******************************************************************************/
usb_status_t configureUsbDescriptors( DEVICE_PROPERTIES_t *pProp );
/* Configure the device according to the USB speed. */
extern usb_status_t USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed);

/* Get device descriptor request */
usb_status_t USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
                                           usb_device_get_device_descriptor_struct_t *deviceDescriptor);

/* Get device configuration descriptor request */
usb_status_t USB_DeviceGetConfigurationDescriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configurationDescriptor);

/* Get device string descriptor request */
usb_status_t USB_DeviceGetStringDescriptor(usb_device_handle handle,
                                           usb_device_get_string_descriptor_struct_t *stringDescriptor);

#endif /* __USB_DEVICE_DESCRIPTOR_H__ */
