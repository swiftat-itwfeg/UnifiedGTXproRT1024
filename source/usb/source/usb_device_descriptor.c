/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016, 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_descriptor.h"
#include "deviceProperties.h"   
#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
DEVICEMFG_t deviceMfg = UNKNOWN_MFG;  
   
                        /* hobart */
usb_device_endpoint_struct_t    g_UsbDevicePrinterEndpoints[USB_PRINTER_ENDPOINT_COUNT];
usb_device_endpoint_struct_t    g_UsbDeviceWeigherEndpoints[USB_WEIGHER_ENDPOINT_COUNT];
usb_device_interface_struct_t   g_UsbDevicePrinterInterface[1];
usb_device_interface_struct_t   g_UsbDeviceWeigherInterface[1];
usb_device_interfaces_struct_t  g_UsbDevicePrinterInterfaces[USB_PRINTER_INTERFACE_COUNT];
usb_device_interfaces_struct_t  g_UsbDeviceWeigherInterfaces[USB_WEIGHER_INTERFACE_COUNT];
usb_device_interface_list_t     g_UsbDevicePrinterInterfaceList[USB_DEVICE_CONFIGURATION_COUNT];
usb_device_interface_list_t     g_UsbDeviceWeigherInterfaceList[USB_DEVICE_CONFIGURATION_COUNT];

                        /* avery */
usb_device_endpoint_struct_t g_cdcVcomCicEndpoints[USB_DEVICE_CONFIG_CDC_ACM][USB_CDC_VCOM_CIC_ENDPOINT_COUNT];
usb_device_endpoint_struct_t g_cdcVcomDicEndpoints[USB_DEVICE_CONFIG_CDC_ACM][USB_CDC_VCOM_DIC_ENDPOINT_COUNT];

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t                         g_UsbDeviceDescriptor[USB_DEVICE_DESCRIPTOR_SIZE];
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t                         g_UsbDeviceConfigurationDescriptor[USB_CFG_DESCRIPTOR_SIZE];
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString0[USB_STRING_0_HOBART_SIZE];
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString1[USB_STRING_1_HOBART_SIZE];
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString2[USB_STRING_2_HOBART_SIZE];

usb_device_class_struct_t g_UsbDevicePrinterConfig = {
    g_UsbDevicePrinterInterfaceList,    /* The interface list of the printer */
    kUSB_DeviceClassTypePHobart,         /* The printer class type */
    USB_DEVICE_CONFIGURATION_COUNT,     /* The configuration count */
};

usb_device_class_struct_t g_UsbDeviceWeigherConfig = {
    g_UsbDeviceWeigherInterfaceList,    /* The interface list of the weigher */
    kUSB_DeviceClassTypeWHobart,         /* The weigher class type */
    USB_DEVICE_CONFIGURATION_COUNT,     /* The configuration count */
};

uint32_t g_UsbDeviceStringDescriptorLength[USB_DEVICE_STRING_COUNT] = {
    sizeof(g_UsbDeviceString0),
    sizeof(g_UsbDeviceString1),
    sizeof(g_UsbDeviceString2),
};

uint8_t *g_UsbDeviceStringDescriptorArray[USB_DEVICE_STRING_COUNT] = {
    g_UsbDeviceString0,
    g_UsbDeviceString1,
    g_UsbDeviceString2,
};

usb_language_t g_UsbDeviceLanguage[USB_DEVICE_LANGUAGE_COUNT] = {{
    g_UsbDeviceStringDescriptorArray,
    g_UsbDeviceStringDescriptorLength,
    (uint16_t)0x0409U,
}};

usb_language_list_t g_UsbDeviceLanguageList = {
    g_UsbDeviceString0,
    sizeof(g_UsbDeviceString0),
    g_UsbDeviceLanguage,
    USB_DEVICE_LANGUAGE_COUNT,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

usb_status_t configureUsbDescriptors( DEVICE_PROPERTIES_t *pProp )
{
    usb_status_t  status = kStatus_USB_Error;
   
    /* configure the device descriptor based on device class */
    if( ( pProp->class == HOBART_WEIGHER_PRINTER ) || 
        ( pProp->class == HOBART_WEIGHER )  ||
        ( pProp->class == HOBART_PRINTER ) ) {
          
        deviceMfg = HOBART_MFG;  
        /*clear endpoint descriptor */
        memset( &g_UsbDevicePrinterEndpoints[0], 0, sizeof( g_UsbDevicePrinterEndpoints[USB_PRINTER_ENDPOINT_COUNT] ) );
        memset( &g_UsbDeviceWeigherEndpoints[0], 0, sizeof( g_UsbDeviceWeigherEndpoints[USB_WEIGHER_ENDPOINT_COUNT] ) );
        memset( &g_UsbDevicePrinterInterface[0], 0, sizeof( g_UsbDevicePrinterInterface[1] ) ); 
        memset( &g_UsbDevicePrinterInterface[0], 0, sizeof( g_UsbDeviceWeigherInterface[1] ) ); 
        memset( &g_UsbDeviceDescriptor[0], 0, USB_DEVICE_DESCRIPTOR_SIZE );
        
        /* printer INTERRUPT OUT pipe */ 
        g_UsbDevicePrinterEndpoints[0].endpointAddress = USB_PRINTER_INTERRUPT_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDevicePrinterEndpoints[0].interval = USB_ENDPOINT_INTERRUPT;       
        g_UsbDevicePrinterEndpoints[0].maxPacketSize = HS_PRINTER_BULK_OUT_PACKET_SIZE;
        g_UsbDevicePrinterEndpoints[0].transferType = 0U;

        /* printer INTERRUPT IN pipe */
        g_UsbDevicePrinterEndpoints[1].endpointAddress = USB_PRINTER_INTERRUPT_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDevicePrinterEndpoints[1].interval = USB_ENDPOINT_INTERRUPT;
        g_UsbDevicePrinterEndpoints[1].maxPacketSize = HS_PRINTER_BULK_IN_PACKET_SIZE;
        g_UsbDevicePrinterEndpoints[1].transferType = 0U;            

        /* printer BULK OUT pipe */
        g_UsbDevicePrinterEndpoints[2].endpointAddress = USB_PRINTER_BULK_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDevicePrinterEndpoints[2].interval = USB_ENDPOINT_BULK;
        g_UsbDevicePrinterEndpoints[2].maxPacketSize = HS_PRINTER_BULK_OUT_PACKET_SIZE;
        g_UsbDevicePrinterEndpoints[2].transferType = 0U;           

        /* printer BULK IN pipe */
        g_UsbDevicePrinterEndpoints[3].endpointAddress = USB_PRINTER_BULK_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDevicePrinterEndpoints[3].interval = USB_ENDPOINT_BULK;
        g_UsbDevicePrinterEndpoints[3].maxPacketSize = HS_PRINTER_BULK_IN_PACKET_SIZE;
        g_UsbDevicePrinterEndpoints[3].transferType = 0U;         

        /* weigher INTERRUPT OUT pipe */
        g_UsbDevicePrinterEndpoints[4].endpointAddress = USB_WEIGHER_INTERRUPT_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDevicePrinterEndpoints[4].interval = USB_ENDPOINT_INTERRUPT;
        g_UsbDevicePrinterEndpoints[4].maxPacketSize = HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE;
        g_UsbDevicePrinterEndpoints[4].transferType = 0U;           

        /* weigher INTERRUPT IN pipe */
        g_UsbDevicePrinterEndpoints[5].endpointAddress = USB_WEIGHER_INTERRUPT_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDevicePrinterEndpoints[5].interval = USB_ENDPOINT_INTERRUPT;
        g_UsbDevicePrinterEndpoints[5].maxPacketSize = HS_WEIGHER_INTERRUPT_IN_PACKET_SIZE;
        g_UsbDevicePrinterEndpoints[5].transferType = 0U;

        /* weigher INTERRUPT OUT pipe */
        g_UsbDeviceWeigherEndpoints[0].endpointAddress = USB_WEIGHER_INTERRUPT_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDeviceWeigherEndpoints[0].interval = USB_ENDPOINT_INTERRUPT;
        g_UsbDeviceWeigherEndpoints[0].maxPacketSize = HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE;
        g_UsbDeviceWeigherEndpoints[0].transferType = 0U;           

        /* weigher INTERRUPT IN pipe */
        g_UsbDeviceWeigherEndpoints[1].endpointAddress = USB_WEIGHER_INTERRUPT_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDeviceWeigherEndpoints[1].interval = USB_ENDPOINT_INTERRUPT;
        g_UsbDeviceWeigherEndpoints[1].maxPacketSize = HS_WEIGHER_INTERRUPT_IN_PACKET_SIZE;
        g_UsbDeviceWeigherEndpoints[1].transferType = 0U;
        
        /* printer interfaces */
        g_UsbDevicePrinterInterface[0].alternateSetting = USB_PRINTER_INTERFACE_ALTERNATE_0;
        g_UsbDevicePrinterInterface[0].classSpecific = NULL;
        g_UsbDevicePrinterInterface[0].endpointList.count = USB_PRINTER_ENDPOINT_COUNT;
        g_UsbDevicePrinterInterface[0].endpointList.endpoint = &g_UsbDevicePrinterEndpoints[0];
        
        /* weigher interfaces */
        g_UsbDeviceWeigherInterface[0].alternateSetting = USB_WEIGHER_INTERFACE_ALTERNATE_0;
        g_UsbDeviceWeigherInterface[0].classSpecific = NULL;
        g_UsbDeviceWeigherInterface[0].endpointList.count = USB_WEIGHER_ENDPOINT_COUNT;
        g_UsbDeviceWeigherInterface[0].endpointList.endpoint = &g_UsbDeviceWeigherEndpoints[0];

        /* printer and weigher interfaces */
        g_UsbDevicePrinterInterfaces[0].classCode = USB_PRINTER_CLASS;
        g_UsbDevicePrinterInterfaces[0].subclassCode = USB_PRINTER_SUBCLASS;
        g_UsbDevicePrinterInterfaces[0].protocolCode = USB_PRINTER_PROTOCOL;
        g_UsbDevicePrinterInterfaces[0].interfaceNumber = USB_PRINTER_INTERFACE_INDEX;
        g_UsbDevicePrinterInterfaces[0].interface = &g_UsbDevicePrinterInterface[0];
        g_UsbDevicePrinterInterfaces[0].count = sizeof(g_UsbDevicePrinterInterface) / sizeof(usb_device_interface_struct_t);   

        g_UsbDeviceWeigherInterfaces[0].classCode = USB_WEIGHER_CLASS;
        g_UsbDeviceWeigherInterfaces[0].subclassCode = USB_WEIGHER_SUBCLASS;
        g_UsbDeviceWeigherInterfaces[0].protocolCode = USB_WEIGHER_PROTOCOL;
        g_UsbDeviceWeigherInterfaces[0].interfaceNumber = USB_WEIGHER_INTERFACE_INDEX;
        g_UsbDeviceWeigherInterfaces[0].interface = &g_UsbDeviceWeigherInterface[0];
        g_UsbDeviceWeigherInterfaces[0].count = sizeof(g_UsbDeviceWeigherInterface) / sizeof(usb_device_interface_struct_t);
  
        /* interface lists */  
        g_UsbDevicePrinterInterfaceList[0].count = USB_PRINTER_INTERFACE_COUNT; 
        g_UsbDevicePrinterInterfaceList[0].interfaces = &g_UsbDevicePrinterInterfaces[0];
 
        g_UsbDeviceWeigherInterfaceList[0].count = USB_WEIGHER_INTERFACE_COUNT;
        g_UsbDeviceWeigherInterfaceList[0].interfaces = &g_UsbDeviceWeigherInterfaces[0];  

        /* device descriptor */
        g_UsbDeviceDescriptor[0] = USB_DESCRIPTOR_LENGTH_DEVICE;
        g_UsbDeviceDescriptor[1] = USB_DESCRIPTOR_TYPE_DEVICE;
        g_UsbDeviceDescriptor[2] = USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION);
        g_UsbDeviceDescriptor[3] = USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION);
        g_UsbDeviceDescriptor[4] = USB_DEVICE_CLASS;
        g_UsbDeviceDescriptor[5] = USB_DEVICE_SUBCLASS;
        g_UsbDeviceDescriptor[6] = USB_DEVICE_PROTOCOL;
        g_UsbDeviceDescriptor[7] = USB_CONTROL_MAX_PACKET_SIZE;
        g_UsbDeviceDescriptor[8] = USB_SHORT_GET_LOW(USB_DEVICE_VID);
        g_UsbDeviceDescriptor[9] = USB_SHORT_GET_HIGH(USB_DEVICE_VID);
        g_UsbDeviceDescriptor[10] = USB_SHORT_GET_LOW(USB_DEVICE_PID);
        g_UsbDeviceDescriptor[11] = USB_SHORT_GET_HIGH(USB_DEVICE_PID);
        g_UsbDeviceDescriptor[12] = USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION);
        g_UsbDeviceDescriptor[13] = USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION);
        g_UsbDeviceDescriptor[14] = 0x01U;
        g_UsbDeviceDescriptor[15] = 0x02U;
        g_UsbDeviceDescriptor[16] = 0x00U;
        g_UsbDeviceDescriptor[17] = USB_DEVICE_CONFIGURATION_COUNT;
         
        /* device configuration descriptor */
        g_UsbDeviceConfigurationDescriptor[0] = USB_DESCRIPTOR_LENGTH_CONFIGURE;
        g_UsbDeviceConfigurationDescriptor[1] = USB_DESCRIPTOR_TYPE_CONFIGURE;
        g_UsbDeviceConfigurationDescriptor[2] = USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
                                                USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT +
                                                USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT +
                                                USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT );
        g_UsbDeviceConfigurationDescriptor[3] = USB_SHORT_GET_HIGH(USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
                                                USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT +
                                                USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT +
                                                USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT );
        g_UsbDeviceConfigurationDescriptor[4] = USB_COMPOSITE_INTERFACE_COUNT;
        g_UsbDeviceConfigurationDescriptor[5] = USB_COMPOSITE_CONFIGURE_INDEX;
        g_UsbDeviceConfigurationDescriptor[6] = 0x00U;
        g_UsbDeviceConfigurationDescriptor[7] = ( USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK ) |
                                                ( USB_DEVICE_CONFIG_SELF_POWER << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT ) |
                                                ( USB_DEVICE_CONFIG_REMOTE_WAKEUP << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT );
        g_UsbDeviceConfigurationDescriptor[8] = USB_DEVICE_MAX_POWER;
        g_UsbDeviceConfigurationDescriptor[9] = USB_DESCRIPTOR_LENGTH_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[10] = USB_DESCRIPTOR_TYPE_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[11] = USB_PRINTER_INTERFACE_INDEX;
        g_UsbDeviceConfigurationDescriptor[12] = USB_PRINTER_INTERFACE_ALTERNATE_0;
        g_UsbDeviceConfigurationDescriptor[13] = USB_PRINTER_ENDPOINT_COUNT;
        g_UsbDeviceConfigurationDescriptor[14] = USB_PRINTER_CLASS;
        g_UsbDeviceConfigurationDescriptor[15] = USB_PRINTER_SUBCLASS;
        g_UsbDeviceConfigurationDescriptor[16] = USB_PRINTER_PROTOCOL;
        g_UsbDeviceConfigurationDescriptor[17] = 0x00U;
        g_UsbDeviceConfigurationDescriptor[18] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[19] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[20] = USB_PRINTER_INTERRUPT_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDeviceConfigurationDescriptor[21] = USB_ENDPOINT_INTERRUPT;
        g_UsbDeviceConfigurationDescriptor[22] = USB_SHORT_GET_LOW(HS_PRINTER_INTERRUPT_IN_PACKET_SIZE), USB_SHORT_GET_HIGH(HS_PRINTER_INTERRUPT_IN_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[23] = HS_PRINTER_INTERRUPT_IN_INTERVAL;
        g_UsbDeviceConfigurationDescriptor[24] = USB_DESCRIPTOR_LENGTH_ENDPOINT
        g_UsbDeviceConfigurationDescriptor[25] = USB_DESCRIPTOR_TYPE_ENDPOINT
        g_UsbDeviceConfigurationDescriptor[26] = USB_PRINTER_INTERRUPT_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDeviceConfigurationDescriptor[27] = USB_ENDPOINT_INTERRUPT;
        g_UsbDeviceConfigurationDescriptor[28] = USB_SHORT_GET_LOW(FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE), USB_SHORT_GET_HIGH(FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[29] = HS_PRINTER_INTERRUPT_OUT_INTERVAL;
        g_UsbDeviceConfigurationDescriptor[30] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[31] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[32] = USB_PRINTER_BULK_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDeviceConfigurationDescriptor[33] = USB_ENDPOINT_BULK;
        g_UsbDeviceConfigurationDescriptor[34] = USB_SHORT_GET_LOW(HS_PRINTER_BULK_OUT_PACKET_SIZE), USB_SHORT_GET_HIGH(HS_PRINTER_BULK_OUT_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[35] = HS_PRINTER_BULK_OUT_INTERVAL;
        g_UsbDeviceConfigurationDescriptor[36] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[37] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[38] = USB_PRINTER_BULK_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        g_UsbDeviceConfigurationDescriptor[39] = USB_ENDPOINT_BULK;
        g_UsbDeviceConfigurationDescriptor[40] = USB_SHORT_GET_LOW(HS_PRINTER_BULK_IN_PACKET_SIZE), USB_SHORT_GET_HIGH(HS_PRINTER_BULK_IN_PACKET_SIZE);         
        g_UsbDeviceConfigurationDescriptor[41] = HS_PRINTER_BULK_IN_INTERVAL;         
        g_UsbDeviceConfigurationDescriptor[42] = USB_DESCRIPTOR_LENGTH_ENDPOINT;         
        g_UsbDeviceConfigurationDescriptor[43] = USB_DESCRIPTOR_TYPE_ENDPOINT;         
        g_UsbDeviceConfigurationDescriptor[44] = USB_WEIGHER_INTERRUPT_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);         
        g_UsbDeviceConfigurationDescriptor[45] = USB_ENDPOINT_INTERRUPT;         
        g_UsbDeviceConfigurationDescriptor[46] = USB_SHORT_GET_LOW(FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE), USB_SHORT_GET_HIGH(FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE);         
        g_UsbDeviceConfigurationDescriptor[47] = HS_WEIGHER_INTERRUPT_IN_INTERVAL;         
        g_UsbDeviceConfigurationDescriptor[48] = USB_DESCRIPTOR_LENGTH_ENDPOINT;         
        g_UsbDeviceConfigurationDescriptor[49] = USB_DESCRIPTOR_TYPE_ENDPOINT;         
        g_UsbDeviceConfigurationDescriptor[50] = USB_WEIGHER_INTERRUPT_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);         
        g_UsbDeviceConfigurationDescriptor[51] = USB_ENDPOINT_INTERRUPT;         
        g_UsbDeviceConfigurationDescriptor[52] = USB_SHORT_GET_LOW(HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE), USB_SHORT_GET_HIGH(HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE);         
        g_UsbDeviceConfigurationDescriptor[53] = HS_WEIGHER_INTERRUPT_OUT_INTERVAL;              
        
        /* string desctriptors */
        g_UsbDeviceString0[0] = 2U + 2U;
        g_UsbDeviceString0[1] = USB_DESCRIPTOR_TYPE_STRING;
        g_UsbDeviceString0[2] = 0x09U;
        g_UsbDeviceString0[3] = 0x04U;
        
        g_UsbDeviceString1[0] = 2U + 2U * 24U;
        g_UsbDeviceString1[1] = USB_DESCRIPTOR_TYPE_STRING;
        g_UsbDeviceString1[2] = 'I';
        g_UsbDeviceString1[3] = 0x00U;
        g_UsbDeviceString1[4] = 'T';
        g_UsbDeviceString1[5] = 0x00U;
        g_UsbDeviceString1[6] = 'W';
        g_UsbDeviceString1[7] = 0x00U;
        g_UsbDeviceString1[8] = ' ';
        g_UsbDeviceString1[9] = 0x00U;
        g_UsbDeviceString1[10] = 'F';
        g_UsbDeviceString1[11] = 0x00U;
        g_UsbDeviceString1[12] = 'O';
        g_UsbDeviceString1[13] = 0x00U;
        g_UsbDeviceString1[14] = 'O';
        g_UsbDeviceString1[15] = 0x00U;
        g_UsbDeviceString1[16] = 'D';
        g_UsbDeviceString1[17] = 0x00U;
        g_UsbDeviceString1[18] = ' ';
        g_UsbDeviceString1[19] = 0x00U;
        g_UsbDeviceString1[20] = 'E';
        g_UsbDeviceString1[21] = 0x00U;
        g_UsbDeviceString1[22] = 'Q';
        g_UsbDeviceString1[23] = 0x00U;
        g_UsbDeviceString1[24] = 'U';        
        g_UsbDeviceString1[25] = 0x00U;
        g_UsbDeviceString1[26] = 'I';
        g_UsbDeviceString1[27] = 0x00U;
        g_UsbDeviceString1[28] = 'P';
        g_UsbDeviceString1[29] = 0x00U;
        g_UsbDeviceString1[30] = 'M';
        g_UsbDeviceString1[31] = 0x00U;
        g_UsbDeviceString1[32] = 'E';
        g_UsbDeviceString1[33] = 'N';
        g_UsbDeviceString1[34] = 0x00U;
        g_UsbDeviceString1[35] = 'T';
        g_UsbDeviceString1[36] = 0x00U;
        g_UsbDeviceString1[37] = ' ';
        g_UsbDeviceString1[38] = 0x00U;
        g_UsbDeviceString1[39] = 'G';
        g_UsbDeviceString1[40] = 0x00U;
        g_UsbDeviceString1[41] = 'R';
        g_UsbDeviceString1[42] = 0x00U;
        g_UsbDeviceString1[43] = 'O';
        g_UsbDeviceString1[44] = 0x00U;
        g_UsbDeviceString1[45] = 'U';
        g_UsbDeviceString1[46] = 0x00U;
        g_UsbDeviceString1[47] = 'P';
        g_UsbDeviceString1[48] = 0x00U;

        
        g_UsbDeviceString2[0] = 2U + 2U * 19U; 
        g_UsbDeviceString2[1] = USB_DESCRIPTOR_TYPE_STRING;
        g_UsbDeviceString2[2] = 'G';
        g_UsbDeviceString2[3] = 0x00U;
        g_UsbDeviceString2[4] = 'L';
        g_UsbDeviceString2[5] = 0x00U;
        g_UsbDeviceString2[6] = 'O';
        g_UsbDeviceString2[7] = 0x00U;
        g_UsbDeviceString2[8] = 'B';
        g_UsbDeviceString2[9] = 0x00U;
        g_UsbDeviceString2[10] = 'A';
        g_UsbDeviceString2[11] = 0x00U;
        g_UsbDeviceString2[12] = 'L';
        g_UsbDeviceString2[13] = 0x00U;
        g_UsbDeviceString2[14] = ' ';
        g_UsbDeviceString2[15] = 0x00U;
        g_UsbDeviceString2[16] = 'S';
        g_UsbDeviceString2[17] = 0x00U;
        g_UsbDeviceString2[18] = 'C';
        g_UsbDeviceString2[19] = 0x00U;
        g_UsbDeviceString2[20] = 'A';
        g_UsbDeviceString2[21] = 0x00U;
        g_UsbDeviceString2[22] = 'L';
        g_UsbDeviceString2[23] = 0x00U;
        g_UsbDeviceString2[24] = 'E';
        g_UsbDeviceString2[25] = 0x00U;
        g_UsbDeviceString2[26] = ' ';
        g_UsbDeviceString2[27] = 0x00U;
        g_UsbDeviceString2[28] = 'D';
        g_UsbDeviceString2[29] = 0x00U;
        g_UsbDeviceString2[30] = 'E';
        g_UsbDeviceString2[31] = 0x00U;
        g_UsbDeviceString2[32] = 'V';
        g_UsbDeviceString2[33] = 0x00U;
        g_UsbDeviceString2[34] = 'I';
        g_UsbDeviceString2[35] = 0x00U;
        g_UsbDeviceString2[36] = 'C';
        g_UsbDeviceString2[37] = 0x00U;
        g_UsbDeviceString2[38] = 'E';
        g_UsbDeviceString2[39] = 0x00U;
        
        status = kStatus_USB_Success;
        
    } else if( ( pProp->class == AVERY_WEIGHER_PRINTER ) || 
               ( pProp->class == AVERY_WEIGHER ) ||
               ( pProp->class == AVERY_PRINTER ) ) {
                 
        deviceMfg = AVERY_MFG;         
        /*clear endpoint descriptor */
        memset( &g_cdcVcomCicEndpoints[0][0], 0, sizeof( g_UsbDevicePrinterEndpoints[USB_PRINTER_ENDPOINT_COUNT] ) );
        memset( &g_cdcVcomDicEndpoints[0][0], 0, sizeof( g_UsbDeviceWeigherEndpoints[USB_WEIGHER_ENDPOINT_COUNT] ) );
        
        g_cdcVcomCicEndpoints[0][0].endpointAddress = USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_1 | (USB_IN << 7U);
        g_cdcVcomCicEndpoints[0][0].interval = USB_ENDPOINT_INTERRUPT;
        g_cdcVcomCicEndpoints[0][0].maxPacketSize = HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
        g_cdcVcomCicEndpoints[0][0].transferType = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL; 
        

        g_cdcVcomCicEndpoints[1][0].endpointAddress = USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_2 | (USB_IN << 7U);
        g_cdcVcomCicEndpoints[1][0].interval = USB_ENDPOINT_INTERRUPT;
        g_cdcVcomCicEndpoints[1][0].maxPacketSize = HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
        g_cdcVcomCicEndpoints[1][0].transferType = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL; 
        
        g_cdcVcomDicEndpoints[0][0].endpointAddress = USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_1 | (USB_IN << 7U);        
        g_cdcVcomDicEndpoints[0][0].interval = USB_ENDPOINT_BULK;
        g_cdcVcomDicEndpoints[0][0].maxPacketSize = HS_CDC_VCOM_BULK_IN_PACKET_SIZE;
        g_cdcVcomDicEndpoints[0][0].transferType = 0U;
          
        g_cdcVcomDicEndpoints[0][1].endpointAddress = USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_1 | (USB_OUT << 7U);        
        g_cdcVcomDicEndpoints[0][1].interval = USB_ENDPOINT_BULK;
        g_cdcVcomDicEndpoints[0][1].maxPacketSize = HS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
        g_cdcVcomDicEndpoints[0][1].transferType = 0U;

        g_cdcVcomDicEndpoints[1][0].endpointAddress = USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_2 | (USB_IN << 7U);        
        g_cdcVcomDicEndpoints[1][0].interval = USB_ENDPOINT_BULK;
        g_cdcVcomDicEndpoints[1][0].maxPacketSize = HS_CDC_VCOM_BULK_IN_PACKET_SIZE;
        g_cdcVcomDicEndpoints[1][0].transferType = 0U;
          
        g_cdcVcomDicEndpoints[1][1].endpointAddress = USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_2 | (USB_OUT << 7U);        
        g_cdcVcomDicEndpoints[1][1].interval = USB_ENDPOINT_BULK;
        g_cdcVcomDicEndpoints[1][1].maxPacketSize = HS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
        g_cdcVcomDicEndpoints[1][1].transferType = 0U;
        
        
        /* device descriptor */
        g_UsbDeviceDescriptor[0] = USB_DESCRIPTOR_LENGTH_DEVICE;
        g_UsbDeviceDescriptor[1] = USB_DESCRIPTOR_TYPE_DEVICE;
        g_UsbDeviceDescriptor[2] = USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION);
        g_UsbDeviceDescriptor[3] = USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION);
        g_UsbDeviceDescriptor[4] = USB_DEVICE_CLASS;
        g_UsbDeviceDescriptor[5] = USB_DEVICE_SUBCLASS;
        g_UsbDeviceDescriptor[6] = USB_DEVICE_PROTOCOL;
        g_UsbDeviceDescriptor[7] = USB_CONTROL_MAX_PACKET_SIZE;
        g_UsbDeviceDescriptor[8] = USB_SHORT_GET_LOW(USB_DEVICE_AV_VID);
        g_UsbDeviceDescriptor[9] = USB_SHORT_GET_HIGH(USB_DEVICE_AV_VID);
        g_UsbDeviceDescriptor[10] = USB_SHORT_GET_LOW(USB_DEVICE_AV_PID);
        g_UsbDeviceDescriptor[11] = USB_SHORT_GET_HIGH(USB_DEVICE_AV_PID);
        g_UsbDeviceDescriptor[12] = USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION);
        g_UsbDeviceDescriptor[13] = USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION);
        g_UsbDeviceDescriptor[14] = 0x01U;
        g_UsbDeviceDescriptor[15] = 0x02U;
        g_UsbDeviceDescriptor[16] = 0x03U;
        g_UsbDeviceDescriptor[17] = USB_DEVICE_CONFIGURATION_COUNT;
         
        /* device configuration descriptor */
        g_UsbDeviceConfigurationDescriptor[0] = USB_DESCRIPTOR_LENGTH_CONFIGURE;
        g_UsbDeviceConfigurationDescriptor[1] = USB_DESCRIPTOR_TYPE_CONFIGURE;
        g_UsbDeviceConfigurationDescriptor[2] = USB_SHORT_GET_LOW ( USB_DESCRIPTOR_LENGTH_CONFIGURE + 
                                                                  (  USB_IAD_DESC_SIZE +
                                                                     USB_DESCRIPTOR_LENGTH_INTERFACE +
                                                                     USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC +
                                                                     USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG +
                                                                     USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT +
                                                                     USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC +
                                                                     USB_DESCRIPTOR_LENGTH_ENDPOINT +
                                                                     USB_DESCRIPTOR_LENGTH_INTERFACE +
                                                                     USB_DESCRIPTOR_LENGTH_ENDPOINT +
                                                                     USB_DESCRIPTOR_LENGTH_ENDPOINT ) *
                                                                     USB_DEVICE_CONFIG_CDC_ACM );

        g_UsbDeviceConfigurationDescriptor[3] = USB_SHORT_GET_HIGH( USB_DESCRIPTOR_LENGTH_CONFIGURE +
                                                                  ( USB_IAD_DESC_SIZE +
                                                                    USB_DESCRIPTOR_LENGTH_INTERFACE +
                                                                    USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC +
                                                                    USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG +   
                                                                    USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT +   
                                                                    USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC +   
                                                                    USB_DESCRIPTOR_LENGTH_ENDPOINT +    
                                                                    USB_DESCRIPTOR_LENGTH_INTERFACE +   
                                                                    USB_DESCRIPTOR_LENGTH_ENDPOINT +
                                                                    USB_DESCRIPTOR_LENGTH_ENDPOINT ) *  
                                                                    USB_DEVICE_CONFIG_CDC_ACM ); 
                                                                   
        g_UsbDeviceConfigurationDescriptor[4] = USB_AV_INTERFACE_COUNT;
        g_UsbDeviceConfigurationDescriptor[5] = USB_COMPOSITE_CONFIGURE_INDEX;
        g_UsbDeviceConfigurationDescriptor[6] = 0x00U;
        
        g_UsbDeviceConfigurationDescriptor[7] = ( USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK ) |
                                                ( USB_DEVICE_CONFIG_SELF_POWER << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT ) |
                                                ( USB_DEVICE_CONFIG_REMOTE_WAKEUP << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT );
        g_UsbDeviceConfigurationDescriptor[8] = USB_DEVICE_MAX_POWER;
        g_UsbDeviceConfigurationDescriptor[9] = USB_IAD_DESC_SIZE;
        g_UsbDeviceConfigurationDescriptor[10] = USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION;
        g_UsbDeviceConfigurationDescriptor[11] = 0x00;
        g_UsbDeviceConfigurationDescriptor[12] = 0x02;
        g_UsbDeviceConfigurationDescriptor[13] = USB_CDC_VCOM_CIC_CLASS;
        g_UsbDeviceConfigurationDescriptor[14] = USB_CDC_VCOM_CIC_SUBCLASS;
        g_UsbDeviceConfigurationDescriptor[15] = 0x00;
        g_UsbDeviceConfigurationDescriptor[16] = 0x02;
        g_UsbDeviceConfigurationDescriptor[17] = USB_DESCRIPTOR_LENGTH_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[18] = USB_DESCRIPTOR_TYPE_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[19] = USB_CDC_VCOM_CIC_INTERFACE_INDEX_1;
        g_UsbDeviceConfigurationDescriptor[20] = USB_CDC_VCOM_CIC_INTERFACE_ALTERNATE_0;
        g_UsbDeviceConfigurationDescriptor[21] = USB_CDC_VCOM_CIC_ENDPOINT_COUNT;
        g_UsbDeviceConfigurationDescriptor[22] = USB_CDC_VCOM_CIC_CLASS;
        g_UsbDeviceConfigurationDescriptor[23] = USB_CDC_VCOM_CIC_SUBCLASS;
        g_UsbDeviceConfigurationDescriptor[24] = USB_CDC_VCOM_CIC_PROTOCOL;
        g_UsbDeviceConfigurationDescriptor[25] = 0x00;
        g_UsbDeviceConfigurationDescriptor[26] = USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC;
        g_UsbDeviceConfigurationDescriptor[27] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[28] = USB_CDC_HEADER_FUNC_DESC;
        g_UsbDeviceConfigurationDescriptor[29] = 0x10;
        g_UsbDeviceConfigurationDescriptor[30] = 0x01;
        g_UsbDeviceConfigurationDescriptor[31] = USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG;
        g_UsbDeviceConfigurationDescriptor[32] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[33] = USB_CDC_CALL_MANAGEMENT_FUNC_DESC;
        g_UsbDeviceConfigurationDescriptor[34] = 0x01;
        g_UsbDeviceConfigurationDescriptor[35] = 0x01;
        g_UsbDeviceConfigurationDescriptor[36] = USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT;
        g_UsbDeviceConfigurationDescriptor[37] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[38] = USB_CDC_ABSTRACT_CONTROL_FUNC_DESC;
        g_UsbDeviceConfigurationDescriptor[39] = 0x02;
        g_UsbDeviceConfigurationDescriptor[40] = USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC;         
        g_UsbDeviceConfigurationDescriptor[41] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;         
        g_UsbDeviceConfigurationDescriptor[42] = USB_CDC_UNION_FUNC_DESC;         
        g_UsbDeviceConfigurationDescriptor[43] = USB_CDC_VCOM_CIC_INTERFACE_INDEX_1;         
        g_UsbDeviceConfigurationDescriptor[44] = USB_CDC_VCOM_DIC_INTERFACE_INDEX_1;         
        g_UsbDeviceConfigurationDescriptor[45] = USB_DESCRIPTOR_LENGTH_ENDPOINT;         
        g_UsbDeviceConfigurationDescriptor[46] = USB_DESCRIPTOR_TYPE_ENDPOINT;         
        g_UsbDeviceConfigurationDescriptor[47] = USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_1 | (USB_IN << 7U);         
        g_UsbDeviceConfigurationDescriptor[48] = USB_ENDPOINT_INTERRUPT;         
        g_UsbDeviceConfigurationDescriptor[49] = USB_SHORT_GET_LOW(HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE);         
        g_UsbDeviceConfigurationDescriptor[50] = USB_SHORT_GET_HIGH(HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE);         
        g_UsbDeviceConfigurationDescriptor[51] = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL;         
        g_UsbDeviceConfigurationDescriptor[52] = USB_DESCRIPTOR_LENGTH_INTERFACE;         
        g_UsbDeviceConfigurationDescriptor[53] = USB_DESCRIPTOR_TYPE_INTERFACE;              
        g_UsbDeviceConfigurationDescriptor[54] = USB_CDC_VCOM_DIC_INTERFACE_INDEX_1;
        g_UsbDeviceConfigurationDescriptor[55] = USB_CDC_VCOM_DIC_INTERFACE_ALTERNATE_0;
        g_UsbDeviceConfigurationDescriptor[56] = USB_CDC_VCOM_DIC_ENDPOINT_COUNT;
        g_UsbDeviceConfigurationDescriptor[57] = USB_CDC_VCOM_DIC_CLASS;
        g_UsbDeviceConfigurationDescriptor[58] = USB_CDC_VCOM_DIC_SUBCLASS;
        g_UsbDeviceConfigurationDescriptor[59] = USB_CDC_VCOM_DIC_PROTOCOL;
        g_UsbDeviceConfigurationDescriptor[60] = 0x00;
        g_UsbDeviceConfigurationDescriptor[61] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[62] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[63] = USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_1 | (USB_IN << 7U);
        g_UsbDeviceConfigurationDescriptor[64] = USB_ENDPOINT_BULK;
        g_UsbDeviceConfigurationDescriptor[65] = USB_SHORT_GET_LOW(HS_CDC_VCOM_BULK_IN_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[66] = USB_SHORT_GET_HIGH(HS_CDC_VCOM_BULK_IN_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[67] = 0x00;
        g_UsbDeviceConfigurationDescriptor[68] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[69] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[70] = USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_1 | (USB_OUT << 7U);
        g_UsbDeviceConfigurationDescriptor[71] = USB_ENDPOINT_BULK;
        g_UsbDeviceConfigurationDescriptor[72] = USB_SHORT_GET_LOW(HS_CDC_VCOM_BULK_OUT_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[73] = USB_SHORT_GET_HIGH(HS_CDC_VCOM_BULK_OUT_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[74] = 0x00;
        g_UsbDeviceConfigurationDescriptor[75] = USB_IAD_DESC_SIZE;
        g_UsbDeviceConfigurationDescriptor[76] = USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION;
        g_UsbDeviceConfigurationDescriptor[77] = 0x02;
        g_UsbDeviceConfigurationDescriptor[78] = 0x02;
        g_UsbDeviceConfigurationDescriptor[79] = USB_CDC_VCOM_CIC_CLASS;
        g_UsbDeviceConfigurationDescriptor[80] = USB_CDC_VCOM_CIC_SUBCLASS;
        g_UsbDeviceConfigurationDescriptor[81] = 0x00;
        g_UsbDeviceConfigurationDescriptor[82] = 0x02;
        g_UsbDeviceConfigurationDescriptor[83] = USB_DESCRIPTOR_LENGTH_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[84] = USB_DESCRIPTOR_TYPE_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[85] = USB_CDC_VCOM_CIC_INTERFACE_INDEX_2;
        g_UsbDeviceConfigurationDescriptor[86] = USB_CDC_VCOM_CIC_INTERFACE_ALTERNATE_0;
        g_UsbDeviceConfigurationDescriptor[87] = USB_CDC_VCOM_CIC_ENDPOINT_COUNT;
        g_UsbDeviceConfigurationDescriptor[88] = USB_CDC_VCOM_CIC_CLASS;
        g_UsbDeviceConfigurationDescriptor[89] = USB_CDC_VCOM_CIC_SUBCLASS;
        g_UsbDeviceConfigurationDescriptor[90] = USB_CDC_VCOM_CIC_PROTOCOL;
        g_UsbDeviceConfigurationDescriptor[91] = 0x00;
        g_UsbDeviceConfigurationDescriptor[92] = USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC;
        g_UsbDeviceConfigurationDescriptor[93] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[94] = USB_CDC_HEADER_FUNC_DESC;
        g_UsbDeviceConfigurationDescriptor[95] = 0x10;
        g_UsbDeviceConfigurationDescriptor[96] = 0x01;
        g_UsbDeviceConfigurationDescriptor[97] = USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG;
        g_UsbDeviceConfigurationDescriptor[98] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[99] = USB_CDC_CALL_MANAGEMENT_FUNC_DESC;
        g_UsbDeviceConfigurationDescriptor[100] = 0x01;
        g_UsbDeviceConfigurationDescriptor[101] = 0x01;
        g_UsbDeviceConfigurationDescriptor[102] = USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT;
        g_UsbDeviceConfigurationDescriptor[103] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[104] = USB_CDC_ABSTRACT_CONTROL_FUNC_DESC;
        g_UsbDeviceConfigurationDescriptor[105] = 0x02;
        g_UsbDeviceConfigurationDescriptor[106] = USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC;
        g_UsbDeviceConfigurationDescriptor[107] = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[108] = USB_CDC_UNION_FUNC_DESC;
        g_UsbDeviceConfigurationDescriptor[109] = USB_CDC_VCOM_CIC_INTERFACE_INDEX_2;
        g_UsbDeviceConfigurationDescriptor[110] = USB_CDC_VCOM_DIC_INTERFACE_INDEX_2;
        g_UsbDeviceConfigurationDescriptor[111] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[112] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[113] = USB_CDC_VCOM_CIC_INTERRUPT_IN_ENDPOINT_2 | (USB_IN << 7U);
        g_UsbDeviceConfigurationDescriptor[114] = USB_ENDPOINT_INTERRUPT;
        g_UsbDeviceConfigurationDescriptor[115] = USB_SHORT_GET_LOW(HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[116] = USB_SHORT_GET_HIGH(HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[117] = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
        g_UsbDeviceConfigurationDescriptor[118] = USB_DESCRIPTOR_LENGTH_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[119] = USB_DESCRIPTOR_TYPE_INTERFACE;
        g_UsbDeviceConfigurationDescriptor[120] = USB_CDC_VCOM_DIC_INTERFACE_INDEX_2;
        g_UsbDeviceConfigurationDescriptor[121] = USB_CDC_VCOM_DIC_INTERFACE_ALTERNATE_0;
        g_UsbDeviceConfigurationDescriptor[122] = USB_CDC_VCOM_DIC_ENDPOINT_COUNT;
        g_UsbDeviceConfigurationDescriptor[123] = USB_CDC_VCOM_DIC_CLASS;
        g_UsbDeviceConfigurationDescriptor[124] = USB_CDC_VCOM_DIC_SUBCLASS;
        g_UsbDeviceConfigurationDescriptor[125] = USB_CDC_VCOM_DIC_PROTOCOL;
        g_UsbDeviceConfigurationDescriptor[126] = 0x00;
        g_UsbDeviceConfigurationDescriptor[127] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[128] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[129] = USB_CDC_VCOM_DIC_BULK_IN_ENDPOINT_2 | (USB_IN << 7U);
        g_UsbDeviceConfigurationDescriptor[130] = USB_ENDPOINT_BULK;
        g_UsbDeviceConfigurationDescriptor[131] = USB_SHORT_GET_LOW(FS_CDC_VCOM_BULK_IN_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[132] = USB_SHORT_GET_HIGH(FS_CDC_VCOM_BULK_IN_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[133] = 0x00;
        g_UsbDeviceConfigurationDescriptor[134] = USB_DESCRIPTOR_LENGTH_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[135] = USB_DESCRIPTOR_TYPE_ENDPOINT;
        g_UsbDeviceConfigurationDescriptor[136] = USB_CDC_VCOM_DIC_BULK_OUT_ENDPOINT_2 | (USB_OUT << 7U);
        g_UsbDeviceConfigurationDescriptor[137] = USB_ENDPOINT_BULK;
        g_UsbDeviceConfigurationDescriptor[138] = USB_SHORT_GET_LOW(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[139] = USB_SHORT_GET_HIGH(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE);
        g_UsbDeviceConfigurationDescriptor[140] = 0x00;

        /* string desctriptors */
        g_UsbDeviceString0[0] = 2U + 2U;
        g_UsbDeviceString0[1] = USB_DESCRIPTOR_TYPE_STRING;
        g_UsbDeviceString0[2] = 0x09U;
        g_UsbDeviceString0[3] = 0x04U;
        
        g_UsbDeviceString1[0] = 2U + 2U * 12U;
        g_UsbDeviceString1[1] = USB_DESCRIPTOR_TYPE_STRING;
        g_UsbDeviceString1[1] = 'A';
        g_UsbDeviceString1[2] = 0x00U;
        g_UsbDeviceString1[3] = 'v';
        g_UsbDeviceString1[4] = 0x00U;
        g_UsbDeviceString1[5] = 'e';
        g_UsbDeviceString1[6] = 0x00U;
        g_UsbDeviceString1[7] = 'r';
        g_UsbDeviceString1[8] = 0x00U;
        g_UsbDeviceString1[9] = 'y';
        g_UsbDeviceString1[10] = 0x00U;
        g_UsbDeviceString1[11] = ' ';
        g_UsbDeviceString1[12] = 0x00U;
        g_UsbDeviceString1[13] = 'B';
        g_UsbDeviceString1[14] = 0x00U;
        g_UsbDeviceString1[15] = 'e';
        g_UsbDeviceString1[16] = 0x00U;
        g_UsbDeviceString1[17] = 'r';
        g_UsbDeviceString1[18] = 0x00U;
        g_UsbDeviceString1[19] = 'k';
        g_UsbDeviceString1[20] = 0x00U;
        g_UsbDeviceString1[21] = 'e';
        g_UsbDeviceString1[22] = 0x00U;
        g_UsbDeviceString1[23] = 'l';        
        g_UsbDeviceString1[24] = 0x00U;

 
        
        g_UsbDeviceString2[0] = 2U + 2U * 32U;
        g_UsbDeviceString2[1] = USB_DESCRIPTOR_TYPE_STRING;
        g_UsbDeviceString2[2] = 'P';
        g_UsbDeviceString2[3] = 0x00U;
        g_UsbDeviceString2[4] = 'r';
        g_UsbDeviceString2[5] = 0x00U;
        g_UsbDeviceString2[6] = 'i';
        g_UsbDeviceString2[7] = 0x00U;
        g_UsbDeviceString2[8] = 'n';
        g_UsbDeviceString2[9] = 0x00U;
        g_UsbDeviceString2[10] = 't';
        g_UsbDeviceString2[11] = 0x00U;
        g_UsbDeviceString2[12] = 'e';
        g_UsbDeviceString2[13] = 0x00U;
        g_UsbDeviceString2[14] = 'r';
        g_UsbDeviceString2[15] = 0x00U;
        g_UsbDeviceString2[16] = ' ';
        g_UsbDeviceString2[17] = 0x00U;
        g_UsbDeviceString2[18] = '&';
        g_UsbDeviceString2[19] = 0x00U;
        g_UsbDeviceString2[20] = ' ';
        g_UsbDeviceString2[21] = 0x00U;
        g_UsbDeviceString2[22] = 'L';
        g_UsbDeviceString2[23] = 0x00U;
        g_UsbDeviceString2[24] = 'o';
        g_UsbDeviceString2[25] = 0x00U;
        g_UsbDeviceString2[26] = 'a';
        g_UsbDeviceString2[27] = 0x00U;
        g_UsbDeviceString2[28] = 'd';
        g_UsbDeviceString2[29] = 0x00U;
        g_UsbDeviceString2[30] = 'c';
        g_UsbDeviceString2[31] = 0x00U;
        g_UsbDeviceString2[32] = 'e';
        g_UsbDeviceString2[33] = 0x00U;
        g_UsbDeviceString2[34] = 'l';
        g_UsbDeviceString2[35] = 0x00U;
        g_UsbDeviceString2[36] = 'l';
        g_UsbDeviceString2[37] = 0x00U;
        g_UsbDeviceString2[38] = ' ';
        g_UsbDeviceString2[39] = 0x00U;
        g_UsbDeviceString2[40] = '(';
        g_UsbDeviceString2[41] = 0x00U;
        g_UsbDeviceString2[42] = 'i';
        g_UsbDeviceString2[43] = 0x00U;
        g_UsbDeviceString2[44] = '.';
        g_UsbDeviceString2[45] = 0x00U;
        g_UsbDeviceString2[46] = 'M';
        g_UsbDeviceString2[47] = 0x00U;
          
        g_UsbDeviceString2[40] = 'X';
        g_UsbDeviceString2[41] = 0x00U;
        g_UsbDeviceString2[42] = ' ';
        g_UsbDeviceString2[43] = 0x00U;
        g_UsbDeviceString2[44] = 'R';
        g_UsbDeviceString2[45] = 0x00U;
        g_UsbDeviceString2[46] = 'T';
        g_UsbDeviceString2[47] = 0x00U;

        g_UsbDeviceString2[40] = '1';
        g_UsbDeviceString2[41] = 0x00U;
        g_UsbDeviceString2[42] = '0';
        g_UsbDeviceString2[43] = 0x00U;
        g_UsbDeviceString2[44] = '2';
        g_UsbDeviceString2[45] = 0x00U;
        g_UsbDeviceString2[46] = '4';
        g_UsbDeviceString2[47] = 0x00U;
        g_UsbDeviceString2[48] = ')';
        g_UsbDeviceString2[49] = 0x00U;

    } else {
        PRINTF("device class error: %d\r\n", pProp->class );
    }
}


/* Get device descriptor request */
usb_status_t USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
                                           usb_device_get_device_descriptor_struct_t *deviceDescriptor)
{
    deviceDescriptor->buffer = g_UsbDeviceDescriptor;
    deviceDescriptor->length = USB_DESCRIPTOR_LENGTH_DEVICE;
    return kStatus_USB_Success;
}

/* Get device configuration descriptor request */
usb_status_t USB_DeviceGetConfigurationDescriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configurationDescriptor)
{
    if( USB_COMPOSITE_CONFIGURE_INDEX > configurationDescriptor->configuration )
    {
        configurationDescriptor->buffer = g_UsbDeviceConfigurationDescriptor;
        configurationDescriptor->length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        return kStatus_USB_Success;
    }
    return kStatus_USB_InvalidRequest;
}

/* Get device string descriptor request */
usb_status_t USB_DeviceGetStringDescriptor(usb_device_handle handle,
                                           usb_device_get_string_descriptor_struct_t *stringDescriptor)
{
    uint8_t languageIndex = 0U;
    uint8_t stringIndex   = USB_DEVICE_STRING_COUNT;

    if (stringDescriptor->stringIndex == 0U)
    {
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
        stringDescriptor->length = g_UsbDeviceLanguageList.stringLength;
    }
    else
    {
        for (; languageIndex < USB_DEVICE_LANGUAGE_COUNT; languageIndex++)
        {
            if (stringDescriptor->languageId == g_UsbDeviceLanguageList.languageList[languageIndex].languageId)
            {
                if (stringDescriptor->stringIndex < USB_DEVICE_STRING_COUNT)
                {
                    stringIndex = stringDescriptor->stringIndex;
                }
                break;
            }
        }

        if (USB_DEVICE_STRING_COUNT == stringIndex)
        {
            return kStatus_USB_InvalidRequest;
        }
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageList[languageIndex].string[stringIndex];
        stringDescriptor->length = g_UsbDeviceLanguageList.languageList[languageIndex].length[stringIndex];
    }
    return kStatus_USB_Success;
}

/* Due to the difference of HS and FS descriptors, the device descriptors and configurations need to be updated to match
 * current speed.
 * As the default, the device descriptors and configurations are configured by using FS parameters for both EHCI and
 * KHCI.
 * When the EHCI is enabled, the application needs to call this function to update device by using current speed.
 * The updated information includes endpoint max packet size, endpoint interval, etc. */
usb_status_t USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed)
{
    usb_descriptor_union_t *descriptorHead;
    usb_descriptor_union_t *descriptorTail;

    descriptorHead = (usb_descriptor_union_t *)&g_UsbDeviceConfigurationDescriptor[0];
    descriptorTail = (usb_descriptor_union_t *)( &g_UsbDeviceConfigurationDescriptor[ USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL - 1U ] );

    while( descriptorHead < descriptorTail )
    {
        if( descriptorHead->common.bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT ) {
            if( USB_SPEED_HIGH == speed ) {
                if( ( ( descriptorHead->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                    ( USB_PRINTER_INTERRUPT_ENDPOINT_OUT ==
                     ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK) ) ) {
                    descriptorHead->endpoint.bInterval = HS_PRINTER_INTERRUPT_OUT_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( HS_PRINTER_INTERRUPT_OUT_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress &
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) &&
                           ( USB_PRINTER_INTERRUPT_ENDPOINT_IN ==
                           ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = HS_PRINTER_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( HS_PRINTER_INTERRUPT_IN_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                          ( USB_PRINTER_BULK_ENDPOINT_OUT ==
                          ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = HS_PRINTER_BULK_OUT_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( HS_PRINTER_BULK_OUT_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress &
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
                           ( USB_PRINTER_BULK_ENDPOINT_IN ==
                           ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = HS_PRINTER_BULK_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( HS_PRINTER_BULK_IN_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                           ( USB_WEIGHER_INTERRUPT_ENDPOINT_OUT ==
                           ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = HS_WEIGHER_INTERRUPT_OUT_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress &
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
                           ( USB_WEIGHER_INTERRUPT_ENDPOINT_IN ==
                           ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = HS_WEIGHER_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_WEIGHER_INTERRUPT_IN_PACKET_SIZE,
                                                       descriptorHead->endpoint.wMaxPacketSize);
                } else {
                
                }
            } else {
                if( ( ( descriptorHead->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) &&
                    ( USB_PRINTER_INTERRUPT_ENDPOINT_OUT ==
                    ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = FS_PRINTER_INTERRUPT_OUT_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress &
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) &&
                          ( USB_PRINTER_INTERRUPT_ENDPOINT_IN ==
                          ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = FS_PRINTER_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( FS_PRINTER_INTERRUPT_IN_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                          USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) &&
                          ( USB_PRINTER_BULK_ENDPOINT_OUT ==
                          ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = FS_PRINTER_BULK_OUT_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( FS_PRINTER_BULK_OUT_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress &
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) &&
                           ( USB_PRINTER_BULK_ENDPOINT_IN ==
                           ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = FS_PRINTER_BULK_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_PRINTER_BULK_IN_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) &&
                           ( USB_WEIGHER_INTERRUPT_ENDPOINT_OUT ==
                           ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = FS_WEIGHER_INTERRUPT_OUT_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else if( ( ( descriptorHead->endpoint.bEndpointAddress &
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                           USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) &&
                           ( USB_WEIGHER_INTERRUPT_ENDPOINT_IN ==
                           ( descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK ) ) ) {
                    descriptorHead->endpoint.bInterval = FS_WEIGHER_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS( FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize );
                } else {
                
                }
            }
        }
        descriptorHead = (usb_descriptor_union_t *)( (uint8_t *)descriptorHead + descriptorHead->common.bLength );
    }
    
    if( deviceMfg == HOBART_MFG ) { 
      
        for( int i = 0U; i < USB_PRINTER_ENDPOINT_COUNT; i++ )
        {
            if( USB_SPEED_HIGH == speed ) {
                if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) {
                    g_UsbDevicePrinterEndpoints[i].maxPacketSize = HS_PRINTER_INTERRUPT_OUT_PACKET_SIZE;
                } else if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress &
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) == USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) {
                    g_UsbDevicePrinterEndpoints[i].maxPacketSize = HS_PRINTER_INTERRUPT_IN_PACKET_SIZE;
                } else if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress &
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) == USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) {
                    g_UsbDevicePrinterEndpoints[i].maxPacketSize = HS_PRINTER_BULK_IN_PACKET_SIZE;
                } else if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress &
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) == USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) {
                    g_UsbDevicePrinterEndpoints[i].maxPacketSize = HS_PRINTER_BULK_OUT_PACKET_SIZE;
                } else {
                
                }
            } else {
                if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                      USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) {
                      g_UsbDevicePrinterEndpoints[i].maxPacketSize = FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE;
                } else if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) == 
                      USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) {
                      g_UsbDevicePrinterEndpoints[i].maxPacketSize = FS_PRINTER_INTERRUPT_IN_PACKET_SIZE;
                } else if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                      USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) {
                      g_UsbDevicePrinterEndpoints[i].maxPacketSize = FS_PRINTER_BULK_OUT_PACKET_SIZE;
                } else if( ( g_UsbDevicePrinterEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) == 
                      USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) {
                    g_UsbDevicePrinterEndpoints[i].maxPacketSize = FS_PRINTER_BULK_IN_PACKET_SIZE;
                } else {

                }
            }
        }

        for( int i = 0U; i < USB_WEIGHER_INTERFACE_COUNT; i++ )
        {
            if( USB_SPEED_HIGH == speed ) {
                if( ( g_UsbDeviceWeigherEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) {
                    g_UsbDeviceWeigherEndpoints[i].maxPacketSize = HS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE;
                } else if( ( g_UsbDeviceWeigherEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) == 
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) {
                    g_UsbDeviceWeigherEndpoints[i].maxPacketSize = HS_WEIGHER_INTERRUPT_IN_PACKET_SIZE;
                }        
            } else {
                if( ( g_UsbDeviceWeigherEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) ==
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT ) {
                    g_UsbDeviceWeigherEndpoints[i].maxPacketSize = FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE;
                } else if( ( g_UsbDeviceWeigherEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) == 
                    USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN ) {
                    g_UsbDeviceWeigherEndpoints[i].maxPacketSize = FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE;
                }
            }
        }   
    } else if( deviceMfg == AVERY_MFG ) {
        for( int j = 0; j < USB_DEVICE_CONFIG_CDC_ACM; j++ ) 
        {
            for (int i = 0; i < USB_CDC_VCOM_CIC_ENDPOINT_COUNT; i++)
            {
                if( USB_SPEED_HIGH == speed ) {
                    g_cdcVcomCicEndpoints[j][i].maxPacketSize = HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
                    g_cdcVcomCicEndpoints[j][i].interval      = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
                } else {
                    g_cdcVcomCicEndpoints[j][i].maxPacketSize = FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
                    g_cdcVcomCicEndpoints[j][i].interval      = FS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
                }
            }

            for( int i = 0; i < USB_CDC_VCOM_DIC_ENDPOINT_COUNT; i++ )
            {
                if( USB_SPEED_HIGH == speed ) {
                    if( g_cdcVcomDicEndpoints[j][i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) {
                        g_cdcVcomDicEndpoints[j][i].maxPacketSize = HS_CDC_VCOM_BULK_IN_PACKET_SIZE;
                    } else {
                        g_cdcVcomDicEndpoints[j][i].maxPacketSize = HS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
                    }
                } else {
                    if( g_cdcVcomDicEndpoints[j][i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK ) {
                        g_cdcVcomDicEndpoints[j][i].maxPacketSize = FS_CDC_VCOM_BULK_IN_PACKET_SIZE;
                    } else {
                        g_cdcVcomDicEndpoints[j][i].maxPacketSize = FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
                    }
                }
            }
        }      
    } else {
    
    }
    return kStatus_USB_Success;
}
