/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
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
#include "hid_mouse.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_tsi_v4.h"
#include "fsl_debug_console.h"
#include "fsl_lptmr.h"

#include <stdio.h>
#include <stdlib.h>
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#if (USB_DEVICE_CONFIG_HID < 2U)
#error USB_DEVICE_CONFIG_HID need to > 1U, Please change the MARCO USB_DEVICE_CONFIG_HID in file "usb_device_config.h".
#endif

#include "pin_mux.h"
/*******************************************************************************/
#include "math.h"
#include "fsl_mma.h"
#include "fsl_tpm.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Available PAD names on board */
#define PAD_TSI_ELECTRODE_1_NAME "E1"

/* TSI indication leds for electrode 1/2 */
#define LED_INIT() LED_RED_INIT(LOGIC_LED_OFF)
#define LED_TOGGLE() LED_RED_TOGGLE()

/* LLWU module wakeup source index for TSI module */
#define LLWU_TSI_IDX 4U

/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT (250000U)
/* Define the delta value to indicate a touch event */
#define TOUCH_DELTA_VALUE 3000U

volatile uint8_t shouldToggleTsiChannel = false;

volatile enum operation typeOfOperation = none;
/*******************************************************************************/
#define BOARD_TIMER_BASEADDR TPM0
#define BOARD_FIRST_TIMER_CHANNEL 5U
#define BOARD_SECOND_TIMER_CHANNEL 2U
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define TIMER_CLOCK_MODE 1U
/* I2C source clock */
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U
#define I2C_RELEASE_BUS_COUNT 100U
/* Upper bound and lower bound angle values */
#define ANGLE_UPPER_BOUND 85U
#define ANGLE_LOWER_BOUND 5U


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event,
		void *param);
static void USB_DeviceApplicationInit(void);

void BOARD_I2C_ReleaseBus(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern usb_device_class_struct_t g_UsbDeviceHidMouseConfig;
extern usb_device_class_struct_t g_UsbDeviceHidKeyboardConfig;

usb_device_composite_struct_t g_UsbDeviceComposite;

/* Set class configurations */
usb_device_class_config_struct_t g_CompositeClassConfig[USB_COMPOSITE_INTERFACE_COUNT] =
		{ { USB_DeviceHidKeyboardCallback, /* HID keyboard class callback pointer */
		(class_handle_t) NULL, /* The HID class handle, This field is set by USB_DeviceClassInit */
		&g_UsbDeviceHidKeyboardConfig, /* The HID keyboard configuration, including class code, subcode, and protocol,
		 class
		 type, transfer type, endpoint address, max packet size, etc.*/
		}, { USB_DeviceHidMouseCallback, /* HID mouse class callback pointer */
		(class_handle_t) NULL, /* The HID class handle, This field is set by USB_DeviceClassInit */
		&g_UsbDeviceHidMouseConfig, /* The HID mouse configuration, including class code, subcode, and protocol, class
		 type,
		 transfer type, endpoint address, max packet size, etc.*/
		} };

/* Set class configuration list */
usb_device_class_config_list_struct_t g_UsbDeviceCompositeConfigList = {
		g_CompositeClassConfig, /* Class configurations */
		USB_DeviceCallback, /* Device callback pointer */
		USB_COMPOSITE_INTERFACE_COUNT, /* Class count */
};

tsi_calibration_data_t buffer;

/* MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};
/*******************************************************************************
 * Code
 ******************************************************************************/
#if (defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U))
void USB0_IRQHandler(void) {
	USB_DeviceKhciIsrFunction(g_UsbDeviceComposite.deviceHandle);
}
#endif
void USB_DeviceClockInit(void) {
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
	SystemCoreClockUpdate();
	CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0,
			CLOCK_GetFreq(kCLOCK_PllFllSelClk));
	/*
	 * If the SOC has USB KHCI dedicated RAM, the RAM memory needs to be clear after
	 * the KHCI clock is enabled. When the demo uses USB EHCI IP, the USB KHCI dedicated
	 * RAM can not be used and the memory can't be accessed.
	 */
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U))
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS) && (FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS > 0U))
	for (int i = 0; i < FSL_FEATURE_USB_KHCI_USB_RAM; i++)
	{
		((uint8_t *)FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
	}
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS */
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM */
#endif
}
void USB_DeviceIsrEnable(void) {
	uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
	uint8_t usbDeviceKhciIrq[] = USB_IRQS;
	irqNumber = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];
#endif
	/* Install isr, set priority, and enable IRQ. */
	NVIC_SetPriority((IRQn_Type) irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
	EnableIRQ((IRQn_Type) irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
	USB_DeviceKhciTaskFunction(deviceHandle);
#endif
}
#endif

/* The Device callback */
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event,
		void *param) {
	usb_status_t error = kStatus_USB_Error;
	uint16_t *temp16 = (uint16_t *) param;
	uint8_t *temp8 = (uint8_t *) param;

	switch (event) {
	case kUSB_DeviceEventBusReset: {
		/* USB bus reset signal detected */
		g_UsbDeviceComposite.attach = 0U;
		error = kStatus_USB_Success;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
		/* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
		if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_UsbDeviceComposite.speed))
		{
			USB_DeviceSetSpeed(handle, g_UsbDeviceComposite.speed);
		}
#endif
	}
		break;
	case kUSB_DeviceEventSetConfiguration:
		if (param) {
			/* Set device configuration request */
			g_UsbDeviceComposite.attach = 1U;
			g_UsbDeviceComposite.currentConfiguration = *temp8;
			USB_DeviceHidMouseSetConfigure(g_UsbDeviceComposite.hidMouseHandle,
					*temp8);
			USB_DeviceHidKeyboardSetConfigure(
					g_UsbDeviceComposite.hidKeyboardHandle, *temp8);
			error = kStatus_USB_Success;
		}
		break;
	case kUSB_DeviceEventSetInterface:
		if (g_UsbDeviceComposite.attach) {
			/* Set device interface request */
			uint8_t interface = (uint8_t) ((*temp16 & 0xFF00U) >> 0x08U);
			uint8_t alternateSetting = (uint8_t) (*temp16 & 0x00FFU);
			if (interface < USB_COMPOSITE_INTERFACE_COUNT) {
				g_UsbDeviceComposite.currentInterfaceAlternateSetting[interface] =
						alternateSetting;
				USB_DeviceHidMouseSetInterface(
						g_UsbDeviceComposite.hidMouseHandle, interface,
						alternateSetting);
				USB_DeviceHidKeyboardSetInterface(
						g_UsbDeviceComposite.hidKeyboardHandle, interface,
						alternateSetting);
				error = kStatus_USB_Success;
			}
		}
		break;
	case kUSB_DeviceEventGetConfiguration:
		if (param) {
			/* Get current configuration request */
			*temp8 = g_UsbDeviceComposite.currentConfiguration;
			error = kStatus_USB_Success;
		}
		break;
	case kUSB_DeviceEventGetInterface:
		if (param) {
			/* Get current alternate setting of the interface request */
			uint8_t interface = (uint8_t) ((*temp16 & 0xFF00U) >> 0x08U);
			if (interface < USB_COMPOSITE_INTERFACE_COUNT) {
				*temp16 =
						(*temp16 & 0xFF00U)
								| g_UsbDeviceComposite.currentInterfaceAlternateSetting[interface];
				error = kStatus_USB_Success;
			} else {
				error = kStatus_USB_InvalidRequest;
			}
		}
		break;
	case kUSB_DeviceEventGetDeviceDescriptor:
		if (param) {
			/* Get device descriptor request */
			error = USB_DeviceGetDeviceDescriptor(handle,
					(usb_device_get_device_descriptor_struct_t *) param);
		}
		break;
	case kUSB_DeviceEventGetConfigurationDescriptor:
		if (param) {
			/* Get device configuration descriptor request */
			error = USB_DeviceGetConfigurationDescriptor(handle,
					(usb_device_get_configuration_descriptor_struct_t *) param);
		}
		break;
	case kUSB_DeviceEventGetStringDescriptor:
		if (param) {
			/* Get device string descriptor request */
			error = USB_DeviceGetStringDescriptor(handle,
					(usb_device_get_string_descriptor_struct_t *) param);
		}
		break;
	case kUSB_DeviceEventGetHidDescriptor:
		if (param) {
			/* Get hid descriptor request */
			error = USB_DeviceGetHidDescriptor(handle,
					(usb_device_get_hid_descriptor_struct_t *) param);
		}
		break;
	case kUSB_DeviceEventGetHidReportDescriptor:
		if (param) {
			/* Get hid report descriptor request */
			error = USB_DeviceGetHidReportDescriptor(handle,
					(usb_device_get_hid_report_descriptor_struct_t *) param);
		}
		break;
#if (defined(USB_DEVICE_CONFIG_CV_TEST) && (USB_DEVICE_CONFIG_CV_TEST > 0U))
		case kUSB_DeviceEventGetDeviceQualifierDescriptor:
		if (param)
		{
			/* Get Qualifier descriptor request */
			error = USB_DeviceGetDeviceQualifierDescriptor(
					handle, (usb_device_get_device_qualifier_descriptor_struct_t *)param);
		}
		break;
#endif
	case kUSB_DeviceEventGetHidPhysicalDescriptor:
		if (param) {
			/* Get hid physical descriptor request */
			error = USB_DeviceGetHidPhysicalDescriptor(handle,
					(usb_device_get_hid_physical_descriptor_struct_t *) param);
		}
		break;
	default:
		break;
	}

	return error;
}

/* Application initialization */
static void USB_DeviceApplicationInit(void) {
	USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
	SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

	/* Set composite device to default state */
	g_UsbDeviceComposite.speed = USB_SPEED_FULL;
	g_UsbDeviceComposite.attach = 0U;
	g_UsbDeviceComposite.hidMouseHandle = (class_handle_t) NULL;
	g_UsbDeviceComposite.hidKeyboardHandle = (class_handle_t) NULL;
	g_UsbDeviceComposite.deviceHandle = NULL;

	if (kStatus_USB_Success
			!= USB_DeviceClassInit(CONTROLLER_ID,
					&g_UsbDeviceCompositeConfigList,
					&g_UsbDeviceComposite.deviceHandle)) {
		usb_echo("USB device composite demo init failed\r\n");
		return;
	} else {
		usb_echo("USB device composite demo\r\n");
		/* Get the HID keyboard class handle */
		g_UsbDeviceComposite.hidKeyboardHandle =
				g_UsbDeviceCompositeConfigList.config[0].classHandle;
		/* Get the HID mouse class handle */
		g_UsbDeviceComposite.hidMouseHandle =
				g_UsbDeviceCompositeConfigList.config[1].classHandle;

		USB_DeviceHidKeyboardInit(&g_UsbDeviceComposite);
		USB_DeviceHidMouseInit(&g_UsbDeviceComposite);
	}

	USB_DeviceIsrEnable();

	/* Start the device function */
	USB_DeviceRun(g_UsbDeviceComposite.deviceHandle);
}

void TSI0_IRQHandler(void) {
	if (TSI_GetMeasuredChannelNumber(TSI0) == BOARD_TSI_ELECTRODE_2) {
		if (TSI_GetCounter(TSI0)
				> (uint16_t) (buffer.calibratedData[BOARD_TSI_ELECTRODE_2]
						+ TOUCH_DELTA_VALUE)) { /* we are on the right side of the touch slider */
			LED_GREEN_TOGGLE(); /* Toggle the touch event indicating LED */
			typeOfOperation = mail;
		}
	}
	if (TSI_GetMeasuredChannelNumber(TSI0) == BOARD_TSI_ELECTRODE_1) {
		if (TSI_GetCounter(TSI0)
				> (uint16_t) (buffer.calibratedData[BOARD_TSI_ELECTRODE_1]
						+ TOUCH_DELTA_VALUE)) { /* we are on the left side of the touch slider */
			LED_RED_TOGGLE(); /* Toggle the touch event indicating LED */
			typeOfOperation = printscreen;
		}
	}

	/* Clear flags */
	TSI_ClearStatusFlags(TSI0, kTSI_EndOfScanFlag);
	TSI_ClearStatusFlags(TSI0, kTSI_OutOfRangeFlag);
	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
	 exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
#endif

	shouldToggleTsiChannel = true;
}
/*******************************************************************************/
static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortE);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
/* Initialize timer module */
static void Timer_Init(void)
{
    /* convert to match type of data */
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[2];

    /* Configure tpm params with frequency 24kHZ */
    tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL;
    tpmParam[0].level = kTPM_LowTrue;
    tpmParam[0].dutyCyclePercent = 0U;

    tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL;
    tpmParam[1].level = kTPM_LowTrue;
    tpmParam[1].dutyCyclePercent = 0U;

    /* Initialize TPM module */
    TPM_GetDefaultConfig(&tpmInfo);
    TPM_Init(BOARD_TIMER_BASEADDR, &tpmInfo);

    CLOCK_SetTpmClock(1U);

    TPM_SetupPwm(BOARD_TIMER_BASEADDR, tpmParam, 2U, kTPM_EdgeAlignedPwm, 24000U, BOARD_TIMER_SOURCE_CLOCK);
    TPM_StartTimer(BOARD_TIMER_BASEADDR, kTPM_SystemClock);
}

/* Update the duty cycle of an active pwm signal */
static void Board_UpdatePwm(uint16_t x, uint16_t y)
{
    /* Updated duty cycle */
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, x);
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, y);
}






/*!
 * @brief Main function
 */

#if defined(__CC_ARM) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
	mma_handle_t mmaHandle = {0};
    mma_data_t sensorData = {0};
    mma_config_t config = {0}; 
    status_t result; 
    uint8_t sensorRange = 0;
    uint8_t dataScale = 0;
    int16_t xData = 0;
    int16_t yData = 0;
    int16_t xAngle = 0;
    int16_t yAngle = 0;
    int16_t xDuty = 0;
    int16_t yDuty = 0;
    uint8_t idx = 0;
    uint8_t array_addr_size = 0;
    
	volatile uint32_t i = 0;
	tsi_config_t tsiConfig_normal = { 0 };
	lptmr_config_t lptmrConfig;
	memset((void *) &lptmrConfig, 0, sizeof(lptmrConfig));

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
	BOARD_InitDebugConsole();

	USB_DeviceApplicationInit();
	LED_RED_INIT(LOGIC_LED_OFF);
	LED_GREEN_INIT(LOGIC_LED_OFF);

	/* Configure LPTMR */
	/*
	 * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
	 * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
	 * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
	 * lptmrConfig.enableFreeRunning = false;
	 * lptmrConfig.bypassPrescaler = true;
	 * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
	 * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
	 */
	LPTMR_GetDefaultConfig(&lptmrConfig);
	/* TSI default hardware configuration for normal mode */
	TSI_GetNormalModeDefaultConfig(&tsiConfig_normal);

	/* Initialize the LPTMR */
	LPTMR_Init(LPTMR0, &lptmrConfig);
	/* Initialize the TSI */
	TSI_Init(TSI0, &tsiConfig_normal);

	/* Set timer period */
	LPTMR_SetTimerPeriod(LPTMR0,
			USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

	NVIC_EnableIRQ(TSI0_IRQn);
	TSI_EnableModule(TSI0, true); /* Enable module */

	PRINTF("\r\nTSI_V4 Normal_mode Example Start!\r\n");
	/*********  CALIBRATION PROCESS ************/
	memset((void *) &buffer, 0, sizeof(buffer));
	TSI_Calibrate(TSI0, &buffer);
	/* Print calibrated counter values */
	for (i = 0U; i < FSL_FEATURE_TSI_CHANNEL_COUNT; i++) {
		PRINTF("Calibrated counters for channel %d is: %d \r\n", i,
				buffer.calibratedData[i]);
	}

	/********** HARDWARE TRIGGER SCAN ********/
	PRINTF("\r\nNOW, comes to the hardware trigger scan method!\r\n");
	PRINTF(
			"After running, touch pad %s each time, you will see LED toggles.\r\n",
			PAD_TSI_ELECTRODE_1_NAME);
	TSI_EnableModule(TSI0, false);
	TSI_EnableHardwareTriggerScan(TSI0, true);
	TSI_EnableInterrupts(TSI0, kTSI_GlobalInterruptEnable);
	TSI_EnableInterrupts(TSI0, kTSI_EndOfScanInterruptEnable);
	TSI_ClearStatusFlags(TSI0, kTSI_EndOfScanFlag);
	/* Select BOARD_TSI_ELECTRODE_2 as detecting electrode. */
	TSI_SetMeasuredChannelNumber(TSI0, BOARD_TSI_ELECTRODE_2);
	TSI_EnableModule(TSI0, true);
	LPTMR_StartTimer(LPTMR0); /* Start LPTMR triggering */

	//setting default operation to doing nothing
	typeOfOperation = none;
	
	/* I2C initialize */
    BOARD_Accel_I2C_Init();
    /* Configure the I2C function */
    config.I2C_SendFunc = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

    /* Initialize sensor devices */
    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
    for (idx = 0; idx < array_addr_size; idx++)
    {
        config.slaveAddress = g_accel_address[idx];
        /* Initialize accelerometer sensor */
        result = MMA_Init(&mmaHandle, &config);
        if (result == kStatus_Success)
        {
            break;
        }
    }

    if (result != kStatus_Success)
    {
        PRINTF("\r\nSensor device initialize failed!\r\n");
        return -1;
    }
    /* Get sensor range */
    if (MMA_ReadReg(&mmaHandle, kMMA8451_XYZ_DATA_CFG, &sensorRange) != kStatus_Success)
    {
        return -1;
    }
    if (sensorRange == 0x00)
    {
        dataScale = 2U;
    }
    else if (sensorRange == 0x01)
    {
        dataScale = 4U;
    }
    else if (sensorRange == 0x10)
    {
        dataScale = 8U;
    }
    else
    {
    }
    /* Init timer */
    Timer_Init();

    /* Print a note to terminal */
    PRINTF("\r\nWelcome to the BUBBLE example\r\n");
    PRINTF("\r\nYou will see angle data change in the console when change the angles of board\r\n");

	while (1U) {
#if USB_DEVICE_CONFIG_USE_TASK
		USB_DeviceTaskFn(g_UsbDeviceComposite.deviceHandle);
#endif

		if (shouldToggleTsiChannel == true) {
			shouldToggleTsiChannel = false;
			TSI_EnableModule(TSI0, false);
			if (TSI_GetMeasuredChannelNumber(TSI0) == BOARD_TSI_ELECTRODE_2) {
				TSI_SetMeasuredChannelNumber(TSI0, BOARD_TSI_ELECTRODE_1);
			} else {
				TSI_SetMeasuredChannelNumber(TSI0, BOARD_TSI_ELECTRODE_2);
			}
			TSI_EnableModule(TSI0, true);
		}

		if (typeOfOperation == mail) {
			askToSendMail();
			typeOfOperation = none;
		}
		else if (typeOfOperation == printscreen) {
			askToPrintscreen();
			typeOfOperation = none;
		}
	}
}
