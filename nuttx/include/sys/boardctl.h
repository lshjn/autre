/****************************************************************************
 * include/sys/boardctl.h
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_SYS_BOARDCTL_H
#define __INCLUDE_SYS_BOARDCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_LIB_BOARDCTL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Common commands
 *
 * CMD:           BOARDIOC_INIT
 * DESCRIPTION:   Perform one-time application initialization.
 * ARG:           The boardctl() argument is passed to the
 *                board_app_initialize() implementation without modification.
 *                The argument has no meaning to NuttX; the meaning of the
 *                argument is a contract between the board-specific
 *                initalization logic and the matching application logic.
 *                The value cold be such things as a mode enumeration value,
 *                a set of DIP switch switch settings, a pointer to
 *                configuration data read from a file or serial FLASH, or
 *                whatever you would like to do with it.  Every
 *                implementation should accept zero/NULL as a default
 *                configuration.
 * CONFIGURATION: CONFIG_LIB_BOARDCTL
 * DEPENDENCIES:  Board logic must provide board_app_initialize()
 *
 * CMD:           BOARDIOC_POWEROFF
 * DESCRIPTION:   Power off the board
 * ARG:           Integer value providing power off status information
 * CONFIGURATION: CONFIG_BOARDCTL_POWEROFF
 * DEPENDENCIES:  Board logic must provide the board_power_off() interface.
 *
 * CMD:           BOARDIOC_RESET
 * DESCRIPTION:   Reset the board
 * ARG:           Integer value providing power off status information
 * CONFIGURATION: CONFIG_BOARDCTL_RESET
 * DEPENDENCIES:  Board logic must provide the board_reset() interface.
 *
 * CMD:           BOARDIOC_UNIQUEID
 * DESCRIPTION:   Return a unique ID associated with the board (such as a
 *                serial number or a MAC address).
 * ARG:           A writable array of size CONFIG_BOARDCTL_UNIQUEID_SIZE in
 *                which to receive the board unique ID.
 * DEPENDENCIES:  Board logic must provide the board_uniqueid() interface.
 *
 * CMD:           BOARDIOC_APP_SYMTAB
 * DESCRIPTION:   Select the application symbol table.  This symbol table
 *                provides the symbol definitions exported to application
 *                code from application space.
 * ARG:           A pointer to an instance of struct boardioc_symtab_s
 * CONFIGURATION: CONFIG_BOARDCTL_APP_SYMTAB
 * DEPENDENCIES:  None
 *
 * CMD:           BOARDIOC_OS_SYMTAB
 * DESCRIPTION:   Select the OS symbol table.  This symbol table provides
 *                the symbol definitions exported by the OS to kernel
 *                modules.
 * ARG:           A pointer to an instance of struct boardioc_symtab_s
 * CONFIGURATION: CONFIG_BOARDCTL_OS_SYMTAB
 * DEPENDENCIES:  None
 *
 * CMD:           BOARDIOC_USBDEV_CONTROL
 * DESCRIPTION:   Manage USB device classes
 * ARG:           A pointer to an instance of struct boardioc_usbdev_ctrl_s
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_USBDEVCTRL
 * DEPENDENCIES:  Board logic must provide board_<usbdev>_initialize()
 *
 * CMD:           BOARDIOC_NX_START
 * DESCRIPTION:   Start the NX servier
 * ARG:           None
 * CONFIGURATION: CONFIG_NX_MULTIUSER
 * DEPENDENCIES:  Base graphics logic provides nx_start()
 *
 * CMD:           BOARDIOC_TSCTEST_SETUP
 * DESCRIPTION:   Touchscreen controller test configuration
 * ARG:           Touch controller device minor number
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_TSCTEST
 * DEPENDENCIES:  Board logic must provide board_tsc_setup()
 *
 * CMD:           BOARDIOC_TSCTEST_TEARDOWN
 * DESCRIPTION:   Touchscreen controller test configuration
 * ARG:           None
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_TSCTEST
 * DEPENDENCIES:  Board logic must provide board_tsc_teardown()
 *
 * CMD:           BOARDIOC_GRAPHICS_SETUP
 * DESCRIPTION:   Configure graphics that require special initialization
 *                procedures
 * ARG:           A pointer to an instance of struct boardioc_graphics_s
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_GRAPHICS
 * DEPENDENCIES:  Board logic must provide board_graphics_setup()
 */

#define BOARDIOC_INIT              _BOARDIOC(0x0001)
#define BOARDIOC_POWEROFF          _BOARDIOC(0x0002)
#define BOARDIOC_RESET             _BOARDIOC(0x0003)
#define BOARDIOC_UNIQUEID          _BOARDIOC(0x0004)
#define BOARDIOC_APP_SYMTAB        _BOARDIOC(0x0005)
#define BOARDIOC_OS_SYMTAB         _BOARDIOC(0x0006)
#define BOARDIOC_USBDEV_CONTROL    _BOARDIOC(0x0007)
#define BOARDIOC_NX_START          _BOARDIOC(0x0008)
#define BOARDIOC_TSCTEST_SETUP     _BOARDIOC(0x0009)
#define BOARDIOC_TSCTEST_TEARDOWN  _BOARDIOC(0x000a)
#define BOARDIOC_GRAPHICS_SETUP    _BOARDIOC(0x000b)

/* If CONFIG_BOARDCTL_IOCTL=y, then boad-specific commands will be support.
 * In this case, all commands not recognized by boardctl() will be forwarded
 * to the board-provided board_ioctl() function.
 *
 * User defined board commands may begin with this value:
 */

#define 	BOARDIOC_USER              _BOARDIOC(0x000d)

//add by liushuhe  2017.11.16
#define     BOARDIOC_LED1_ON				BOARDIOC_USER + 1
#define     BOARDIOC_LED1_OFF				BOARDIOC_USER + 2

#define     BOARDIOC_LED2_ON				BOARDIOC_USER + 3
#define     BOARDIOC_LED2_OFF				BOARDIOC_USER + 4

#define     BOARDIOC_LED3_ON				BOARDIOC_USER + 5
#define     BOARDIOC_LED3_OFF				BOARDIOC_USER + 6

//add by liushuhe  2017.11.16
#define     BOARDIOC_I2C_PWRON				BOARDIOC_USER + 7
#define     BOARDIOC_I2C_PWROFF			BOARDIOC_USER + 8

#define     BOARDIOC_VREF_D3V_PWRON		BOARDIOC_USER + 9
#define     BOARDIOC_VREF_D3V_PWROFF		BOARDIOC_USER + 10

#define     BOARDIOC_BRG_CPU_PWRON		BOARDIOC_USER + 11
#define     BOARDIOC_BRG_CPU_PWROFF		BOARDIOC_USER + 12

#define     BOARDIOC_433_A3V_PWRON		BOARDIOC_USER + 13
#define     BOARDIOC_433_A3V_PWROFF		BOARDIOC_USER + 14



/****************************************************************************/
//debug
//add by liushuhe 2017.11.21

#define     BOARDIOC_A12_ON		BOARDIOC_USER + 15
#define     BOARDIOC_A12_OFF		BOARDIOC_USER + 16

#define     BOARDIOC_A8_ON			BOARDIOC_USER + 17
#define     BOARDIOC_A8_OFF		BOARDIOC_USER + 18

#define     BOARDIOC_A11_ON		BOARDIOC_USER + 19
#define     BOARDIOC_A11_OFF		BOARDIOC_USER + 20

#define     BOARDIOC_B2_ON			BOARDIOC_USER + 21
#define     BOARDIOC_B2_OFF		BOARDIOC_USER + 22

#define     BOARDIOC_B10_ON		BOARDIOC_USER + 23
#define     BOARDIOC_B10_OFF		BOARDIOC_USER + 24

#define     BOARDIOC_B8_ON			BOARDIOC_USER + 25
#define     BOARDIOC_B8_OFF		BOARDIOC_USER + 26

#define     BOARDIOC_B9_ON			BOARDIOC_USER + 27
#define     BOARDIOC_B9_OFF		BOARDIOC_USER + 28

#define     BOARDIOC_B1_ON			BOARDIOC_USER + 29
#define     BOARDIOC_B1_OFF		BOARDIOC_USER + 30

#define     BOARDIOC_C5_ON			BOARDIOC_USER + 31
#define     BOARDIOC_C5_OFF		BOARDIOC_USER + 32

#define     BOARDIOC_C12_ON		BOARDIOC_USER + 33
#define     BOARDIOC_C12_OFF		BOARDIOC_USER + 34

#define     BOARDIOC_C8_ON			BOARDIOC_USER + 35
#define     BOARDIOC_C8_OFF		BOARDIOC_USER + 36

#define     BOARDIOC_C6_ON			BOARDIOC_USER + 37
#define     BOARDIOC_C6_OFF		BOARDIOC_USER + 38

#define     BOARDIOC_C9_ON			BOARDIOC_USER + 39
#define     BOARDIOC_C9_OFF		BOARDIOC_USER + 40

#define     BOARDIOC_D2_ON			BOARDIOC_USER + 41
#define     BOARDIOC_D2_OFF		BOARDIOC_USER + 42
//
#define     qBOARDIOC_A12_ON		BOARDIOC_USER + 43
#define     qBOARDIOC_A12_OFF		BOARDIOC_USER + 44

#define     qBOARDIOC_A8_ON			BOARDIOC_USER + 45
#define     qBOARDIOC_A8_OFF		BOARDIOC_USER + 46

#define     qBOARDIOC_A11_ON		BOARDIOC_USER + 47
#define     qBOARDIOC_A11_OFF		BOARDIOC_USER + 48

#define     qBOARDIOC_B2_ON			BOARDIOC_USER + 49
#define     qBOARDIOC_B2_OFF		BOARDIOC_USER + 50

#define     qBOARDIOC_B10_ON		BOARDIOC_USER + 51
#define     qBOARDIOC_B10_OFF		BOARDIOC_USER + 52

#define     qBOARDIOC_B8_ON			BOARDIOC_USER + 53
#define     qBOARDIOC_B8_OFF		BOARDIOC_USER + 54

#define     qBOARDIOC_B9_ON			BOARDIOC_USER + 55
#define     qBOARDIOC_B9_OFF		BOARDIOC_USER + 56

#define     qBOARDIOC_B1_ON			BOARDIOC_USER + 57
#define     qBOARDIOC_B1_OFF		BOARDIOC_USER + 58

#define     qBOARDIOC_C5_ON			BOARDIOC_USER + 59
#define     qBOARDIOC_C5_OFF		BOARDIOC_USER + 60

#define     qBOARDIOC_C12_ON		BOARDIOC_USER + 61
#define     qBOARDIOC_C12_OFF		BOARDIOC_USER + 62

#define     qBOARDIOC_C8_ON			BOARDIOC_USER + 63
#define     qBOARDIOC_C8_OFF		BOARDIOC_USER + 64

#define     qBOARDIOC_C6_ON			BOARDIOC_USER + 65
#define     qBOARDIOC_C6_OFF		BOARDIOC_USER + 66

#define     qBOARDIOC_C9_ON			BOARDIOC_USER + 67
#define     qBOARDIOC_C9_OFF		BOARDIOC_USER + 68

#define     qBOARDIOC_D2_ON			BOARDIOC_USER + 69
#define     qBOARDIOC_D2_OFF		BOARDIOC_USER + 70

#define     qBOARDIOC_LED1_ON				BOARDIOC_USER + 71
#define     qBOARDIOC_LED1_OFF				BOARDIOC_USER + 72

#define     qBOARDIOC_LED2_ON				BOARDIOC_USER + 73
#define     qBOARDIOC_LED2_OFF				BOARDIOC_USER + 74

#define     qBOARDIOC_LED3_ON				BOARDIOC_USER + 75
#define     qBOARDIOC_LED3_OFF				BOARDIOC_USER + 76

#define     qBOARDIOC_B5_ON			BOARDIOC_USER + 77
#define     qBOARDIOC_B11_ON			BOARDIOC_USER + 78



#define     BOARDIOC_B5_ON			BOARDIOC_USER + 79
#define     BOARDIOC_B5_OFF		BOARDIOC_USER + 80

#define     BOARDIOC_B11_ON			BOARDIOC_USER + 81
#define     BOARDIOC_B11_OFF		BOARDIOC_USER + 82


/****************************************************************************/
/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Structure used to pass arguments and get returned values from the
 * BOARDIOC_GRAPHICS_SETUP command.
 */

#ifdef CONFIG_NX_LCDDRIVER
struct lcd_dev_s;                /* Forward reference */
#else
struct fb_vtable_s;              /* Forward reference */
#endif

struct boardioc_graphics_s
{
  int devno;                     /* IN: Graphics device number */
#ifdef CONFIG_NX_LCDDRIVER
  FAR struct lcd_dev_s *dev;     /* OUT: LCD driver instance */
#else
  FAR struct fb_vtable_s *dev;   /* OUT: Framebuffer driver instance */
#endif
};

/* In order to full describe a symbol table, a vector containing the address
 * of the symbol table and the number of elements in the symbol table is
 * required.
 */

struct symtab_s; /* Forward reference */
struct boardioc_symtab_s
{
  FAR struct symtab_s *symtab;
  int nsymbols;
};

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
/* This structure provides the argument BOARDIOC_USBDEV_CONTROL and
 * describes which device should be controlled and what should be
 * done.
 *
 * enum boardioc_usbdev_identifier_e: Identifies the USB device class.
 *   In the case of multiple instances of the USB device class, the
 *   specific instance is identifed by the 'inst' field of the structure.
 *
 * enum boardioc_usbdev_action_e: Identifies the action to peform on
 *   the USB device class instance.
 *
 * struct boardioc_usbdev_ctrl_s:
 *   - usbdev: A value from enum boardioc_usbdev_identifier_e that
 *     identifies the USB device class.
 *   - action: The action to be performed on the USB device class.
 *   - instance:  If there are multiple instances of the USB device
 *     class, this identifies the particular instance.  This is normally
 *     zero but could be non-zero if there are multiple USB device ports
 *     supported by the board.  For CDC/ACM, this is the /dev/ttyACM minor
 *     number; For other devices this would be a port number.
 *   - handle:  This value is returned by the BOARDIOC_USBDEV_CONNECT
 *     action and must be provided to the BOARDIOC_USBDEV_DISCONNECT
 *     action.  It is not used with the BOARDIOC_USBDEV_INITIALIZE action.
 */

enum boardioc_usbdev_identifier_e
{
  BOARDIOC_USBDEV_NONE = 0        /* Not valid */
#ifdef CONFIG_CDCACM
  , BOARDIOC_USBDEV_CDCACM        /* CDC/ACM */
#endif
#ifdef CONFIG_PL2303
  , BOARDIOC_USBDEV_PL2303        /* PL2303 serial */
#endif
#ifdef CONFIG_USBMSC
  , BOARDIOC_USBDEV_MSC           /* Mass storage class */
#endif
#ifdef CONFIG_USBDEV_COMPOSITE
  , BOARDIOC_USBDEV_COMPOSITE     /* Composite device */
#endif
};

enum boardioc_usbdev_action_e
{
  BOARDIOC_USBDEV_INITIALIZE = 0, /* Initialize USB device */
  BOARDIOC_USBDEV_CONNECT,        /* Connect the USB device */
  BOARDIOC_USBDEV_DISCONNECT,     /* Disconnect the USB device */
};

struct boardioc_usbdev_ctrl_s
{
  uint8_t usbdev;                 /* See enum boardioc_usbdev_identifier_e */
  uint8_t action;                 /* See enum boardioc_usbdev_action_e */
  uint8_t instance;               /* Identifies the USB device class instance */
  uint8_t config;                 /* Configuration used with BOARDIOC_USBDEV_CONNECT */
  FAR void **handle;              /* Connection handle */
};
#endif /* CONFIG_BOARDCTL_USBDEVCTRL */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: boardctl
 *
 * Description:
 *   In a small embedded system, there will typically be a much greater
 *   interaction between application and low-level board features.  The
 *   canonically correct to implement such interactions is by implementing a
 *   character driver and performing the interactions via low level ioctl
 *   calls.  This, however, may not be practical in many cases and will lead
 *   to "correct" but awkward implementations.
 *
 *   boardctl() is non-standard OS interface to alleviate the problem.  It
 *   basically circumvents the normal device driver ioctl interlace and allows
 *   the application to perform direct IOCTL-like calls to the board-specific
 *   logic.  It is especially useful for setting up board operational and
 *   test configurations.
 *
 * Input Parameters:
 *   cmd - Identifies the board command to be executed
 *   arg - The argument that accompanies the command.  The nature of the
 *         argument is determined by the specific command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable to to indicate the nature of the failure.
 *
 ****************************************************************************/

int boardctl(unsigned int cmd, uintptr_t arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_LIB_BOARDCTL */
#endif /* __INCLUDE_SYS_BOARDCTL_H */
