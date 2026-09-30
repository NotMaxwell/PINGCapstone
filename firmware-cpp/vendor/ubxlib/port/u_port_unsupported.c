/*
 * I2C, SPI and PPP for ubxlib's ESP-IDF port: all "not supported".
 *
 * This replaces three files of ubxlib's ESP-IDF port:
 *
 * - u_port_i2c.c drives ESP-IDF's legacy I2C driver (driver/i2c.h). The
 *   BNO085 uses the current one (driver/i2c_master.h), and ESP-IDF refuses
 *   to run with both linked in: the legacy driver checks at start-up and
 *   aborts. The GNSS receiver is on a UART, so ubxlib never needs I2C.
 * - u_port_spi.c is unused for the same reason, and dropping it keeps the
 *   SPI master driver out of the build.
 * - u_port_ppp.c would pull in esp_netif and lwIP just to report that PPP is
 *   switched off, which is all it does without U_CFG_PPP_ENABLE.
 *
 * Should the receiver ever move to I2C, port u_port_i2c.c to i2c_master.h
 * rather than bringing the legacy file back.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "u_common_spi.h"
#include "u_error_common.h"
#include "u_port_i2c.h"
#include "u_port_ppp_private.h"
#include "u_port_spi.h"

#define NOT_SUPPORTED ((int32_t) U_ERROR_COMMON_NOT_SUPPORTED)

/* ----------------------------------------------------------------
 * I2C
 * -------------------------------------------------------------- */

int32_t uPortI2cInit(void)
{
    return (int32_t) U_ERROR_COMMON_SUCCESS;
}

void uPortI2cDeinit(void)
{
}

int32_t uPortI2cOpen(int32_t i2c, int32_t pinSda, int32_t pinSdc, bool controller)
{
    (void) i2c;
    (void) pinSda;
    (void) pinSdc;
    (void) controller;
    return NOT_SUPPORTED;
}

int32_t uPortI2cAdopt(int32_t i2c, bool controller)
{
    (void) i2c;
    (void) controller;
    return NOT_SUPPORTED;
}

void uPortI2cClose(int32_t handle)
{
    (void) handle;
}

int32_t uPortI2cCloseRecoverBus(int32_t handle)
{
    (void) handle;
    return NOT_SUPPORTED;
}

int32_t uPortI2cSetClock(int32_t handle, int32_t clockHertz)
{
    (void) handle;
    (void) clockHertz;
    return NOT_SUPPORTED;
}

int32_t uPortI2cGetClock(int32_t handle)
{
    (void) handle;
    return NOT_SUPPORTED;
}

int32_t uPortI2cSetTimeout(int32_t handle, int32_t timeoutMs)
{
    (void) handle;
    (void) timeoutMs;
    return NOT_SUPPORTED;
}

int32_t uPortI2cGetTimeout(int32_t handle)
{
    (void) handle;
    return NOT_SUPPORTED;
}

int32_t uPortI2cControllerSendReceive(int32_t handle, uint16_t address,
                                      const char *pSend, size_t bytesToSend,
                                      char *pReceive, size_t bytesToReceive)
{
    (void) handle;
    (void) address;
    (void) pSend;
    (void) bytesToSend;
    (void) pReceive;
    (void) bytesToReceive;
    return NOT_SUPPORTED;
}

int32_t uPortI2cControllerSend(int32_t handle, uint16_t address,
                               const char *pSend, size_t bytesToSend,
                               bool noStop)
{
    (void) handle;
    (void) address;
    (void) pSend;
    (void) bytesToSend;
    (void) noStop;
    return NOT_SUPPORTED;
}

int32_t uPortI2cResourceAllocCount(void)
{
    return 0;
}

/* ----------------------------------------------------------------
 * SPI
 * -------------------------------------------------------------- */

int32_t uPortSpiInit(void)
{
    return (int32_t) U_ERROR_COMMON_SUCCESS;
}

void uPortSpiDeinit(void)
{
}

int32_t uPortSpiOpen(int32_t spi, int32_t pinMosi, int32_t pinMiso,
                     int32_t pinClk, bool controller)
{
    (void) spi;
    (void) pinMosi;
    (void) pinMiso;
    (void) pinClk;
    (void) controller;
    return NOT_SUPPORTED;
}

void uPortSpiClose(int32_t handle)
{
    (void) handle;
}

int32_t uPortSpiControllerSetDevice(int32_t handle,
                                    const uCommonSpiControllerDevice_t *pDevice)
{
    (void) handle;
    (void) pDevice;
    return NOT_SUPPORTED;
}

int32_t uPortSpiControllerGetDevice(int32_t handle,
                                    uCommonSpiControllerDevice_t *pDevice)
{
    (void) handle;
    (void) pDevice;
    return NOT_SUPPORTED;
}

uint64_t uPortSpiControllerSendReceiveWord(int32_t handle, uint64_t value,
                                           size_t bytesToSendAndReceive)
{
    (void) handle;
    (void) value;
    (void) bytesToSendAndReceive;
    return 0;
}

int32_t uPortSpiControllerSendReceiveBlock(int32_t handle, const char *pSend,
                                           size_t bytesToSend, char *pReceive,
                                           size_t bytesToReceive)
{
    (void) handle;
    (void) pSend;
    (void) bytesToSend;
    (void) pReceive;
    (void) bytesToReceive;
    return NOT_SUPPORTED;
}

int32_t uPortSpiResourceAllocCount(void)
{
    return 0;
}

/* ----------------------------------------------------------------
 * PPP (called from uPortInit()/uPortDeinit())
 * -------------------------------------------------------------- */

int32_t uPortPppPrivateInit(void)
{
    return (int32_t) U_ERROR_COMMON_SUCCESS;
}

void uPortPppPrivateDeinit(void)
{
}

/* ----------------------------------------------------------------
 * GNSS behind a cellular or short-range module
 *
 * The GNSS code asks these for the AT client of an "intermediate" module
 * when the receiver sits behind one. Ours is wired straight to a UART, and
 * upstream only ever builds GNSS alongside the cellular and short-range
 * modules that define them, so they are stubbed here.
 * -------------------------------------------------------------- */

#include "u_device.h"
#include "u_at_client.h"

/* Declared in u_cell.h and u_short_range.h, which drag in the rest of their
 * modules' headers. */
int32_t uCellAtClientHandleGet(uDeviceHandle_t cellHandle,
                               uAtClientHandle_t *pAtHandle);
int32_t uShortRangeAtClientHandleGet(uDeviceHandle_t devHandle,
                                     uAtClientHandle_t *pAtHandle);

int32_t uCellAtClientHandleGet(uDeviceHandle_t cellHandle,
                               uAtClientHandle_t *pAtHandle)
{
    (void) cellHandle;
    (void) pAtHandle;
    return NOT_SUPPORTED;
}

int32_t uShortRangeAtClientHandleGet(uDeviceHandle_t devHandle,
                                     uAtClientHandle_t *pAtHandle)
{
    (void) devHandle;
    (void) pAtHandle;
    return NOT_SUPPORTED;
}
