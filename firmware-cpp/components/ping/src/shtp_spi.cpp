/*
 * Copyright 2021 CEVA, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and
 * any applicable agreements you may have with CEVA, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * SPI-based HAL for SH2.
 *
 * Modified for PING: ported from CEVA's sh2-demo-nucleo app/spi_hal.c
 * (a99d5a95) to C++ over the abstract SpiBus, with the STM32 HAL calls
 * replaced. Deviations from CEVA's code, all deliberate:
 *
 * - Transfers run in the calling task, not in interrupts. CEVA starts each
 *   transfer with HAL_SPI_TransmitReceive_IT from the INTN interrupt and
 *   chains the next one from the SPI-complete interrupt. ESP-IDF's
 *   spi_master cannot be driven from an ISR, so the INTN ISR only records the
 *   time and sets rxReady (on_intn), and spi_activate/spi_completed run
 *   unchanged in task context: start_transfer records what CEVA would start,
 *   run_transfers performs it and calls spi_completed, from read().
 *   Because spi_activate is never called from the ISR, CEVA's
 *   disableInts/enableInts around it in read() and write() are not needed.
 * - In SPI_WRITE, WAKE is raised after CSN falls but just before the bytes
 *   are clocked (CEVA raises it just after the asynchronous transfer starts).
 * - The busy-wait loops (delayUs, resetDelayUs) block on the bus instead, and
 *   read() may block up to read_wait_us for INTN when it has nothing to
 *   return, so sh2's polling loops sleep rather than spin a FreeRTOS task.
 * - A failed SPI transfer is counted and recovered from (CSN high, back to
 *   IDLE, the transfer's data dropped); CEVA's HAL_SPI_ErrorCallback hangs.
 * - write() abandons a cargo the hub has not collected within
 *   kWakeTimeoutUs (WAKE raised, SH2_ERR_TIMEOUT returned). CEVA returns 0
 *   forever, and sh2's txProcess retries a 0 forever, so a hub that never
 *   answers WAKE would hang the task.
 * - open() fails with SH2_ERR_IO if the platform refuses to set up the bus.
 *   open() returns SH2_OK without INTN as CEVA's does; in_reset() reports it.
 * - The deadline abort (arm_deadline) is added: see shtp_spi.hpp.
 * - The DFU HAL, BOOTN, PS1 and CLKSEL0 are left out: the board has PS1 tied
 *   high and BOOTN pulled up, and never updates the hub's firmware.
 * - The SPI clock is the board's (constants.hpp), not CEVA's 1.3 MHz.
 */

#include "ping/shtp_spi.hpp"

#include <cstring>

namespace ping::shtp {

namespace {

// Dummy transmit data for SPI reads
const std::array<std::uint8_t, kMaxTransferIn> kTxZeros{};

} // namespace

void SpiHal::start_transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len)
{
    pending_tx_ = tx;
    pending_rx_ = rx;
    pending_len_ = len;
    transfer_pending_ = true;
}

void SpiHal::run_transfers()
{
    spi_activate();
    while (transfer_pending_) {
        transfer_pending_ = false;
        if (bus_.transfer(pending_tx_, pending_rx_, pending_len_)) {
            if (is_open_) {
                spi_completed();
            }
            continue;
        }
        ++bus_errors_;
        bus_.csn(true);
        if (spi_state_ == SpiState::kWrite) {
            tx_buf_len_ = 0;
        }
        rx_buf_len_ = 0;
        spi_state_ = SpiState::kIdle;
    }
}

void SpiHal::spi_dummy_op()
{
    // We need to establish SCLK in proper initial state.
    // Do one SPI operation with reset asserted and no CS asserted to get clock sorted.
    std::array<std::uint8_t, 1> dummy_tx{0xAA};
    std::array<std::uint8_t, 1> dummy_rx{};
    (void)bus_.transfer(dummy_tx.data(), dummy_rx.data(), dummy_tx.size());
}

// Attempt to start a SPI operation.
// If SPI periph is not in use and there is data to send or receive,
// this will start a SPI operation.
void SpiHal::spi_activate()
{
    if ((spi_state_ == SpiState::kIdle) && (rx_buf_len_ == 0)) {
        if (rx_ready_.exchange(false)) {
            // assert CSN
            bus_.csn(false);

            if (tx_buf_len_ > 0) {
                spi_state_ = SpiState::kWrite;

                // Start operation to write (and, incidentally, read)
                start_transfer(tx_buf_.data(), rx_buf_.data(), tx_buf_len_);

                // Deassert Wake
                bus_.ps0_waken(true);
            } else {
                spi_state_ = SpiState::kRdHdr;

                // Start SPI operation to read header (writing zeros)
                start_transfer(kTxZeros.data(), rx_buf_.data(), kReadLen);
            }
        }
    }
}

// Handle the end of a SPI operation.
// Depending on spiState, it may start a follow-up operation or transition
// to idle.  In the latter case, it will call spi_activate
void SpiHal::spi_completed()
{
    // Get length of payload available
    auto rx_len = static_cast<std::uint16_t>((rx_buf_[0] + (rx_buf_[1] << 8)) & ~0x8000);

    // Truncate that to max len we can read
    if (rx_len > rx_buf_.size()) {
        rx_len = static_cast<std::uint16_t>(rx_buf_.size());
    }

    if (spi_state_ == SpiState::kDummy) {
        // SPI Dummy operation completed, transition now to idle
        spi_state_ = SpiState::kIdle;
    } else if (spi_state_ == SpiState::kRdHdr) {
        // We read a header

        if (rx_len > kReadLen) {
            // There is more to read

            // Transition to RD_BODY state
            spi_state_ = SpiState::kRdBody;

            // Start a read operation for the remaining length.  (We already read the first READ_LEN bytes.)
            start_transfer(kTxZeros.data(), rx_buf_.data() + kReadLen, rx_len - kReadLen);
        } else {
            // No SHTP payload was received, this operation is done
            bus_.csn(true);               // deassert CSN
            rx_buf_len_ = 0;              // no rx data available
            spi_state_ = SpiState::kIdle; // back to idle state
            spi_activate();               // activate next operation, if any.
        }
    } else if (spi_state_ == SpiState::kRdBody) {
        // We completed the read or write of a payload
        // deassert CSN.
        bus_.csn(true);

        // Check len of data read and set rxBufLen
        rx_buf_len_ = rx_len;

        // transition back to idle state
        spi_state_ = SpiState::kIdle;

        // Activate the next operation, if any.
        spi_activate();
    } else if (spi_state_ == SpiState::kWrite) {
        // We completed the read or write of a payload
        // deassert CSN.
        bus_.csn(true);

        // Since operation was a write, transaction was for txBufLen bytes.  So received
        // data len is, at a maximum, txBufLen.
        rx_buf_len_ = (tx_buf_len_ < rx_len) ? tx_buf_len_ : rx_len;

        // Tx buffer is empty now.
        tx_buf_len_ = 0;

        // transition back to idle state
        spi_state_ = SpiState::kIdle;

        // Activate the next operation, if any.
        spi_activate();
    }
}

void SpiHal::reset_delay_us(std::uint32_t delay)
{
    const std::uint32_t start = bus_.time_now_us();
    std::uint32_t now = start;
    while (((now - start) < delay) && in_reset_.load()) {
        bus_.wait_int(delay - (now - start));
        now = bus_.time_now_us();
    }
}

void SpiHal::arm_deadline(std::uint32_t timeout_us)
{
    deadline_armed_ = true;
    aborted_ = false;
    deadline_start_us_ = bus_.time_now_us();
    deadline_us_ = timeout_us;
}

// ------------------------------------------------------------------------
// SH2 SPI Hal Methods

int SpiHal::open()
{
    int retval = kSh2Ok;

    if (is_open_) {
        // Can't open if another instance is already open
        return kSh2Err;
    }

    is_open_ = true;

    // Init hardware (non-DFU config)
    if (!bus_.init_hw()) {
        is_open_ = false;
        return kSh2ErrIo;
    }

    // Hold in reset
    bus_.rstn(false);

    // deassert CSN
    bus_.csn(true);

    // Clear rx, tx buffers
    rx_buf_len_ = 0;
    tx_buf_len_ = 0;
    rx_ready_.store(false);

    in_reset_.store(true); // will change back to false when INTN serviced

    // Do dummy SPI operation
    // (First SPI op after reconfig has bad initial state of signals
    // so this is a throwaway operation.  Afterward, all is well.)
    spi_state_ = SpiState::kDummy;
    spi_dummy_op();
    spi_state_ = SpiState::kIdle;

    // Delay for RESET_DELAY_US to ensure reset takes effect
    bus_.delay_us(kResetDelayUs);

    // To boot in SHTP-SPI mode, must have PS1=1, PS0=1.
    // PS1 is tied high on the board.
    // So we set PS0_WAKEN signal to 1
    bus_.ps0_waken(true);

    // Deassert reset (BOOTN is pulled up on the board: non-DFU mode)
    bus_.rstn(true);

    // enable interrupts
    bus_.enable_ints();

    // Wait for INTN to be asserted
    reset_delay_us(kStartDelayUs);

    return retval;
}

void SpiHal::close()
{
    // Disable interrupts
    bus_.disable_ints();

    // Set state machine to INIT state
    spi_state_ = SpiState::kInit;

    // Hold sensor hub in reset
    bus_.rstn(false);

    // deassert CSN
    bus_.csn(true);

    // Deinit SPI peripheral
    bus_.deinit_hw();

    transfer_pending_ = false;

    // No longer open
    is_open_ = false;
}

int SpiHal::read(std::uint8_t* buffer, unsigned len, std::uint32_t* t_us)
{
    int retval = 0;

    if (deadline_armed_ && bus_.time_now_us() - deadline_start_us_ >= deadline_us_) {
        deadline_armed_ = false;
        aborted_ = true;
        // Executable channel, "reset complete": sh2 aborts its operation.
        constexpr std::array<std::uint8_t, 5> kResetComplete{5, 0, 1, 0, 1};
        if (len >= kResetComplete.size()) {
            std::memcpy(buffer, kResetComplete.data(), kResetComplete.size());
            *t_us = bus_.time_now_us();
            return static_cast<int>(kResetComplete.size());
        }
    }

    run_transfers();
    if (rx_buf_len_ == 0 && read_wait_us_ > 0 && is_open_) {
        bus_.wait_int(read_wait_us_);
        run_transfers();
    }

    // If there is received data available...
    if (rx_buf_len_ > 0) {
        // And if the data will fit in this buffer...
        if (len >= rx_buf_len_) {
            // Copy data to the client buffer
            std::memcpy(buffer, rx_buf_.data(), rx_buf_len_);
            retval = static_cast<int>(rx_buf_len_);

            // Set timestamp of that data
            *t_us = rx_timestamp_us_.load();

            // Clear rxBuf so we can receive again
            rx_buf_len_ = 0;
        } else {
            // Discard what was read and return error because buffer was too small.
            retval = kSh2ErrBadParam;
            rx_buf_len_ = 0;
        }

        // Now that rxBuf is empty, activate SPI processing to send any
        // potential write that was blocked.
        run_transfers();
    }

    return retval;
}

int SpiHal::write(const std::uint8_t* buffer, unsigned len)
{
    int retval = kSh2Ok;

    // Validate parameters
    if ((len > tx_buf_.size()) || ((len > 0) && (buffer == nullptr))) {
        return kSh2ErrBadParam;
    }

    // If tx buffer is not empty, return 0
    if (tx_buf_len_ != 0) {
        if (bus_.time_now_us() - tx_start_us_ < kWakeTimeoutUs) {
            return 0;
        }
        tx_buf_len_ = 0;
        bus_.ps0_waken(true);
        ++wake_timeouts_;
        return kSh2ErrTimeout;
    }

    // Copy data to tx buffer
    std::memcpy(tx_buf_.data(), buffer, len);
    tx_buf_len_ = len;
    retval = static_cast<int>(len);
    tx_start_us_ = bus_.time_now_us();

    // Assert Wake
    bus_.ps0_waken(false);

    return retval;
}

} // namespace ping::shtp
