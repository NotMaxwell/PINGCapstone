/// @file
/// CEVA's reference SPI HAL for sh2, ported from the STM32 to an abstract bus.
///
/// This is `app/spi_hal.c` from CEVA's sh2-demo-nucleo (commit a99d5a95): the
/// same state machine (INIT, DUMMY, IDLE, RD_HDR, RD_BODY, WRITE), the same
/// timings (RESET_DELAY_US, START_DELAY_US), the same INTN-driven transfers
/// that read a READ_LEN header before the body, and the same PS0/WAKE
/// handling. The STM32 HAL calls become the @ref SpiBus interface, which
/// `bno085_hal.cpp` implements on ESP-IDF and the host tests fake, so this
/// file has no hardware dependency. Every deviation from CEVA's code is
/// listed in `src/shtp_spi.cpp`.

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ping::shtp {

// sh2_err.h and sh2_hal.h values, repeated so this file needs no sh2 header;
// bno085_hal.cpp checks they match.
inline constexpr int kSh2Ok = 0;
inline constexpr int kSh2Err = -1;
inline constexpr int kSh2ErrBadParam = -2;
inline constexpr int kSh2ErrIo = -4;
inline constexpr int kSh2ErrTimeout = -6;
inline constexpr std::size_t kMaxTransferIn = 1024;
inline constexpr std::size_t kMaxTransferOut = 128;

/// CEVA's timings.
inline constexpr std::uint32_t kResetDelayUs = 10'000;
inline constexpr std::uint32_t kStartDelayUs = 2'000'000;
inline constexpr std::size_t kReadLen = 4;

/// Not in CEVA's HAL: how long a write may wait for the hub to answer WAKE
/// before it is abandoned (see shtp_spi.cpp).
inline constexpr std::uint32_t kWakeTimeoutUs = 100'000;

/// The hardware operations CEVA's spi_hal.c performs through the STM32 HAL.
/// Pin levels are `true` for high.
struct SpiBus {
    SpiBus() = default;
    SpiBus(const SpiBus&) = delete;
    SpiBus& operator=(const SpiBus&) = delete;
    virtual ~SpiBus() = default;

    /// hal_init_hw: pins (PS0/WAKE, RSTN and CSN driven low, INTN input with
    /// pull-up and a falling-edge interrupt left disabled) and the SPI
    /// peripheral (mode 3). Returns false if the platform refused.
    virtual bool init_hw() = 0;
    /// HAL_SPI_DeInit.
    virtual void deinit_hw() = 0;
    virtual void rstn(bool high) = 0;
    virtual void ps0_waken(bool high) = 0;
    virtual void csn(bool high) = 0;
    /// A blocking full-duplex transfer with CSN left as it is. Returns false
    /// if the bus failed.
    virtual bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) = 0;
    /// The INTN interrupt. While enabled, each falling edge must call
    /// @ref SpiHal::on_intn and then wake @ref wait_int.
    virtual void enable_ints() = 0;
    virtual void disable_ints() = 0;
    /// Blocks until the INTN interrupt has fired since the last call, or
    /// `timeout_us` passes. Replaces CEVA's busy-wait loops.
    virtual void wait_int(std::uint32_t timeout_us) = 0;
    virtual void delay_us(std::uint32_t us) = 0;
    /// A free-running microsecond counter that wraps at 32 bits (CEVA's TIM2).
    virtual std::uint32_t time_now_us() = 0;
};

/// Link counters, kept alongside CEVA's state machine without changing it.
/// All counts are since construction and wrap at 32 bits.
struct LinkCounters {
    /// INTN falling edges the interrupt reported.
    std::uint32_t int_edges;
    /// Low 32 bits of the microsecond clock at the last INTN edge.
    std::uint32_t last_int_us;
    std::uint32_t transfers_ok;
    std::uint32_t transfers_failed;
    /// Cargos with a payload, read whole or clocked in during a write.
    std::uint32_t cargos_read;
    /// Header reads of length 0: INTN without anything to send.
    std::uint32_t empty_headers;
    /// Header reads of length 1 to 4, or all ones (a floating MISO).
    std::uint32_t invalid_headers;
    /// Cargos longer than the receive buffer, truncated as CEVA does.
    std::uint32_t oversized_cargos;
    /// Cargos dropped because sh2's buffer was too small for them.
    std::uint32_t discarded_cargos;
    /// Writes abandoned because the hub never answered WAKE.
    std::uint32_t wake_timeouts;
};

/// The sh2_Hal_t methods of CEVA's SPI HAL, for one BNO08x.
class SpiHal {
  public:
    explicit SpiHal(SpiBus& bus) : bus_(bus) {}
    SpiHal(const SpiHal&) = delete;
    SpiHal& operator=(const SpiHal&) = delete;

    int open();
    void close();
    int read(std::uint8_t* buffer, unsigned len, std::uint32_t* t_us);
    int write(const std::uint8_t* buffer, unsigned len);
    std::uint32_t get_time_us() { return bus_.time_now_us(); }

    /// HAL_GPIO_EXTI_Callback. Safe in interrupt context: it only records the
    /// time and sets flags.
    void on_intn(std::uint32_t now_us)
    {
        int_edges_.fetch_add(1, std::memory_order_relaxed);
        rx_timestamp_us_.store(now_us);
        in_reset_.store(false);
        rx_ready_.store(true);
    }

    /// How long one @ref read may block waiting for INTN when it has nothing
    /// to return. sh2 polls the HAL in tight loops; this lets them sleep.
    void set_read_wait_us(std::uint32_t us) { read_wait_us_ = us; }

    /// Most sh2 operations wait for the hub's answer with no timeout at all.
    /// Once an armed deadline passes, the next @ref read returns a made-up
    /// "reset complete" cargo, which is sh2's own way to abort an operation
    /// (it then returns SH2_ERR); @ref aborted reports that it happened.
    void arm_deadline(std::uint32_t timeout_us);
    void disarm_deadline()
    {
        deadline_armed_ = false;
        aborted_ = false;
    }
    [[nodiscard]] bool aborted() const { return aborted_; }

    /// True after @ref open if the hub never pulled INTN low within
    /// START_DELAY_US of leaving reset.
    [[nodiscard]] bool in_reset() const { return in_reset_.load(); }
    [[nodiscard]] bool is_open() const { return is_open_; }

    [[nodiscard]] LinkCounters counters() const
    {
        LinkCounters out = counters_;
        out.int_edges = int_edges_.load(std::memory_order_relaxed);
        out.last_int_us = rx_timestamp_us_.load();
        return out;
    }

  private:
    enum class SpiState : std::uint8_t { kInit, kDummy, kIdle, kRdHdr, kRdBody, kWrite };

    void spi_dummy_op();
    void spi_activate();
    void spi_completed();
    void start_transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len);
    void run_transfers();
    void reset_delay_us(std::uint32_t delay);
    void count_header(std::uint16_t rx_len);

    SpiBus& bus_;
    SpiState spi_state_ = SpiState::kInit;
    std::atomic<std::uint32_t> rx_timestamp_us_{0};
    std::atomic<bool> in_reset_{false};
    std::atomic<bool> rx_ready_{false};
    std::array<std::uint8_t, kMaxTransferIn> rx_buf_{};
    std::uint32_t rx_buf_len_ = 0;
    std::array<std::uint8_t, kMaxTransferOut> tx_buf_{};
    std::uint32_t tx_buf_len_ = 0;
    bool is_open_ = false;

    // The transfer spi_activate/spi_completed asked for, which CEVA starts
    // with HAL_SPI_TransmitReceive_IT and run_transfers performs.
    const std::uint8_t* pending_tx_ = nullptr;
    std::uint8_t* pending_rx_ = nullptr;
    std::size_t pending_len_ = 0;
    bool transfer_pending_ = false;

    std::uint32_t tx_start_us_ = 0;
    std::uint32_t read_wait_us_ = 0;
    bool deadline_armed_ = false;
    bool aborted_ = false;
    std::uint32_t deadline_start_us_ = 0;
    std::uint32_t deadline_us_ = 0;
    std::atomic<std::uint32_t> int_edges_{0};
    LinkCounters counters_{};
};

} // namespace ping::shtp
