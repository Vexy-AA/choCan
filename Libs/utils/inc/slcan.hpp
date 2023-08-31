#pragma once
#ifndef SLCAN_HPP
#define SLCAN_HPP

#include "stdint.h"
#include "ringBuffer.hpp"
#include <stdio.h>
//#include "stm32f4xx_hal.h"
//#include "usbd_cdc_if.h"
//#include "stm32f4xx_hal_can.h"

#include <string.h>
#include "bxcan.hpp"
#include "ch.hpp"
#include "hal.h"
#define HAL_CANFD_SUPPORTED FALSE

#define SLCAN_BUFFER_SIZE 200
#define SLCAN_RX_QUEUE_SIZE 64

#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

#ifndef UID_BASE
#define _STM303

#ifdef _STM303
#define UID_BASE              0x1FFFF7ACU       /*!< Unique device ID register base address */
#elif _STM405
#define UID_BASE                     0x1FFF7A10U           /*!< Unique device ID register base address */
#endif
#endif

#define UDID_START UID_BASE
#ifndef CAN1_BASE
#define CAN1_BASE             (APB1PERIPH_BASE + 0x00006400U)
#endif
#ifndef CAN2_BASE
#define CAN2_BASE             (APB1PERIPH_BASE + 0x00006800U)
#endif

#define PERF_STATS(x) (x++)
#define HAL_CAN_BASE_LIST reinterpret_cast<bxcan::CanType*>(uintptr_t(CAN1_BASE))

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");



bool get_system_id_unformatted(uint8_t buf[], uint8_t &len);


namespace SLCAN
{

typedef uint16_t CanIOFlags;

static const CanIOFlags Loopback = 1;
    static const CanIOFlags AbortOnError = 2;
    static const CanIOFlags IsMAVCAN = 4;

    // Single Rx Frame with related info
    

struct CANFrame {
    static const uint32_t MaskStdID = 0x000007FFU;
    static const uint32_t MaskExtID = 0x1FFFFFFFU;
    static const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    static const uint32_t FlagERR = 1U << 29;                  ///< Error frame

#if HAL_CANFD_SUPPORTED
    static const uint8_t NonFDCANMaxDataLen = 8;
    static const uint8_t MaxDataLen = 64;
#else
    static const uint8_t NonFDCANMaxDataLen = 8;
    static const uint8_t MaxDataLen = 8;
#endif
    uint32_t id;                ///< CAN ID with flags (above)
    union {
        uint8_t data[MaxDataLen];
        uint32_t data_32[MaxDataLen/4];
    };
    uint8_t dlc;                ///< Data Length Code
    bool canfd;

    CANFrame() :
        id(0),
        dlc(0),
        canfd(false)
    {
        memset(data,0, MaxDataLen);
    }

    CANFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len, bool canfd_frame = false);

    bool operator!=(const CANFrame& rhs) const
    {
        return !operator==(rhs);
    }
    bool operator==(const CANFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && (memcmp(data, rhs.data, dlc) == 0);
    }

    // signed version of id, for use by scriping where uint32_t is expensive
    int32_t id_signed(void) const {
        return isExtended()? int32_t(id & MaskExtID) : int32_t(id & MaskStdID);
    }

    bool isExtended()                  const
    {
        return id & FlagEFF;
    }
    bool isRemoteTransmissionRequest() const
    {
        return id & FlagRTR;
    }
    bool isErrorFrame()                const
    {
        return id & FlagERR;
    }
    void setCanFD(bool canfd_frame)
    {
        canfd = canfd_frame;
    }

    bool isCanFDFrame() const
    {
        return canfd;
    }

    static uint8_t dlcToDataLength(uint8_t dlc);

    static uint8_t dataLengthToDlc(uint8_t data_length);
    /**
     * CAN frame arbitration rules, particularly STD vs EXT:
     *     Marco Di Natale - "Understanding and using the Controller Area Network"
     *     http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     */
    bool priorityHigherThan(const CANFrame& rhs) const;
    bool priorityLowerThan(const CANFrame& rhs) const
    {
        return rhs.priorityHigherThan(*this);
    }
};

class CANIface
{
public:
    CANIface():
        rx_queue_(HAL_CAN_RX_QUEUE_SIZE),
        tx_queue_(HAL_CAN_RX_QUEUE_SIZE),
        rxSerial(100)
    {
        rx_queue_.clear();
        tx_queue_.clear();

        pending_tx_[0] = CanTxItem();
        pending_tx_[1] = CanTxItem();
        pending_tx_[2] = CanTxItem();

       // AP_Param::setup_object_defaults(this, var_info);
    }
    int8_t sendSerialByUSB(uint8_t  *pbuff, uint16_t length);
    // Overriden methods
    //bool set_event_handle(EventHandle* evt_handle) ;
    int8_t sendBack(uint8_t* Buf, uint32_t *Len);
    uint16_t getNumFilters() const;
    uint32_t getErrorCount() const;
    //void get_stats(ExpandingString &);
    bool is_busoff() const;
    //bool configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs);
    void flush_tx();
    void clear_rx();
    bool is_initialized() const;
    bool select(bool &read, bool &write,
                const CANFrame* const pending_tx,
                uint64_t blocking_deadline) ;
    int16_t send(const CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) ;

    int16_t receive(CANFrame& out_frame, uint64_t& rx_time,
                    CanIOFlags& out_flags) ;
    int16_t storeSerialMessage(const uint8_t* Buf, uint32_t *Len);
    int16_t storeCanMessage(uint8_t fifo_index, uint64_t timestamp_us);
    int16_t storeCanMessage(CANRxFrame& inFrame);

    enum OperatingMode {
        PassThroughMode,
        NormalMode,
        SilentMode,
        FilteredMode
    };

    OperatingMode get_operating_mode() { return mode_; }

    typedef uint16_t CanIOFlags;
    static const CanIOFlags Loopback = 1;
    static const CanIOFlags AbortOnError = 2;
    static const CanIOFlags IsMAVCAN = 4;


    typedef struct {
        uint32_t tx_requests;
        uint32_t tx_rejected;
        uint32_t tx_overflow;
        uint32_t tx_success;
        uint32_t tx_timedout;
        uint32_t tx_abort;
        uint32_t rx_received;
        uint32_t rx_overflow;
        uint32_t rx_errors;
        uint32_t num_busoff_err;
    } bus_stats_t;

    // Single Rx Frame with related info
    struct CanRxItem {
        uint64_t timestamp_us = 0;
        CanIOFlags flags = 0;
        CANFrame frame;
    };

    // Single Tx Frame with related info
    struct CanTxItem {
        uint64_t deadline = 0;
        CANFrame frame;
        uint32_t index = 0;
        bool loopback:1;
        bool abort_on_error:1;
        bool aborted:1;
        bool pushed:1;
        bool setup:1;
        bool canfd_frame:1;

        bool operator<(const CanTxItem& rhs) const
        {
            if (frame.priorityLowerThan(rhs.frame)) {
                return true;
            }
            if (frame.priorityHigherThan(rhs.frame)) {
                return false;
            }
            return index > rhs.index;
        }
    };

    static uint64_t native_micros64(){
        uint32_t ms = TIME_I2MS(chVTGetSystemTime());
        uint32_t us = SysTick->VAL;

        if (ms != TIME_I2MS(chVTGetSystemTime())){
            ms = TIME_I2MS(chVTGetSystemTime());
            us = SysTick->VAL;
        }
        return ms * 1000 - us / ((SysTick->LOAD + 1) / 1000);
    }
    chibios_rt::Mutex* getRxMutex(){
        return rx_queue_.mutex;
    }
    chibios_rt::Mutex* getTxMutex(){
        return tx_queue_.mutex;
    }
    int16_t sendUsb();
    int16_t serialToCan();
    int16_t sendCan();
private:    
    static constexpr bxcan::CanType* const cans_[1] = {reinterpret_cast<bxcan::CanType*>(uintptr_t(CAN1_BASE))};
    int16_t canFrameSendBySerial(const CANFrame& frame, uint64_t timestamp_usec);

    const char* processCommand(char* cmd);

    // pushes received frame into queue, false if failed
    bool putCanFrameToTxBuffer(CANFrame &frame);

    // Methods to handle different types of frames,
    // return true if successfully received frame type
    bool handle_FrameRTRStd(const char* cmd);
    bool handle_FrameRTRExt(const char* cmd);
    bool handle_FrameDataStd(const char* cmd);
    bool handle_FrameDataExt(const char* cmd, bool canfd);

    bool handle_FDFrameDataExt(const char* cmd);

    // Parsing bytes received on the serial port
    inline void addByte(const uint8_t byte);

    bool initialized_;

    char buf_[SLCAN_BUFFER_SIZE + 1]; // buffer to record raw frame nibbles before parsing
    int16_t pos_ = 0; // position in the buffer recording nibble frames before parsing
    //UARTDriver* _port; // UART interface port reference to be used for SLCAN iface

    ObjectBuffer<CanRxItem> rx_queue_; // Parsed Rx Frame queue
    ObjectBuffer<CanRxItem> tx_queue_; // Parsed Rx Frame queue
    ByteBuffer rxSerial;
    const uint32_t _serial_lock_key = 0x53494442; // Key used to lock UART port for use by slcan

   /*  AP_Int8 _slcan_can_port;
    AP_Int8 _slcan_ser_port;
    AP_Int8 _slcan_timeout;
    AP_Int8 _slcan_start_delay; */

    bool _slcan_start_req;
    uint32_t _slcan_start_req_time;
    int8_t _prev_ser_port;
    int8_t _iface_num = -1;
    uint32_t _last_had_activity;
    uint8_t num_tries;
    //HAL_Semaphore port_sem;
    bool _set_by_sermgr;

    

    //simple variant of std c function to reduce used flash space
    void *memset(void *s, int c, size_t n){
        uint8_t *b = (uint8_t *)s;
        while (n--) {
            *b++ = c;
        }
        return s;
    }
    
    struct bus_stats {
        uint32_t tx_requests;
        uint32_t tx_rejected;
        uint32_t tx_overflow;
        uint32_t tx_success;
        uint32_t tx_timedout;
        uint32_t tx_abort;
        uint32_t rx_received;
        uint32_t rx_overflow;
        uint32_t rx_errors;
        uint32_t num_busoff_err;
        uint32_t num_events;
        uint32_t esr;
    } stats;

    CANFrame isr_rx_frame;
    CanRxItem isr_rx_item;
    bool irq_init_:1;
    bool initialised_:1;
    bool had_activity_:1;

    int16_t canFrameSendByCAN(const CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags);

    CanTxItem pending_tx_[3];
    int8_t usbFree();
protected:
    int8_t get_iface_num() {
        return _iface_num;
    }

    bool add_to_rx_queue(const CanRxItem &frm) {
        return rx_queue_.push(frm);
    }
    bool add_to_tx_queue(const CanRxItem &frm) {
        return tx_queue_.push(frm);
    }
    
    uint32_t bitrate_;
    OperatingMode mode_;
};


}


#endif /* SLCAN_HPP */
