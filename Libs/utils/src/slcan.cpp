#include "slcan.hpp"



//extern USBD_HandleTypeDef hUsbDeviceFS;
//extern CAN_HandleTypeDef hcan;
extern SerialUSBDriver SDU1;
////////Helper Methods//////////
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
static bool hex2nibble_error;

static uint8_t nibble2hex(uint8_t x)
{
    // Allocating in RAM because it's faster
    static uint8_t ConversionTable[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    return ConversionTable[x & 0x0F];
}

static uint8_t hex2nibble(char c)
{
    // Must go into RAM, not flash, because flash is slow
    static uint8_t NumConversionTable[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    };

    static uint8_t AlphaConversionTable[] = {
        10, 11, 12, 13, 14, 15
    };

    uint8_t out = 255;

    if (c >= '0' && c <= '9') {
        out = NumConversionTable[int(c) - int('0')];
    } else if (c >= 'a' && c <= 'f') {
        out = AlphaConversionTable[int(c) - int('a')];
    } else if (c >= 'A' && c <= 'F') {
        out = AlphaConversionTable[int(c) - int('A')];
    }

    if (out == 255) {
        hex2nibble_error = true;
    }
    return out;
}

bool SLCAN::CANIface::putCanFrameToTxBuffer(CANFrame &frame)
{
    CANIface::CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.timestamp_us = native_micros64();
    return add_to_tx_queue(frm);
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
bool SLCAN::CANIface::handle_FrameDataExt(const char* cmd, bool canfd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.canfd = canfd;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    f.dlc = hex2nibble(cmd[9]);
    if (hex2nibble_error || f.dlc > (canfd?15:8)) {
        return false;
    }
    {
        const char* p = &cmd[10];
        const uint8_t dlen = CANFrame::dlcToDataLength(f.dlc);
        for (unsigned i = 0; i < dlen; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return putCanFrameToTxBuffer(f);
}

uint8_t SLCAN::CANFrame::dlcToDataLength(uint8_t dlc)
{
    /*
    Data Length Code      9  10  11  12  13  14  15
    Number of data bytes 12  16  20  24  32  48  64
    */
    if (dlc <= 8) {
        return dlc;
    } else if (dlc == 9) {
        return 12;
    } else if (dlc == 10) {
        return 16;
    } else if (dlc == 11) {
        return 20;
    } else if (dlc == 12) {
        return 24;
    } else if (dlc == 13) {
        return 32;
    } else if (dlc == 14) {
        return 48;
    }
    return 64;
}

uint8_t SLCAN::CANFrame::dataLengthToDlc(uint8_t data_length)
{
    if (data_length <= 8) {
        return data_length;
    } else if (data_length <= 12) {
        return 9;
    } else if (data_length <= 16) {
        return 10;
    } else if (data_length <= 20) {
        return 11;
    } else if (data_length <= 24) {
        return 12;
    } else if (data_length <= 32) {
        return 13;
    } else if (data_length <= 48) {
        return 14;
    }
    return 15;
}
/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
bool SLCAN::CANIface::handle_FDFrameDataExt(const char* cmd)
{
#if HAL_CANFD_SUPPORTED
    return false;
#else
    CANFrame f {};
    hex2nibble_error = false;
    f.canfd = true;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    f.dlc = hex2nibble(cmd[9]);
    if (f.dlc > CANFrame::dataLengthToDlc(CANFrame::MaxDataLen)) {
        return false;
    }
    {
        const char* p = &cmd[10];
        for (unsigned i = 0; i < CANFrame::dlcToDataLength(f.dlc); i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return putCanFrameToTxBuffer(f);
#endif //#if HAL_CANFD_SUPPORTED
}

bool SLCAN::CANIface::handle_FrameDataStd(const char* cmd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.id = (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + CANFrame::NonFDCANMaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc > CANFrame::NonFDCANMaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[5];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return putCanFrameToTxBuffer(f);
}

bool SLCAN::CANIface::handle_FrameRTRExt(const char* cmd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.id = f.FlagEFF | f.FlagRTR |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + CANFrame::NonFDCANMaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';

    if (f.dlc > CANFrame::NonFDCANMaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return putCanFrameToTxBuffer(f);
}

bool SLCAN::CANIface::handle_FrameRTRStd(const char* cmd)
{
    CANFrame f {};
    hex2nibble_error = false;
    f.id = f.FlagRTR |
           (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + CANFrame::NonFDCANMaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc <= CANFrame::NonFDCANMaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return putCanFrameToTxBuffer(f);
}

static inline const char* getASCIIStatusCode(bool status)
{
    return status ? "\r" : "\a";
}


//Accepts command string, returns response string or nullptr if no response is needed.
const char* SLCAN::CANIface::processCommand(char* cmd)
{
    /*
    * High-traffic SLCAN commands go first
    */
    if (cmd[0] == 'T' || cmd[0] == 'D') {
        return handle_FrameDataExt(cmd, cmd[0]=='D') ? "Z\r" : "\a";
    } else if (cmd[0] == 't') {
        return handle_FrameDataStd(cmd) ? "z\r" : "\a";
    } else if (cmd[0] == 'R') {
        return handle_FrameRTRExt(cmd) ? "Z\r" : "\a";
    } else if (cmd[0] == 'r' && cmd[1] <= '9') { // The second condition is needed to avoid greedy matching
        // See long commands below
        return handle_FrameRTRStd(cmd) ? "z\r" : "\a";
    }
#if HAL_CANFD_SUPPORTED 
    else if (cmd[0] == 'D') {
        return handle_FDFrameDataExt(cmd) ? "Z\r" : "\a";
    }
#endif

    uint8_t resp_bytes[40];
    uint16_t resp_len;
    /*
    * Regular SLCAN commands
    */
    switch (cmd[0]) {
    case 'S':               // Set CAN bitrate
    case 'O':               // Open CAN in normal mode
    case 'L':               // Open CAN in listen-only mode
    case 'l':               // Open CAN with loopback enabled
    case 'C':               // Close CAN
    case 'M':               // Set CAN acceptance filter ID
    case 'm':               // Set CAN acceptance filter mask
    case 'U':               // Set UART baud rate, see http://www.can232.com/docs/can232_v3.pdf
    case 'Z': {             // Enable/disable RX and loopback timestamping
        return getASCIIStatusCode(true);    // Returning success for compatibility reasons
    }
    case 'F': {             // Get status flags
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes), "F%02X\r", unsigned(0));    // Returning success for compatibility reasons
        if (resp_len > 0) {
            sendSerialByUSB(resp_bytes,resp_len);
        }
        return nullptr;
    }
    case 'V': {             // HW/SW version
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"V%x%x%x%x\r", 1, 0, 1, 0);
        if (resp_len > 0) {
            sendSerialByUSB(resp_bytes,resp_len);
        }
        return nullptr;
    }
    case 'N': {             // Serial number
        const uint8_t uid_buf_len = 12;
        uint8_t uid_len = uid_buf_len;
        uint8_t unique_id[uid_buf_len];
        char buf[uid_buf_len * 2 + 1] = {'\0'};
        char* pos = &buf[0];
        if (get_system_id_unformatted(unique_id, uid_len)) {
            for (uint8_t i = 0; i < uid_buf_len; i++) {
                *pos++ = nibble2hex(unique_id[i] >> 4);
                *pos++ = nibble2hex(unique_id[i]);
            }
        } 
        *pos++ = '\0';
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"N%s\r", &buf[0]);
        if (resp_len > 0) {
            sendSerialByUSB(resp_bytes,resp_len);
        }
        return nullptr;
    }
    default: {
        break;
    }
    }

    return getASCIIStatusCode(false);
}

// add bytes to parse the received SLCAN Data stream
inline void SLCAN::CANIface::addByte(const uint8_t byte)
{
    /* if (_port == nullptr) {
        return;
    } */
    if ((byte >= 32 && byte <= 126)) {  // Normal printable ASCII character
        if (pos_ < SLCAN_BUFFER_SIZE) {
            buf_[pos_] = char(byte);
            pos_ += 1;
        } else {
            pos_ = 0;   // Buffer overrun; silently drop the data
        }
    } else if (byte == '\r') {  // End of command (SLCAN)

        // Processing the command
        buf_[pos_] = '\0';
        const char* const response = processCommand(reinterpret_cast<char*>(&buf_[0]));
        pos_ = 0;

        // Sending the response if provided
        if (response != nullptr) {
            char resp[10];
            strcpy(resp, response);
            sendSerialByUSB(reinterpret_cast<uint8_t*>(resp),strlen(response));
        }
    } else if (byte == 8 || byte == 127) {  // DEL or BS (backspace)
        if (pos_ > 0) {
            pos_ -= 1;
        }
    } else {    // This also includes Ctrl+C, Ctrl+D
        pos_ = 0;   // Invalid byte - drop the current command
    }
}

// receive method to read the frame recorded in the buffer
int16_t SLCAN::CANIface::receive(CANFrame& out_frame, uint64_t& rx_time,
                                 CanIOFlags& out_flags)
{
    // When in passthrough mode select is handled through can iface
    CanRxItem rx_item;
    /* if (rxSerial.available()){
        uint8_t buffer1[255];
        uint32_t bytesNumber = rxSerial.read(buffer1,255);
        sendSerialByUSB(buffer1,bytesNumber);
    }
    return 1; */
    if (usbFree()){
        if (!rx_queue_.pop(rx_item)) {
            //return 0;
        }else{
            out_frame    = rx_item.frame;
            rx_time = rx_item.timestamp_us;
            out_flags    = rx_item.flags;
            canFrameSendBySerial(out_frame, native_micros64()); 
            return 1;
        }
    

        // We found nothing in HAL's CANIface recieve, so look in SLCANIface

        int32_t nBytes = rxSerial.available();
        // flush bytes from port
        while (nBytes--) {
            uint8_t b;
            rxSerial.read_byte(&b);
            addByte(b);
            if (!tx_queue_.space()) {
                break;
            }
        }
    }
    if (tx_queue_.available()) {
        // if we already have something in buffer transmit it
        CanRxItem frm;
        if (!tx_queue_.peek(frm)) {
            return 0;
        }
        out_frame = frm.frame;
        rx_time = frm.timestamp_us;
        out_flags = frm.flags;
        _last_had_activity = TIME_I2MS(chVTGetSystemTime());
        // Also send this frame over can_iface when in passthrough mode,
        // We just push this frame without caring for priority etc
        bool write = true;
        //_can_iface->select(read, write, &out_frame, 0); // select without blocking


        if (write && canFrameSendByCAN(out_frame, native_micros64() + 100000, out_flags) == 1) {
                tx_queue_.pop();
                num_tries = 0;
        } else if (num_tries > 8) {
            tx_queue_.pop();
            num_tries = 0;
        } else {
            num_tries++;
        }
        
        return 1;
    }
    return 0;
}


bool get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    len = MIN(12, len);
    memcpy(buf, (const void *)UDID_START, len);
    return true;
}

///////// SEND
///////// SERIAL

/**
 * General frame format:
 *  <type> <id> <dlc> <data> [timestamp msec] [flags]
 * Types:
 *  R - RTR extended
 *  r - RTR standard
 *  T - Data extended
 *  t - Data standard
 * Flags:
 *  L - this frame is a loopback frame; timestamp field contains TX timestamp
 */

/// @brief Send serial canframe
/// @param frame 
/// @param timestamp_usec 
/// @return 
// canFrameSendBySerial
int16_t SLCAN::CANIface::canFrameSendBySerial(const CANFrame& frame, uint64_t timestamp_usec)
{
#if HAL_CANFD_SUPPORTED
    constexpr unsigned SLCANMaxFrameSize = 200;
#else
    constexpr unsigned SLCANMaxFrameSize = 40;
#endif
    uint8_t buffer[SLCANMaxFrameSize] = {'\0'};
    uint8_t* p = &buffer[0];
    /*
    * Frame type
    */
    if (frame.isRemoteTransmissionRequest()) {
        *p++ = frame.isExtended() ? 'R' : 'r';
    } else if (frame.isErrorFrame()) {
        return -1;     // Not supported
    }
#if HAL_CANFD_SUPPORTED
    else if (frame.canfd) {
        *p++ = frame.isExtended() ? 'D' : 'd';
    }
#endif 
    else {
        *p++ = frame.isExtended() ? 'T' : 't';
    }

    /*
    * ID
    */
    {
        const uint32_t id = frame.id & frame.MaskExtID;
        if (frame.isExtended()) {
            *p++ = nibble2hex(id >> 28);
            *p++ = nibble2hex(id >> 24);
            *p++ = nibble2hex(id >> 20);
            *p++ = nibble2hex(id >> 16);
            *p++ = nibble2hex(id >> 12);
        }
        *p++ = nibble2hex(id >> 8);
        *p++ = nibble2hex(id >> 4);
        *p++ = nibble2hex(id >> 0);
    }

    /*
    * DLC
    */
    *p++ = nibble2hex(frame.dlc);

    /*
    * Data
    */
    for (unsigned i = 0; i < CANFrame::dlcToDataLength(frame.dlc); i++) {
        const uint8_t byte = frame.data[i];
        *p++ = nibble2hex(byte >> 4);
        *p++ = nibble2hex(byte);
    }

    /*
    * Timestamp
    */
    {
        // SLCAN format - [0, 60000) milliseconds
        const auto slcan_timestamp = uint16_t(timestamp_usec / 1000U);
        *p++ = nibble2hex(slcan_timestamp >> 12);
        *p++ = nibble2hex(slcan_timestamp >> 8);
        *p++ = nibble2hex(slcan_timestamp >> 4);
        *p++ = nibble2hex(slcan_timestamp >> 0);
    }

    /*
    * Finalization
    */
    *p++ = '\r';
    const auto frame_size = unsigned(p - &buffer[0]);

    /* if (_port->txspace() < frame_size) {
        return 0;
    } */
    //Write to Serial
    sendSerialByUSB(&buffer[0], frame_size);
    /* if (!_port->write_locked(&buffer[0], frame_size, _serial_lock_key)) { !!!
        return 0;
    } */
    return 1;
}


/// @brief Send can canframe
/// @param frame 
/// @param tx_deadline 
/// @param flags 
/// @return 
// canFrameSendbyCAN
int16_t SLCAN::CANIface::canFrameSendByCAN(const CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -1;
    }
    bxcan::CanType* can_ = cans_[0];

    /*
        * Seeking for an empty slot
        */
    uint8_t txmailbox = 0xFF;
    if ((can_->TSR & bxcan::TSR_TME0) == bxcan::TSR_TME0) {
        txmailbox = 0;
    } else if ((can_->TSR & bxcan::TSR_TME1) == bxcan::TSR_TME1) {
        txmailbox = 1;
    } else if ((can_->TSR & bxcan::TSR_TME2) == bxcan::TSR_TME2) {
        txmailbox = 2;
    } else {
        PERF_STATS(stats.tx_rejected);
        return 0;       // No transmission for you.
    }

    /*
        * Setting up the mailbox
        */
    bxcan::TxMailboxType& mb = can_->TxMailbox[txmailbox];
    if (frame.isExtended()) {
        mb.TIR = ((frame.id & CANFrame::MaskExtID) << 3) | bxcan::TIR_IDE;
    } else {
        mb.TIR = ((frame.id & CANFrame::MaskStdID) << 21);
    }

    if (frame.isRemoteTransmissionRequest()) {
        mb.TIR |= bxcan::TIR_RTR;
    }

    mb.TDTR = frame.dlc;

    mb.TDHR = frame.data_32[1];
    mb.TDLR = frame.data_32[0];

    mb.TIR |= bxcan::TIR_TXRQ;  // Go.

    /*
        * Registering the pending transmission so we can track its deadline and loopback it as needed
        */
    CanTxItem& txi = pending_tx_[txmailbox];
    txi.deadline       = tx_deadline;
    txi.frame          = frame;
    txi.loopback       = (flags & Loopback) != 0;
    txi.abort_on_error = (flags & AbortOnError) != 0;
    // setup frame initial state
    txi.pushed         = false;
    

    return 1;
}
int8_t SLCAN::CANIface::usbFree(){
    /* if (hUsbDeviceFS.pClassData == nullptr) return -1;
    USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef *) hUsbDeviceFS.pClassData;
    if (hcdc->TxState) return 0; */
    return 1;
}
int8_t SLCAN::CANIface::sendSerialByUSB(uint8_t  *resp_bytes, uint16_t resp_len){
    //txSerial.write(resp_bytes,resp_len);
    chnWriteTimeout(&SDU1, resp_bytes, resp_len, TIME_IMMEDIATE);
    /* if (hUsbDeviceFS.pClassData == nullptr) return -1;
    
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, resp_bytes, resp_len);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS); */
    return 0;
}
/// RECEIVE
/// @brief Receive serial bytes interrupt
/// @param Buf 
/// @param Len 
/// @return 
int16_t SLCAN::CANIface::storeSerialMessage(const uint8_t* Buf, uint32_t *Len){
    if (!(*Len)) return 0;
    if (rxSerial.mutex.tryLock()){
        rxSerial.write(Buf, *Len);
        rxSerial.mutex.unlock();
        return 1;
    }
    return 0;
}
//
//
int16_t SLCAN::CANIface::storeCanMessage(CANRxFrame& canRxFrame){
    CANFrame &frame = isr_rx_frame;

    if (canRxFrame.IDE == 0) {
        frame.id = CANFrame::MaskStdID & canRxFrame.SID;
    } else {
        frame.id = CANFrame::MaskExtID & canRxFrame.EID;
        frame.id |= CANFrame::FlagEFF;
    }
    if (canRxFrame.RTR != 0) {
        frame.id |= CANFrame::FlagRTR;
    }
    frame.dlc = canRxFrame.DLC & 15;
    
    frame.data_32[0] = canRxFrame.data32[0];
    frame.data_32[1] = canRxFrame.data32[1];


    CanRxItem &rx_item = isr_rx_item;
    rx_item.frame = frame;
    rx_item.timestamp_us = canRxFrame.TIME;
    rx_item.flags = 0;

    if (add_to_rx_queue(rx_item)) {
        PERF_STATS(stats.rx_received);
    } else {
        PERF_STATS(stats.rx_overflow);
    }

    had_activity_ = true;

    //pollErrorFlagsFromISR();
    return 0;
}
/// @brief Receive can frame interrupt 
/// @param fifo_index 
/// @param timestamp_us 
/// @return 
int16_t SLCAN::CANIface::storeCanMessage(uint8_t fifo_index, uint64_t timestamp_us){
    bxcan::CanType* can_ = cans_[0];
    volatile uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & bxcan::RFR_FMP_MASK) == 0) {
        return 0;
    }

    
    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & bxcan::RFR_FOVR) != 0) {
        PERF_STATS(stats.rx_errors);
    }

    /*
     * Read the frame contents
     */
    CANFrame &frame = isr_rx_frame;
    const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

    if ((rf.RIR & bxcan::RIR_IDE) == 0) {
        frame.id = CANFrame::MaskStdID & (rf.RIR >> 21);
    } else {
        frame.id = CANFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= CANFrame::FlagEFF;
    }

    if ((rf.RIR & bxcan::RIR_RTR) != 0) {
        frame.id |= CANFrame::FlagRTR;
    }

    frame.dlc = rf.RDTR & 15;

    frame.data[0] = uint8_t(0xFF & (rf.RDLR >> 0));
    frame.data[1] = uint8_t(0xFF & (rf.RDLR >> 8));
    frame.data[2] = uint8_t(0xFF & (rf.RDLR >> 16));
    frame.data[3] = uint8_t(0xFF & (rf.RDLR >> 24));
    frame.data[4] = uint8_t(0xFF & (rf.RDHR >> 0));
    frame.data[5] = uint8_t(0xFF & (rf.RDHR >> 8));
    frame.data[6] = uint8_t(0xFF & (rf.RDHR >> 16));
    frame.data[7] = uint8_t(0xFF & (rf.RDHR >> 24));

    *rfr_reg = bxcan::RFR_RFOM | bxcan::RFR_FOVR | bxcan::RFR_FULL;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    CanRxItem &rx_item = isr_rx_item;
    rx_item.frame = frame;
    rx_item.timestamp_us = timestamp_us;
    rx_item.flags = 0;
    if (add_to_rx_queue(rx_item)) {
        PERF_STATS(stats.rx_received);
    } else {
        PERF_STATS(stats.rx_overflow);
    }

    had_activity_ = true;

    //pollErrorFlagsFromISR();
    return 0;
}

int16_t SLCAN::CANIface::sendUsb(){
    CanRxItem rx_item;
    CANFrame out_frame;
    if (!rx_queue_.mutex->tryLock()) return 0;
    if (!rx_queue_.pop(rx_item)) {
        rx_queue_.mutex->unlock();
        return 0;
    }else{
        out_frame    = rx_item.frame;
        canFrameSendBySerial(out_frame, native_micros64()); \
        rx_queue_.mutex->unlock();
        return 1;
    }
    return 0;
}

int16_t SLCAN::CANIface::serialToCan(){
    if (!rxSerial.mutex.tryLock()) return 0;
    int32_t nBytes = rxSerial.available();
    // flush bytes from port
    while (nBytes--) {
        uint8_t b;
        rxSerial.read_byte(&b);
        addByte(b);
        if (!tx_queue_.space()) {
            break;
        }
    }
    rxSerial.mutex.unlock();
    return 1;
}


int16_t SLCAN::CANIface::sendCan(){
    if (!tx_queue_.mutex->tryLock()) return 0;
    CANFrame out_frame;
    CanIOFlags out_flags;
    if (tx_queue_.available()) {
        // if we already have something in buffer transmit it
        CanRxItem frm;
        if (!tx_queue_.peek(frm)) {
            tx_queue_.mutex->unlock();
            return 0;
        }
        out_frame = frm.frame;
        out_flags = frm.flags;
        _last_had_activity = TIME_I2MS(chVTGetSystemTime());
        // Also send this frame over can_iface when in passthrough mode,
        // We just push this frame without caring for priority etc
        bool write = true;
        //_can_iface->select(read, write, &out_frame, 0); // select without blocking


        if (write && canFrameSendByCAN(out_frame, native_micros64() + 100000, out_flags) == 1) {
                tx_queue_.pop();
                num_tries = 0;
        } else if (num_tries > 8) {
            tx_queue_.pop();
            num_tries = 0;
        } else {
            num_tries++;
        }
        tx_queue_.mutex->unlock();
        return 1;
    }
    tx_queue_.mutex->unlock();
    return 0;
}


