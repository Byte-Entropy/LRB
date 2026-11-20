/*
 * CANTask.hpp
 *
 * Unified LRB communication task.
 * - Active transport: RS485 (USART3 / UART::FSB) using EmbeddedProto + CRC16 + COBS.
 * - Virtualized CAN transport: CanAutoNodeDaughter stubbed behind CANBUS_USE_FDCAN.
 * - No SoarOS modifications required.
 */

#ifndef CANBUSTASK_HPP_
#define CANBUSTASK_HPP_

/************************************
 * COMPILE-TIME FEATURE TOGGLES
 ************************************/
// Leave commented out to run over RS485 (USART3).
// Uncomment later when hardware FDCAN / CanAutoNodeDaughter is wired.
// #define CANBUS_USE_FDCAN

// Toggle Debug UART (USART2 via SOAR_PRINT) mirroring of RS485 traffic.
// Comment/uncomment to enable or silence each direction without touching SoarOS.
#define RS485_DEBUG_MIRROR_TX
#define RS485_DEBUG_MIRROR_RX

// Half-duplex echo suppression stub (currently full-duplex; uncomment later if needed).
// #define RS485_HALF_DUPLEX_SUPPRESS_ECHO

/************************************
 * INCLUDES
 ************************************/
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTDriver.hpp"
#include "queue.h"

#ifdef CANBUS_USE_FDCAN
#include "CanAutoNodeDaughter.hpp"
#endif

/************************************
 * MACROS AND DEFINES
 ************************************/
enum CAN_TASK_COMMANDS : uint16_t {
    CAN_TASK_CMD_NONE = 0,
    CAN_TASK_CMD_RS485_RX_COMPLETE,
    CAN_TASK_CMD_TX_PENDING
};

/************************************
 * TYPEDEFS
 ************************************/
enum class CANBusTxType : uint8_t {
    NONE = 0,
    LOAD_CELL,
    TEMPERATURE,
    SYSTEM_STATE
};

struct CANBusTxMsg {
    CANBusTxType type;
    int32_t val1; // LOAD_CELL: nos1_mass (g) | TEMPERATURE: tc1 (C) | SYSTEM_STATE: sys_state
    int32_t val2; // LOAD_CELL: nos2_mass (g) | TEMPERATURE: tc2 (C) | SYSTEM_STATE: rocket_state
};

/************************************
 * CLASS DEFINITIONS
 ************************************/
class CANBusTask : public Task, public UARTReceiverBase
{
public:
    static CANBusTask& Inst() {
        static CANBusTask inst;
        return inst;
    }

    void InitTask();

    // UART RX ISR callback (invoked from USART3_IRQHandler -> Driver::usart3.HandleIRQ_UART)
    void InterruptRxData(uint8_t errors);

    // Thread-safe, zero-malloc outbound API for sensor/control tasks
    bool SendLoadCellGrams(int32_t nos1Grams, int32_t nos2Grams = 0);
    bool SendTemperatureC(int32_t tc1C, int32_t tc2C);
    bool SendSystemState(uint8_t sysState, uint8_t rocketState = 0);

protected:
    static void RunTask(void* pvParams) { CANBusTask::Inst().Run(pvParams); }
    void Run(void* pvParams);
    void HandleCommand(Command& cm);

private:
    CANBusTask();
    CANBusTask(const CANBusTask&);
    CANBusTask& operator=(const CANBusTask&);

    bool EnqueueTxMsg(const CANBusTxMsg& msg);
    void ProcessTxQueue();
    void SendTxMsgRs485(const CANBusTxMsg& msg);
    void SendProtobufFrame(const uint8_t* payload, uint16_t size, uint8_t msgId);
    void TransmitRs485(uint8_t* data, uint16_t len);

    bool ArmRs485Rx();
    void ProcessRs485Rx();

#ifdef CANBUS_USE_FDCAN
    void SendTxMsgCan(const CANBusTxMsg& msg);
    void ReceiveCanData();
    CanAutoNodeDaughter* dau = nullptr;
#endif

    static constexpr uint16_t RS485_PROTO_BUF_SZ   = 256;
    static constexpr uint8_t  RS485_CHECKSUM_BYTES = 2;
    static constexpr uint8_t  RS485_OVERHEAD_BYTES = 1 + RS485_CHECKSUM_BYTES; // 1B msgId + 2B CRC16
    static constexpr uint16_t RS485_MIN_FRAME_LEN  = RS485_OVERHEAD_BYTES + 1;
    static constexpr uint8_t  TX_QUEUE_DEPTH       = 16;

    UARTDriver* const kUart_;
    QueueHandle_t txQueue_;

    // Single-owner static buffers (no heap malloc, no per-caller stack bloat)
    uint8_t rxBuf_[RS485_PROTO_BUF_SZ + 1];
    volatile uint16_t rxIdx_;
    volatile bool rxMsgReady_;
    uint8_t rxChar_;

    uint8_t preCobsBuf_[RS485_PROTO_BUF_SZ + RS485_OVERHEAD_BYTES];
    uint8_t txCobsBuf_[RS485_PROTO_BUF_SZ + RS485_OVERHEAD_BYTES + 4];
    uint8_t rxDecodedBuf_[RS485_PROTO_BUF_SZ];
};

#endif /* CANBUSTASK_HPP_ */
