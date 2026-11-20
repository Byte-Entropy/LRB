/*
 * CANTask.cpp
 *
 * Unified LRB communication task.
 * - Runs RS485 (USART3 / UART::FSB) with EmbeddedProto + CRC16 + COBS directly.
 * - Virtualizes FDCAN / CanAutoNodeDaughter behind CANBUS_USE_FDCAN for later use.
 * - Replaces the need to run LRBProtocolTask or TelemetryTask.
 */

/************************************
 * INCLUDES
 ************************************/
#include "CANTask.hpp"
#include "CubeUtils.hpp"
#include <cstring>

#include "cobs.h"
#include "CoreProto.h"
#include "ControlMessage.hpp"
#include "CommandMessage.hpp"
#include "TelemetryMessage.hpp"
#include "WriteBufferFixedSize.h"
#include "ReadBufferFixedSize.h"

#ifdef CANBUS_USE_FDCAN
#include "CANBusTargets.hpp"
#include "SOLLogs.hpp"
extern FDCAN_HandleTypeDef hfdcan1;
#endif

/************************************
 * FUNCTION DEFINITIONS
 ************************************/

CANBusTask::CANBusTask()
    : Task(TASK_CAN_QUEUE_DEPTH_OBJS),
      kUart_(UART::FSB),
      txQueue_(nullptr),
      rxIdx_(0),
      rxMsgReady_(false),
      rxChar_(0)
{
    memset(rxBuf_, 0, sizeof(rxBuf_));
    memset(preCobsBuf_, 0, sizeof(preCobsBuf_));
    memset(txCobsBuf_, 0, sizeof(txCobsBuf_));
    memset(rxDecodedBuf_, 0, sizeof(rxDecodedBuf_));
}

void CANBusTask::InitTask()
{
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize CANBusTask twice");

    txQueue_ = xQueueCreate(TX_QUEUE_DEPTH, sizeof(CANBusTxMsg));
    SOAR_ASSERT(txQueue_ != nullptr, "CANBusTask::InitTask() - xQueueCreate() failed");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)CANBusTask::RunTask,
            (const char*)"CANBusTask",
            (uint16_t)TASK_CAN_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_CAN_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "CANBusTask::InitTask() - xTaskCreate() failed");

#ifdef CANBUS_USE_FDCAN
    uint8_t BOARD_TYPE = CAN_GSE_TARGET_PTX_BOARDTYPE;
    uint8_t SLOT_NUMBER = 123;
    dau = new CanAutoNodeDaughter{&hfdcan1,
        SOL_Init,
        SOL_COUNT,
        BOARD_TYPE,
        SLOT_NUMBER,
        CAN_GSE_TARGET_PT1
    };
#endif
}

/************************************
 * THREAD-SAFE OUTBOUND API
 ************************************/

bool CANBusTask::EnqueueTxMsg(const CANBusTxMsg& msg)
{
    if (txQueue_ == nullptr) {
        return false;
    }

    if (xQueueSend(txQueue_, &msg, 0) != pdPASS) {
        return false;
    }

    // Wake CANBusTask immediately without polling delay
    Command wakeCmd(DATA_COMMAND, CAN_TASK_CMD_TX_PENDING);
    qEvtQueue->Send(wakeCmd);
    return true;
}

bool CANBusTask::SendLoadCellGrams(int32_t nos1Grams, int32_t nos2Grams)
{
    CANBusTxMsg msg{CANBusTxType::LOAD_CELL, nos1Grams, nos2Grams};
    return EnqueueTxMsg(msg);
}

bool CANBusTask::SendTemperatureC(int32_t tc1C, int32_t tc2C)
{
    CANBusTxMsg msg{CANBusTxType::TEMPERATURE, tc1C, tc2C};
    return EnqueueTxMsg(msg);
}

bool CANBusTask::SendSystemState(uint8_t sysState, uint8_t rocketState)
{
    CANBusTxMsg msg{CANBusTxType::SYSTEM_STATE, static_cast<int32_t>(sysState), static_cast<int32_t>(rocketState)};
    return EnqueueTxMsg(msg);
}

/************************************
 * TASK RUN LOOP
 ************************************/

void CANBusTask::Run(void* pvParams)
{
    (void)pvParams;

#ifdef CANBUS_USE_FDCAN
    dau->TryRequestingJoiningNetwork();
    while (1) {
        if (dau->GetCurrentState() == dau->READY) {
            ReceiveCanData();
            ProcessTxQueue();
        } else {
            osDelay(1000);
            dau->TryRequestingJoiningNetwork();
        }

        Command cm;
        if (qEvtQueue->Receive(cm, 10)) {
            HandleCommand(cm);
        }
    }
#else
    // Arm USART3 (RS485) RX interrupt
    ArmRs485Rx();

    while (1) {
        Command cm;
        qEvtQueue->ReceiveWait(cm);
        HandleCommand(cm);
    }
#endif
}

void CANBusTask::HandleCommand(Command& cm)
{
    if (cm.GetCommand() == DATA_COMMAND) {
        switch (cm.GetTaskCommand()) {
        case CAN_TASK_CMD_RS485_RX_COMPLETE:
            ProcessRs485Rx();
            break;
        case CAN_TASK_CMD_TX_PENDING:
            ProcessTxQueue();
            break;
        default:
            SOAR_PRINT("CANBusTask - Unsupported DATA_COMMAND {%d}\n", cm.GetTaskCommand());
            break;
        }
    } else {
        SOAR_PRINT("CANBusTask - Unsupported Command {%d}\n", cm.GetCommand());
    }

    cm.Reset();
}

/************************************
 * TX DISPATCH & RS485 FRAMING
 ************************************/

void CANBusTask::ProcessTxQueue()
{
    CANBusTxMsg msg;
    while (xQueueReceive(txQueue_, &msg, 0) == pdPASS) {
#ifdef CANBUS_USE_FDCAN
        SendTxMsgCan(msg);
#else
        SendTxMsgRs485(msg);
#endif
    }
}

void CANBusTask::SendTxMsgRs485(const CANBusTxMsg& msg)
{
    EmbeddedProto::WriteBufferFixedSize<RS485_PROTO_BUF_SZ> writeBuf;

    switch (msg.type) {
    case CANBusTxType::LOAD_CELL: {
        Proto::TelemetryMessage teleMsg;
        teleMsg.set_source(Proto::Node::NODE_LRB);
        teleMsg.set_target(Proto::Node::NODE_FSB);

        Proto::NosLoadCell lcMsg;
        lcMsg.set_nos1_mass(msg.val1);
        teleMsg.set_nosLoadCell(lcMsg);

        teleMsg.serialize(writeBuf);
#ifdef RS485_DEBUG_MIRROR_TX
        SOAR_PRINT("[RS485 TX] LoadCell nos1=%ld g (nos2=%ld g)\n",
                   static_cast<long>(msg.val1), static_cast<long>(msg.val2));
#endif
        SendProtobufFrame(writeBuf.get_data(), writeBuf.get_size(),
                          static_cast<uint8_t>(Proto::MessageID::MSG_TELEMETRY));
        break;
    }
    case CANBusTxType::TEMPERATURE: {
        Proto::TelemetryMessage teleMsg;
        teleMsg.set_source(Proto::Node::NODE_LRB);
        teleMsg.set_target(Proto::Node::NODE_FSB);

        Proto::LrbTemperature tempMsg;
        tempMsg.set_tc1_temperature(msg.val1);
        tempMsg.set_tc2_temperature(msg.val2);
        teleMsg.set_lrbTemperature(tempMsg);

        teleMsg.serialize(writeBuf);
#ifdef RS485_DEBUG_MIRROR_TX
        SOAR_PRINT("[RS485 TX] Temp tc1=%ld C, tc2=%ld C\n",
                   static_cast<long>(msg.val1), static_cast<long>(msg.val2));
#endif
        SendProtobufFrame(writeBuf.get_data(), writeBuf.get_size(),
                          static_cast<uint8_t>(Proto::MessageID::MSG_TELEMETRY));
        break;
    }
    case CANBusTxType::SYSTEM_STATE: {
        Proto::ControlMessage ctrlMsg;
        ctrlMsg.set_source(Proto::Node::NODE_LRB);
        ctrlMsg.set_target(Proto::Node::NODE_FSB);

        Proto::SystemState stateMsg;
        stateMsg.set_sys_state(static_cast<Proto::SystemState::State>(msg.val1));
        stateMsg.set_rocket_state(static_cast<Proto::RocketState>(msg.val2));
        ctrlMsg.set_sys_state(stateMsg);

        ctrlMsg.serialize(writeBuf);
#ifdef RS485_DEBUG_MIRROR_TX
        SOAR_PRINT("[RS485 TX] SysState sys=%ld rocket=%ld\n",
                   static_cast<long>(msg.val1), static_cast<long>(msg.val2));
#endif
        SendProtobufFrame(writeBuf.get_data(), writeBuf.get_size(),
                          static_cast<uint8_t>(Proto::MessageID::MSG_CONTROL));
        break;
    }
    default:
        break;
    }
}

void CANBusTask::SendProtobufFrame(const uint8_t* payload, uint16_t size, uint8_t msgId)
{
    const uint16_t preCobsSize = size + RS485_OVERHEAD_BYTES;
    if (preCobsSize > sizeof(preCobsBuf_)) {
        SOAR_PRINT("CANBusTask - TX payload too large (%u B)\n", size);
        return;
    }

    // Frame: [1B msgId][protobuf payload][2B CRC16]
    preCobsBuf_[0] = msgId;
    if (size > 0) {
        memcpy(&preCobsBuf_[1], payload, size);
    }
    const uint16_t crc = Utils::GetCRC16(preCobsBuf_, size + 1);
    memcpy(&preCobsBuf_[preCobsSize - RS485_CHECKSUM_BYTES], &crc, sizeof(crc));

    // COBS encode + 0x00 delimiter
    const cobs_encode_result enc = cobs_encode(txCobsBuf_, sizeof(txCobsBuf_) - 1, preCobsBuf_, preCobsSize);
    if (enc.status != COBS_ENCODE_OK) {
        SOAR_PRINT("CANBusTask - COBS encode failed (%d)\n", enc.status);
        return;
    }

    txCobsBuf_[enc.out_len] = 0x00;
    const uint16_t txLen = static_cast<uint16_t>(enc.out_len + 1);

    TransmitRs485(txCobsBuf_, txLen);
}

void CANBusTask::TransmitRs485(uint8_t* data, uint16_t len)
{
    // Note: USART3 is initialized with HAL_RS485Ex_Init(), so hardware automatically
    // asserts/de-asserts PB14 (USART3_DE_FSB) around transmission.

#ifdef RS485_HALF_DUPLEX_SUPPRESS_ECHO
    // Half-duplex stub: uncomment RS485_HALF_DUPLEX_SUPPRESS_ECHO in CANTask.hpp if
    // the transceiver echoes TX bytes back into RX.
    // LL_USART_DisableIT_RXNE(USART3);
#endif

    kUart_->Transmit(data, len);

#ifdef RS485_HALF_DUPLEX_SUPPRESS_ECHO
    // Flush any echoed byte and re-enable RX interrupt
    // if (LL_USART_IsActiveFlag_RXNE(USART3)) {
    //     (void)LL_USART_ReceiveData8(USART3);
    // }
    // LL_USART_EnableIT_RXNE(USART3);
#endif
}

/************************************
 * RS485 RX ISR & DECODING
 ************************************/

bool CANBusTask::ArmRs485Rx()
{
    return kUart_->ReceiveIT(&rxChar_, this);
}

void CANBusTask::InterruptRxData(uint8_t errors)
{
    (void)errors;

    if (!rxMsgReady_) {
        uint16_t idx = rxIdx_;
        if (rxChar_ == 0x00 || idx >= RS485_PROTO_BUF_SZ) {
            if (idx < RS485_MIN_FRAME_LEN) {
                rxIdx_ = 0;
                rxMsgReady_ = false;
            } else {
                rxBuf_[idx] = 0x00;
                rxIdx_ = idx + 1;
                rxMsgReady_ = true;

                Command cm(DATA_COMMAND, CAN_TASK_CMD_RS485_RX_COMPLETE);
                if (!qEvtQueue->SendFromISR(cm)) {
                    rxIdx_ = 0;
                    rxMsgReady_ = false;
                }
            }
        } else {
            rxBuf_[idx] = rxChar_;
            rxIdx_ = idx + 1;
        }
    }

    ArmRs485Rx();
}

void CANBusTask::ProcessRs485Rx()
{
    const uint16_t rawLen = rxIdx_;
    if (rawLen < RS485_MIN_FRAME_LEN) {
        rxIdx_ = 0;
        rxMsgReady_ = false;
        return;
    }

    // Decode COBS frame (exclude trailing 0x00 delimiter)
    const cobs_decode_result dec = cobs_decode(rxDecodedBuf_, sizeof(rxDecodedBuf_), rxBuf_, rawLen - 1);

    // Release RX buffer for next incoming frame immediately
    rxIdx_ = 0;
    rxMsgReady_ = false;

    if (dec.status != COBS_DECODE_OK || dec.out_len < RS485_MIN_FRAME_LEN) {
#ifdef RS485_DEBUG_MIRROR_RX
        SOAR_PRINT("[RS485 RX] COBS decode failed (status=%d len=%u)\n",
                   dec.status, static_cast<unsigned>(dec.out_len));
#endif
        return;
    }

    // Validate CRC16
    const uint16_t payloadWithIdLen = static_cast<uint16_t>(dec.out_len - RS485_CHECKSUM_BYTES);
    uint16_t rxCrc = 0;
    memcpy(&rxCrc, &rxDecodedBuf_[payloadWithIdLen], sizeof(rxCrc));

    if (!Utils::IsCrc16Correct(rxDecodedBuf_, payloadWithIdLen, rxCrc)) {
#ifdef RS485_DEBUG_MIRROR_RX
        SOAR_PRINT("[RS485 RX] CRC16 invalid (len=%u)\n", static_cast<unsigned>(dec.out_len));
#endif
        return;
    }

    const Proto::MessageID msgId = static_cast<Proto::MessageID>(rxDecodedBuf_[0]);
    const uint16_t protoLen = payloadWithIdLen - 1;

    EmbeddedProto::ReadBufferFixedSize<RS485_PROTO_BUF_SZ> readBuf;
    for (uint16_t i = 0; i < protoLen; ++i) {
        readBuf.push(rxDecodedBuf_[1 + i]);
    }

    switch (msgId) {
    case Proto::MessageID::MSG_CONTROL: {
        Proto::ControlMessage msg;
        msg.deserialize(readBuf);
#ifdef RS485_DEBUG_MIRROR_RX
        SOAR_PRINT("[RS485 RX] MSG_CONTROL src=%d target=%d\n",
                   static_cast<int>(msg.get_source()), static_cast<int>(msg.get_target()));
#endif
        if (msg.get_source() != Proto::Node::NODE_FSB) {
            break;
        }
        if (msg.has_ping()) {
            Proto::ControlMessage ackResp;
            ackResp.set_source(Proto::Node::NODE_LRB);
            ackResp.set_target(Proto::Node::NODE_FSB);
            Proto::AckNack ack;
            ack.set_acking_msg_source(msg.get_source());
            ack.set_acking_sequence_num(msg.get_source_sequence_num());
            ackResp.set_ack(ack);

            EmbeddedProto::WriteBufferFixedSize<RS485_PROTO_BUF_SZ> ackBuf;
            ackResp.serialize(ackBuf);
            SendProtobufFrame(ackBuf.get_data(), ackBuf.get_size(),
                              static_cast<uint8_t>(Proto::MessageID::MSG_CONTROL));
        } else if (msg.has_sys_ctrl()) {
            if (msg.get_sys_ctrl().get_sys_cmd() == Proto::SystemControl::Command::SYS_RESET) {
                SOAR_ASSERT(false, "System reset requested over RS485!");
            }
        }
        break;
    }
    case Proto::MessageID::MSG_TELEMETRY: {
        Proto::TelemetryMessage msg;
        msg.deserialize(readBuf);
        if (msg.get_target() != Proto::Node::NODE_LRB && msg.get_target() != Proto::Node::NODE_ANY) {
            break;
        }
#ifdef RS485_DEBUG_MIRROR_RX
        if (msg.has_lrbTemperature()) {
            const Proto::LrbTemperature& t = msg.get_lrbTemperature();
            SOAR_PRINT("[RS485 RX] Temp tc1=%ld C, tc2=%ld C\n",
                       static_cast<long>(t.get_tc1_temperature()),
                       static_cast<long>(t.get_tc2_temperature()));
        }
        if (msg.has_nosLoadCell()) {
            const Proto::NosLoadCell& nlc = msg.get_nosLoadCell();
            SOAR_PRINT("[RS485 RX] LoadCell nos1=%ld g\n",
                       static_cast<long>(nlc.get_nos1_mass()));
        }
#endif
        break;
    }
    case Proto::MessageID::MSG_COMMAND: {
        Proto::CommandMessage msg;
        msg.deserialize(readBuf);
#ifdef RS485_DEBUG_MIRROR_RX
        SOAR_PRINT("[RS485 RX] MSG_COMMAND (%u B)\n", protoLen);
#endif
        (void)msg;
        break;
    }
    default:
#ifdef RS485_DEBUG_MIRROR_RX
        SOAR_PRINT("[RS485 RX] Unknown msgId=%d (%u B)\n", static_cast<int>(msgId), protoLen);
#endif
        break;
    }
}

/************************************
 * VIRTUALIZED FDCAN BACKEND (STUB)
 ************************************/
#ifdef CANBUS_USE_FDCAN
void CANBusTask::SendTxMsgCan(const CANBusTxMsg& msg)
{
    // TODO: Map CANBusTxMsg types to LRB LogIDs when FDCAN / CanAutoNodeDaughter is wired.
    // Example:
    // dau->SendMessageToMotherboardByLogID(LRB_LOGID, reinterpret_cast<const uint8_t*>(&payload));
    (void)msg;
}

void CANBusTask::ReceiveCanData()
{
    dau->CheckCANCommands();
    // TODO: Read incoming LRB CAN commands by log index when defined.
}
#endif
