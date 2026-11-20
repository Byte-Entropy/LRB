/*
 * UARTProtoTask.cpp
 *
 *  Created on: May 20, 2026
 *      Author: Javier
 */

/************************************
 * INCLUDES
 ************************************/
#include <LRBProtoTask.hpp>

#include "LRBControlTask.hpp"
#include "ReadBufferFixedSize.h"
// #include "WatchdogTask.hpp"
#include "TelemetryTask.hpp"

// TODO NEW
//#include "FlashTask.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

/************************************
 * FUNCTION DEFINITIONS
 ************************************/

// Information
// Header: 1 byte message ID, followed by protobuf data, followed by 2 byte checksum (pre-COBS)

/**
 * @brief Initialize the PBBProtocolTask
 */
void LRBProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize LRBProtocolTask twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)LRBProtocolTask::RunTask,
            (const char*)"LRBProtocolTask",
            (uint16_t)TASK_PROTOCOL_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_UART_PROTOCOL_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "LRBProtocolTask::InitTask - xTaskCreate() failed");
}

/**
 * @brief Default constructor
 */
LRBProtocolTask::LRBProtocolTask() : ProtocolTask(
        Proto::Node::NODE_LRB,
        UART::FSB,
//        UART_TASK_COMMAND_SEND_LRB)
		UART_TASK_COMMAND_SEND_DEBUG)
{
}

/**
 * @brief Handle a command message
 */
void LRBProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    Proto::CommandMessage msg;
    msg.deserialize(readBuffer);

    // LRB does not route external commands yet.
    (void)msg;
}

/**
 * @brief Handle a control message
 */
void LRBProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    Proto::ControlMessage msg;
    msg.deserialize(readBuffer);

    // Verify the source and target nodes, if they aren't as expected, do nothing
    if (msg.get_source() != Proto::Node::NODE_FSB)
        return;

    // Handle based on the message type
    if(msg.has_hb()) {
        // Heartbeat handling is optional for LRB right now
    }
    else if(msg.has_ping()) {
        // This is a ping message, respond with an ack
        Proto::ControlMessage ackResponse;
        Proto::AckNack ack;
        ack.set_acking_msg_source(msg.get_source());
        ack.set_acking_sequence_num(msg.get_source_sequence_num());
        ackResponse.set_ack(ack);
        EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuf;
        ackResponse.serialize(writeBuf);
        LRBProtocolTask::SendProtobufMessage(writeBuf, Proto::MessageID::MSG_CONTROL);
    }
    else if(msg.has_sys_ctrl()) {
        // This is a system command, handle it
        if (msg.get_sys_ctrl().get_sys_cmd() == Proto::SystemControl::Command::SYS_FLASH_LOG_ENABLE)
        {
            // TODO
        }
        else if(msg.get_sys_ctrl().get_sys_cmd() == Proto::SystemControl::Command::SYS_FLASH_LOG_DISABLE)
        {
            // TODO
        }
        else if(msg.get_sys_ctrl().get_sys_cmd() == Proto::SystemControl::Command::SYS_RESET)
        {
            // This is a request to reset the system
            SOAR_ASSERT(false, "System reset requested!");
        }
        else if (msg.get_sys_ctrl().get_sys_cmd() == Proto::SystemControl::Command::SYS_CRITICAL_FLASH_FULL_ERASE)
        {
            // This is a request that will erase all flash memory, and cause the flash task to stall!
            // TODO NEW
//            FlashTask::Inst().SendCommand(Command(TASK_SPECIFIC_COMMAND, ERASE_ALL_FLASH));
        }
        else if(msg.get_sys_ctrl().get_sys_cmd() == Proto::SystemControl::Command::SYS_LOG_PERIOD_CHANGE)
        {
            uint32_t paramMs = msg.get_sys_ctrl().get_cmd_param();
            paramMs = (paramMs > 0xFFFF) ? 0xFFFE : paramMs;
            TelemetryTask::Inst().SendCommand(Command(TELEMETRY_CHANGE_PERIOD, paramMs));
        }
    }
}

/**
 * @brief Handle a telemetry message
 */
void LRBProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    // Deserialize the generic telemetry wrapper
    Proto::TelemetryMessage msg;
    msg.deserialize(readBuffer);

    // Only process messages intended for this node or broadcast
    if (msg.get_target() != Proto::Node::NODE_LRB && msg.get_target() != Proto::Node::NODE_ANY) {
        return;
    }

    // Thermocouple / Temperature messages
    if (msg.has_lrbTemperature()) {
        const Proto::LrbTemperature &t = msg.get_lrbTemperature();
        // Values are in integer degrees Celsius as defined in the proto
        SOAR_PRINT("PROTO: Received LRB temperatures tc1=%d C, tc2=%d C\n", t.get_tc1_temperature(), t.get_tc2_temperature());
        // TODO: route to telemetry logging or local tasks as needed
    }

   if (msg.has_nosLoadCell()) {
		const Proto::NosLoadCell &nlc = msg.get_nosLoadCell();
		SOAR_PRINT("PROTO: LRB Load Cell (NOS1) mass=%d grams\n", nlc.get_nos1_mass());
	}

	if (msg.has_nosLoadCell()) {
		const Proto::NosLoadCell &nlc = msg.get_nosLoadCell();
		SOAR_PRINT("PROTO: LRB Load Cell (NOS1) mass=%d grams\n", nlc.get_nos1_mass());

		// second NAU7802 channel --> print nlc.get_nos2_mass() here too
	}

    // Anemometer: not yet in the telemetry proto; keep placeholder log entry for future support
    // TODO: If an Anemometer proto message is added, handle it here.
}
