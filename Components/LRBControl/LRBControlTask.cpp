/*
 * LRBControlTask.cpp
 *
 *  Created on: May 20, 2026
 */

/************************************
 * INCLUDES
 ************************************/
#include "LRBControlTask.hpp"
#include "SystemDefines.hpp"
#include "CANTask.hpp"
#include "ControlMessage.hpp"

/************************************
 * FUNCTION DEFINITIONS
 ************************************/
LRBControlTask::LRBControlTask() : Task(LRB_CONTROL_TASK_QUEUE_DEPTH_OBJS)
{
    firstStateSent_ = 0;
}

void LRBControlTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize LRBControlTask twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)LRBControlTask::RunTask,
            (const char*)"LRBControlTask",
            (uint16_t)LRB_CONTROL_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)LRB_CONTROL_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "LRBControlTask::InitTask() - xTaskCreate() failed");
}

void LRBControlTask::Run(void * pvParams)
{
    (void)pvParams;

    while (1) {
        Command cm;
        if (qEvtQueue->ReceiveWait(cm)) {
            HandleCommand(cm);
        }
    }
}

void LRBControlTask::HandleCommand(Command& cm)
{
    if (cm.GetCommand() == TASK_SPECIFIC_COMMAND && cm.GetTaskCommand() == FT_REQUEST_TRANSMIT_STATE) {
        SendRocketState();
    }

    cm.Reset();
}

void LRBControlTask::SendRocketState()
{
    Proto::SystemState::State sysState;
    if (firstStateSent_ < LRB_FLIGHT_TASK_BOOTUP_TELE_CYCLES) {
        if (firstStateSent_ < 1) {
            sysState = Proto::SystemState::State::SYS_UNCAUGHT_RESET;
        } else {
            sysState = Proto::SystemState::State::SYS_BOOTUP_COMPLETE;
        }
        firstStateSent_ += 1;
    } else {
        sysState = Proto::SystemState::State::SYS_NORMAL_OPERATION;
    }

    CANBusTask::Inst().SendSystemState(static_cast<uint8_t>(sysState),
                                       static_cast<uint8_t>(Proto::RocketState::RS_NONE));
}
