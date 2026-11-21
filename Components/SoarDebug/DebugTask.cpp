/**
 ******************************************************************************
 * File Name          : DebugTask.cpp
 * Description        : Task for controlling debug input
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <SoarDebug/Inc/DebugTask.hpp>
#include "Command.hpp"
#include "CubeUtils.hpp"
#include <cstring>
#include "ProfilerTask.hpp"

#include "stm32g4xx_hal.h"

// External Tasks (to send debug commands to)
#include "NAU7802Task.hpp"
#include "CANTask.hpp"


/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr uint8_t DEBUG_TASK_PERIOD = 100;
// extern I2C_HandleTypeDef hi2c2;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Constructor, sets all member variables
 */
DebugTask::DebugTask()
    : Task(TASK_DEBUG_QUEUE_DEPTH_OBJS), kUart_(UART::Debug)
{
  memset(debugBuffer, 0, sizeof(debugBuffer));
  debugMsgIdx = 0;
  isDebugMsgReady = false;
}

/**
 * @brief Init task for RTOS
 */
void DebugTask::InitTask()
{
  // Make sure the task is not already initialized
  SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Debug task twice");

  // Start the task
  BaseType_t rtValue = xTaskCreate(
      (TaskFunction_t)DebugTask::RunTask, (const char *)"DebugTask",
      (uint16_t)TASK_DEBUG_STACK_DEPTH_WORDS, (void *)this,
      (UBaseType_t)TASK_DEBUG_PRIORITY, (TaskHandle_t *)&rtTaskHandle);

  // Ensure creation succeded
  SOAR_ASSERT(rtValue == pdPASS, "DebugTask::InitTask - xTaskCreate() failed");
}

// TODO: Only run thread when appropriate GPIO pin pulled HIGH (or by define)
/**
 *    @brief Runcode for the DebugTask
 */
void DebugTask::Run(void *pvParams)
{
  // Arm the interrupt
  ReceiveData();

  while (1)
  {
    Command cm;

    // Wait forever for a command
    qEvtQueue->ReceiveWait(cm);

    // Process the command
    if (cm.GetCommand() == DATA_COMMAND &&
        cm.GetTaskCommand() == EVENT_DEBUG_RX_COMPLETE)
    {
      HandleDebugMessage((const char *)debugBuffer);
    }

    cm.Reset();
  }
}

/**
 * @brief Handles debug messages, assumes msg is null terminated
 * @param msg Message to read, must be null termianted
 */
void DebugTask::HandleDebugMessage(const char *msg)
{

  //-- NAU7802 LOAD CELL COMMANDS --
  if (strcmp(msg, "nau_read") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_READ));
  }
  else if (strcmp(msg, "nau_stat") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_STATUS));
  }
  else if (strcmp(msg, "nau_drdy") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_DRDY));
  }
  else if (strcmp(msg, "nau_on") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_ON));
  }
  else if (strcmp(msg, "nau_off") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_OFF));
  }
  else if (strcmp(msg, "nau_tog") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_TOGGLE));
  }
  else if (strcmp(msg, "nau_log") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_ENABLE_LOG));
  }
  else if (strcmp(msg, "nau_nolog") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_DISABLE_LOG));
  }
  else if (strcmp(msg, "nau_tare") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_TARE));
  }
  else if (strcmp(msg, "nau_c500") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_CAL_500G));
  }
  else if (strcmp(msg, "nau_c1000") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_CAL_1000G));
  }
  else if (strcmp(msg, "nau_g1") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_SET_GAIN_1X));
  }
  else if (strcmp(msg, "nau_g2") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_SET_GAIN_2X));
  }
  else if (strcmp(msg, "nau_g4") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_SET_GAIN_4X));
  }
  else if (strcmp(msg, "nau_g8") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_SET_GAIN_8X));
  }
  else if (strcmp(msg, "nau_g128") == 0)
  {
    NAU7802Task::Inst().SendCommand(Command(DATA_COMMAND, NAUTASK_COMMAND_NAU_SET_GAIN_128));
  }
  //-- RS485 / COMMS TEST COMMAND --
  else if (strcmp(msg, "rs485_tx") == 0)
  {
    bool ok = CANBusTask::Inst().SendLoadCellGrams(1234);
    SOAR_PRINT("RS485 test TX %s\n", ok ? "queued" : "FAILED");
  }
  //-- SYSTEM / CHAR COMMANDS -- (Must be last)
  else if (strcmp(msg, "sysreset") == 0)
  {
    // Reset the system
    SOAR_ASSERT(false, "System reset requested");
  }
  else if (strcmp(msg, "sysinfo") == 0)
  {
    // Print message
    SOAR_PRINT("\n\n-- CUBE SYSTEM --\n");
    SOAR_PRINT("Current System Free Heap: %d Bytes\n", xPortGetFreeHeapSize());
    SOAR_PRINT("Lowest Ever Free Heap: %d Bytes\n",
               xPortGetMinimumEverFreeHeapSize());
    SOAR_PRINT("Debug Task Runtime  \t: %d ms\n\n",
               TICKS_TO_MS(xTaskGetTickCount()));
  }
	#if (configGENERATE_RUN_TIME_STATS == 1)  // enable profiling commands if profiling enabled
	  else if (strcmp(msg, "profile") == 0) {
		profileSystem = true;
	  } else if (strcmp(msg, "stop profiling") == 0) {
		profileSystem = false;
	  }
	#endif
  else
  {
    // Single character command, or unknown command
    switch (msg[0])
    {
    case 'h':
      SOAR_PRINT("\n-- DEBUG COMMANDS --\n");
      SOAR_PRINT("sysinfo   - System information\n");
      SOAR_PRINT("sysreset  - System reset\n");
#if (configGENERATE_RUN_TIME_STATS == 1)
      SOAR_PRINT("profile   - Start CPU/stack profiler\n");
#endif
      SOAR_PRINT("rs485_tx  - Send test RS485 load-cell packet (1234g)\n");
      SOAR_PRINT("nau_read  - Single NAU7802 read\n");
      SOAR_PRINT("nau_stat  - NAU7802 status\n");
      SOAR_PRINT("nau_drdy  - NAU7802 DRDY check\n");
      SOAR_PRINT("nau_on    - Enable NAU7802 polling\n");
      SOAR_PRINT("nau_off   - Suspend NAU7802 polling\n");
      SOAR_PRINT("nau_tog   - Toggle NAU7802 polling\n");
      SOAR_PRINT("nau_log   - Enable continuous NAU7802 log + RS485 TX\n");
      SOAR_PRINT("nau_nolog - Disable continuous NAU7802 log\n");
      SOAR_PRINT("nau_tare  - Tare NAU7802\n");
      SOAR_PRINT("nau_c500  - Calibrate 500g point\n");
      SOAR_PRINT("nau_c1000 - Calibrate 1000g point\n");
      SOAR_PRINT("nau_g1    - Set gain 1x (also nau_g2/4/8/128)\n");
      SOAR_PRINT("h         - Show this help\n\n");
      break;
    default:
      SOAR_PRINT("Debug, unknown command: %s (type 'h' for help)\n", msg);
      break;
    }
  }

  // We've read the data, clear the buffer
  debugMsgIdx = 0;
  isDebugMsgReady = false;
}

/**
 * @brief Receive data, currently receives by arming interrupt
 */
bool DebugTask::ReceiveData() { return kUart_->ReceiveIT(&debugRxChar, this); }

/**
 * @brief Receive data to the buffer
 * @return Whether the debugBuffer is ready or not
 */
void DebugTask::InterruptRxData(uint8_t errors)
{
  // If we already have an unprocessed debug message, ignore this byte
  if (!isDebugMsgReady)
  {
    // Check byte for end of message - note if using termite you must turn on
    // append CR
    if (debugRxChar == '\r' || debugMsgIdx == DEBUG_RX_BUFFER_SZ_BYTES)
    {
      // Null terminate and process
      debugBuffer[debugMsgIdx++] = '\0';
      isDebugMsgReady = true;

      // Notify the debug task
      Command cm(DATA_COMMAND, EVENT_DEBUG_RX_COMPLETE);
      bool res = qEvtQueue->SendFromISR(cm);

      // If we failed to send the event, we should reset the buffer, that way
      // DebugTask doesn't stall
      if (res == false)
      {
        debugMsgIdx = 0;
        isDebugMsgReady = false;
      }
    }
    else
    {
      debugBuffer[debugMsgIdx++] = debugRxChar;
    }
  }

  // Re-arm the interrupt
  ReceiveData();
}

/* Helper Functions
 * --------------------------------------------------------------*/
/**
 * @brief Extracts an integer parameter from a string
 * @brief msg Message to extract from, MUST be at least identifierLen long, and
 * properly null terminated
 * @brief identifierLen Length of the identifier eg. 'rsc ' (Including the
 * space) is 4
 * @return ERRVAL on failure, otherwise the extracted value
 */
int32_t DebugTask::ExtractIntParameter(const char *msg,
                                       uint16_t identifierLen)
{
  // Handle a command with an int parameter at the end
  if (static_cast<uint16_t>(strlen(msg)) < identifierLen + 1)
  {
    SOAR_PRINT("Int parameter command insufficient length\r\n");
    return ERRVAL;
  }

  // Extract the value and attempt conversion to integer
  const int32_t val = Utils::StringToLong(&msg[identifierLen]);
  if (val == ERRVAL)
  {
    SOAR_PRINT("Int parameter command invalid value\r\n");
  }

  return val;
}
