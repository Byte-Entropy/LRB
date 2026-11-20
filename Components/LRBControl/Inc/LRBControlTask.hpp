/*
 * LRBControlTask.hpp
 *
 *  Created on: May 20, 2026
 */

#ifndef LRBCONTROLTASK_HPP_
#define LRBCONTROLTASK_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "Task.hpp"
#include "SystemDefines.hpp"

/************************************
 * MACROS AND DEFINES
 ************************************/
constexpr uint16_t LRB_CONTROL_TASK_RTOS_PRIORITY = 2;
constexpr uint16_t LRB_CONTROL_TASK_QUEUE_DEPTH_OBJS = 10;
constexpr uint16_t LRB_CONTROL_TASK_STACK_DEPTH_WORDS = 512;
constexpr uint16_t LRB_FLIGHT_TASK_BOOTUP_TELE_CYCLES = 20;

/************************************
 * TYPEDEFS
 ************************************/
 enum FlightTaskRequests
 {
 	FT_REQUEST_NONE = 0,
 	FT_REQUEST_TRANSMIT_STATE,	// Send the current state over the UART
 };

/************************************
 * CLASS DEFINITIONS
 ************************************/
class LRBControlTask : public Task
{
public:
    static LRBControlTask& Inst() {
        static LRBControlTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { LRBControlTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

    void SendRocketState();

private:
    // Private Functions
    LRBControlTask();        // Private constructor
    LRBControlTask(const LRBControlTask&);                        // Prevent copy-construction
    LRBControlTask& operator=(const LRBControlTask&);            // Prevent assignment

    // Private Variables
    uint16_t firstStateSent_;
};

#endif /* LRBCONTROLTASK_HPP_ */
