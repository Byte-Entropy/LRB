/*
 * UARTProtoTask.hpp
 *
 *  Created on: May 20, 2026
 *      Author: Javier
 */

#ifndef LRBPROTOTASK_HPP_
#define LRBPROTOTASK_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/
class LRBProtocolTask : public ProtocolTask
{
public:
    static LRBProtocolTask& Inst() {
        static LRBProtocolTask inst;
        return inst;
    }

    void InitTask();

    static void SendProtobufMessage(EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>& writeBuffer, Proto::MessageID msgId)
    {
        Inst().ProtocolTask::SendProtobufMessage(writeBuffer, msgId);
    }

protected:
    static void RunTask(void* pvParams) { LRBProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);

    // Member variables

private:
    LRBProtocolTask();        // Private constructor
    LRBProtocolTask(const LRBProtocolTask&);                        // Prevent copy-construction
    LRBProtocolTask& operator=(const LRBProtocolTask&);            // Prevent assignment
};

/************************************
 * FUNCTION DECLARATIONS
 ************************************/


#endif /* LRBPROTOTASK_HPP_ */
