#include "controllers/dwmpc/topics.hpp"

// Include the TypeSupport of each message associated to each topic
// #include <dls_messages/dds/<idl_file_name>PubSubTypes.h> // # off-the-shelf message
#include "dls_messages/dds/control_signalPubSubTypes.h" // # off-the-shelf message
// #include "dls_messages/dds/<idl_file_name>PubSubTypes.h" // # custom message

namespace dls
{
    namespace topics
    {
        namespace dwmpc{
            // dls::topicType topic_variable_name = dls::topicType("topic_name", new <message_name>PubSubType());
            dls::topicType tau = dls::topicType("dwmpc", new ControlSignalMsgPubSubType());
        }
    }
}
