#include "ros/node_handle.h"
#include "ros/ros.h"

class SLOSBot 
{
    public:
        SLOSBot() = default;
    private:
        enum class State {
            SEARCH_FOR_OBJECT,
            MATCH_OBJECT
        };
        State state = State::SEARCH_FOR_OBJECT;

        ros::NodeHandle nh;

        void search_for_object();

        void execute_sm();
};