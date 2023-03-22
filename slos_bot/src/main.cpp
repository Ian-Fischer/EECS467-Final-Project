#include <iostream>
#include "slosbot.h"

void SLOSBot::search_for_object() {
    // get image

    // hsv segmentation

    // tell the mbot to rotate

}

void SLOSBot::execute_sm() {
    switch(state) {
        case State::SEARCH_FOR_OBJECT:
            search_for_object();
            break;
        default:
            std::cout << "invalid state" << std::endl;
    };

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "slosbot");
    std::cout << "hi " << std::endl;
}