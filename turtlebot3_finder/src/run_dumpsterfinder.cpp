#include "DumpsterFinder.h"

int main(int argc, char **argv) {
    // Initiate new ROS node named "DumpsterFinder"
    ros::init(argc, argv, "DumpsterFinder");

    // Create new walk and look for a dumpster
    DumpsterFinder dumpsterFinder;

    // Start
    dumpsterFinder.start();


    return 0;
};



