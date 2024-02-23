#include <fake_graph_builder.hpp>
#include <ros/master.h>
#include <unistd.h>
#include <signal.h> 

// Global flag to indicate if SIGINT (Ctrl+C) was received
volatile sig_atomic_t g_shutdown_requested = 0;

// Signal handler function for SIGINT
void sigintHandler(int sig)
{
    // Set the flag indicating shutdown is requested
    g_shutdown_requested = 1;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "main_fake_graph_builder");
    ros::NodeHandle fake_graph_builder_node;
    FakeGraphBuilder fake_graph_builder(fake_graph_builder_node);
    fake_graph_builder.initFakeGraph();

    // Register the signal handler for SIGINT
    signal(SIGINT, sigintHandler);

    while (!g_shutdown_requested){
        fake_graph_builder.processFakeGraph();

        ros::spinOnce(); // Handle ROS callbacks
        sleep(5);        // Sleep for 5 seconds
    }

    ROS_INFO("[INFO] Shutting down the fake graph builder node.");

    return 0;

}

