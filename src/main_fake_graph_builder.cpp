#include <fake_graph_builder.hpp>
#include <ros/master.h>
#include <unistd.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "main_fake_graph_builder");
    ros::NodeHandle fake_graph_builder_node;
    FakeGraphBuilder fake_graph_builder(fake_graph_builder_node);
    fake_graph_builder.initFakeGraph();

    while (true){
        fake_graph_builder.processFakeGraph();
    }
    ros::spin();

    return 0;

}