Découpé en 2 parties :
/Graph building
/Graph matching

//installer requirements
pip install -r requirements.txt


""Example de compilation :
 g++ -o graph_building/output/test -Igraph_building/include/GMLAAS graph_building/src/Node.cpp graph_building/src/Edge.cpp graph_building/src/Graph.cpp graph_building/example/main.cpp
 
 puis :
 ./graph_building//output/test

avec debbuger gdb
 g++ -g -o graph_building/output/test -Igraph_building/include/GMLAAS graph_building/src/Node.cpp graph_building/src/Edge.cpp graph_building/src/Graph.cpp graph_building/example/main.cpp
 puis :
 gdb ./graph_building//output/test
 ""

datasets (folder)
----other_datasets (files)
----snapshots (folder)
--------reference_graph_dataset
--------reference_tags_dataset
--------desired_frame_graph_dataset
--------desired_frame_tags_dataset