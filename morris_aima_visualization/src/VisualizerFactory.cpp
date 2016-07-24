#include <Visualizer.h>
#include <XmlRpcValue.h>
#include <VisualizerFactory.h>
#include <TileFrameVisualizer.h>

Visualizer *VisualizerFactory::createVisualizer(std::string name, XmlRpc::XmlRpcValue &properties) {
    Visualizer *result = NULL;

    if(name.compare("vacuum_world")==0) {
        if((int)(properties["grid_width"])>=1 && (int)(properties["grid_height"])>=1) {
            result = new TileFrameVisualizer;
        } else {
            ROS_INFO("Grid is not valid size.");
        }
    }

    return result;
}

