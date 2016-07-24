/**
 * Class: Visualizer
 * Author: Kyle Morris (March 25, 2016)
 *
 * This is an interface outlining the Visualizer module of this project. The Visualizer is responsible
 * for taking in state information and rendering it for the user/world to see.
 * A visualizer may render an environment state using ascii art, gui with frames/color, and so forth.
 * For state encoding as input to this visualizer, JSON is a good selection as it allows easy communication of data without passing around full EnvironmentState objects.
 * Also this serves to allow future development from the webbrowser, where Environments may manifest themselves in various
 * ways so long as they are convertible to a JSON state.
 *
 */
#ifndef MORRIS_AIMA_VISUALIZER_H
#define MORRIS_AIMA_VISUALIZER_H
#include <string>
#include "ros/ros.h"

class Visualizer {
    /**
     * The handle that this visualizer is referring to.
     */
    ros::NodeHandle *handle;
public:
    /**
     * Render the given environment using whatever information was provided when it was loaded.
     */
    virtual void render() = 0;

    ros::NodeHandle *getNodeHandle();
    void setNodeHandle(ros::NodeHandle *handle);

    /**
     * Set up the ros services/nodes used from here.
     */
    virtual void initialize() = 0;
};

#endif //MORRIS_AIMA_VISUALIZER_H
