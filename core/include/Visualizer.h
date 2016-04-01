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

class Visualizer {
public:
    //TODO update load/visualizer support after changing simulation design
    /**
     * Given a string representing the state of Robot/environment/etc, initialize the Visualizer accordingly
     * so we can render.
     * It is important to note just what kind of state you're reading in however. By default you could support JSON
     * but if you're looking to implement support for say, XML, then create other routines accordingly.
     * @return bool: true if loading was successful, false otherwise.
     */
    virtual void update(std::string state) = 0;

    /**
     * Render the given environment using whatever information was provided when it was loaded.
     */
    virtual void render() = 0;

};

#endif //MORRIS_AIMA_VISUALIZER_H
