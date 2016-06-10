/**
* CLASS: TerminalTextVisualizer
* DATE: 07/06/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Displays text in a terminal, no frames or colours/etc going on. Just essentially
* a standard output window
*/

#ifndef MORRIS_AIMA_TERMINALTEXTVISUALIZER_H
#define MORRIS_AIMA_TERMINALTEXTVISUALIZER_H
#include "Visualizer.h"

class TerminalTextVisualizer : public Visualizer {
    /**
     * The output generated when we update.
     */
    std::string output;
public:
    virtual void update(std::string state) override;

    virtual void render() override;
};

#endif //MORRIS_AIMA_TERMINALTEXTVISUALIZER_H
