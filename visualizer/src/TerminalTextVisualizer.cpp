#include "TerminalTextVisualizer.h"
#include <iostream>
#include <QTextStream>

void TerminalTextVisualizer::update(std::string state) {
    std::cout << state << std::endl;
}

void TerminalTextVisualizer::render() {
}
