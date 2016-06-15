#include "TerminalTextVisualizer.h"
#include <iostream>
#include <QTextStream>
#include <json/json.h>

void TerminalTextVisualizer::update(std::string state) {
    Json::Value root;
    Json::Reader reader;
    bool success = reader.parse(state, root, false);
    std::ostringstream out;
    if (success) {
        bool found = root["Environment"]["found"].asBool();
        if (found) {
            out << "PATH HAS BEEN FOUND:\n";
            for (auto node : root["Environment"]["solution_path"]) {
                if (node["id"].asInt() == 0) {
                    out << " GOAL STATE: ";
                } else {
                    out << "distance to goal: " << node["id"];
                }
                out << "\n--->";
                out << " state: ";
                out << "{LEFT=(" << node["state"]["cannibals_left"] << " cannibals";
                out << ", " << node["state"]["missionaries_left"] << " missionaries";
                out << ")|RIGHT=(" << node["state"]["cannibals_right"] << " cannibals";
                out << ", " << node["state"]["missionaries_right"] << " missionaries)";
                if (node["state"]["river_crossed"].asBool() == true) {
                    out << "river=right side";
                } else {
                    out << "river=left side";
                }
                out << "}\n";
            }
        } else {
            //outputing frontier
            out << "#######FRONTIER#######\n";
            for (auto node : root["Environment"]["frontier"]) {
                out << "{LEFT=(" << node["cannibals_left"] << " cannibals";
                out << ", " << node["missionaries_left"] << " missionaries";
                out << ")|RIGHT=(" << node["cannibals_right"] << " cannibals";
                out << ", " << node["missionaries_right"] << " missionaries)";
                if (node["river_crossed"].asBool() == true) {
                    out << "river=right side";
                } else {
                    out << "river=left side";
                }
                out << "}\n";
            }
            out << "##########EXPLORED SET ###########\n";
            for (auto node : root["Environment"]["explored"]) {
                out << "{LEFT=(" << node["cannibals_left"] << " cannibals";
                out << ", " << node["missionaries_left"] << " missionaries";
                out << ")|RIGHT=(" << node["cannibals_right"] << " cannibals";
                out << ", " << node["missionaries_right"] << " missionaries)";
                if (node["river_crossed"].asBool() == true) {
                    out << "river=right side";
                } else {
                    out << "river=left side";
                }
                out << "}\n";
            }
        }
    } else {
        out << "ERROR IN UPDATING VISUALIZE\n";
        output = "ERROR IN UPDATING VISUALIZER\n";
    }
    output = out.str();
}

void TerminalTextVisualizer::render() {
    std::cout << output << std::endl;
}
