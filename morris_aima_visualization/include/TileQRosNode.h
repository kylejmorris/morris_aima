/**
 * class: TileQRosNode
 * author: Kyle Morris
 * date: July 30, 2016
 * Description: The Ros component for a TileVisualizer, will send info to GUI
 *
 * This grid is fixed size, meaning it's width/height is specified once and then simulation may be done. Must be reset() to
 * load in a new size.
 */
#ifndef MORRIS_AIMA_VISUALIZATION_TILEQROSNODE_H
#define MORRIS_AIMA_VISUALIZATION_TILEQROSNODE_H


#include <QtCore/QThread>
#include <morris_aima_msgs/TileEnvironmentInfo.h>
#include <ros/service_server.h>

class TileQRosNode : public QThread {
    Q_OBJECT
private:
    /**
     * Storing values when initialized
     */
    int argc = 0;
    char **argv = NULL;

    /**
     * When the node was set and initialized
     */
    ros::Time start_time;

    ros::ServiceServer run_service;
public:
    TileQRosNode(int argc, char **argv);

    /**
     * Start the ros main loop on a separate thread.
     */
    void run();

    /**
     * Set up ros services/publications and other hooks.
     * @returns true if initialization was success, false otherwise
     */
    bool initialize();

    //All signals are explained in the TileMainWindow as slots where they are acted on
public: Q_SIGNALS:
        void rosShutdown();
        void update(morris_aima_msgs::TileEnvironmentInfo &msg);
        void enableUpdating();
        void freeze();
        void setParameters(int width, int height, std::string name);
        void reset();

public:
    //Callbacks for ros services/subscriptions, corresponding signals will then be sent.
        void update_callback(morris_aima_msgs::TileEnvironmentInfo &msg);
        void enableUpdating_callback();
        void setParameters_callback(int width, int height, std::string name);
        void reset_callback();
};


#endif //MORRIS_AIMA_VISUALIZATION_TILEQROSNODE_H
