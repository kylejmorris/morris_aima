/**
 * Class: Environment
 * Author: Kyle Morris
 *
 * Abstract environment that acts as interface for all kinds of environments.
 * In general an environment will contains entities and a "moment" which
 * is essentially one cycle of the environment.
 */
#ifndef MORRIS_AIMA_ENVIRONMENT_H
#define MORRIS_AIMA_ENVIRONMENT_H
#include <string>

using namespace std;

class Environment {
public:
    /**
     * Initialize the environment from a map file, setting initial environment state.
     * @param string fileName: Name of the environment to load.
     */
    virtual void loadEnvironment(string fileName) = 0;
};


#endif //MORRIS_AIMA_ENVIRONMENT_H
