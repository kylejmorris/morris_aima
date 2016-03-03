/**
 * Class: EnvironmentState
 * Author: Kyle Morris (March 2 2016)
 *
 * Current state frame for a given environment.
 * For an environment named X the environment state should be called XState.
 * This appears quite similar to the Environment class itself, in terms of methods; however
 * an environment state only contains logic directly pertaining to the environments contents, and
 * doesn't involve any controller-like behaviour, such as generating percepts/interpreting actions.
 * It merely receives commands from the Environment and has basic operations the environment may leverage on
 * to create more advanced functionality.
 * In short, picture the EnvironmentState as an ADT defining all manipulations that may take place
 * in an environment, and the Environment itself is all the logic behind how these manipulations
 * can be used.
 */
#ifndef MORRIS_AIMA_ENVIRONMENTSTATE_H
#define MORRIS_AIMA_ENVIRONMENTSTATE_H

class EnvironmentState {

};


#endif //MORRIS_AIMA_ENVIRONMENTSTATE_H
