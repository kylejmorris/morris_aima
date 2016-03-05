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
    /**
     * Determine if the current state is valid or not. This is really just the invariant
     * of the Environment. Useful for testing purposes and state searching.
     * @return bool: True if state is valid, false otherwise
     */
    virtual bool isValid() = 0;

    /**
     * Compare this state to another Environment state.
     * @return int: If return < 0 then this state is logically "less" than the other state.
     *  What you deem as logically "less" may be determined by how beneficial the state is/valuable, or
     *  which state should come logically first. Completely up to you.
     *  Return > 0 means this state is logically great than the other state.
     *  Return = 0 means both of these states are logically equivalent.
     */
    virtual int compareTo(EnvironmentState *other) = 0;
};


#endif //MORRIS_AIMA_ENVIRONMENTSTATE_H
