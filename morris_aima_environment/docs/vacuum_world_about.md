The VacuumWorld supports chapter 1 & 2 of the AIMA 3rd Book. Involving simple agents.

Agents (supported in environment):
    SimpleReflexVacuumAgent
        -May only be added to locations (0,0) or (1,0). Simply moves left/right depending on if dirt is on tile. Assumes environment is 2x1, but it may be much larger.
    RandomReflexVacuumAgent
        -Randomly selects an adjacenet tile to move to either left, right, up, down. Cleans dirt if tile is dirty.

Entities (supported in environment):
    Dirt (TODO: add link)
    Wall (TODO: add link)

Properties of Task Environment
 Since chapter 2 of AIMA is mainly about this,  the Environment characteristics are outlined below.
   SINGLE AGENT: Only 1 vacuum exists
   DETERMINISTIC: The outcome of an agents actions are exactly as determined, no probabilistic events.
   UNCERTAIN: Since environment is not fully observable, it's deemed uncertain.
   EPISODIC: The agents experience is divided into atomic episodes. Each episode the agent receives a percept, and performs
   an action. The next episode does not depend on the actions taken in previous episodes.
   STATIC: The environment does not change while an Agent is thinking.
   DISCRETE: Finite number of possible states in environment. Not continuous real number values.

ROS Specification
-PUBLISHED TOPICS: inherits publications from TileEnvironment[LINK]
