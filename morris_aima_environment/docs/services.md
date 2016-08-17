All Environments share some common services you may access. They are outlined below.
Any environment you wish to add, must support these as well.

morris_aima_environment/start
- Begin running the environment cycles, and doing state transitions.

morris_aima_environment/stop
- Stop running environment cycles. This will pause the environment in it's current state. 

morris_aima_environment/reset
- Put the environment back into initial state before anything was loaded. This can't happen unless Environment is not active.

morris_aima_environment/load
- Load updated configuration parameters into the environment. Environment must not be active. It will be reset when the new parameters are loaded.
