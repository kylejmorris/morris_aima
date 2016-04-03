# morris_aima

##File Structure
	config: 	Configurations and map layout templates. Go here to see how to customize map files.
	core: 		Abstract classes & descriptions of project. These are untouchable definitions/outlines of core module functionality. 
	include: 	additional includes for project, stray or not specific enough for a separate module.
	src: 		additional source for project, works with /include directory.
	testing: 	gtest module containing all unit test for proejct
	visualizer: module for handling display/gui 


##Functionality/Flow description from top-down
![http://i.imgur.com/SDDS5mA.png]({{site.baseurl}}/Selection_927.png)
The timer will run the simulation through a cycle, which involves running a cycle in the environment (one logical iteration), then feeding the new environment state into the visualizer and displaying it.
##Principle of Substitution at work
