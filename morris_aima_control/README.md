#Morris AiMA Control Unit
This package provides controller functionality between the Environment(Model) and Visualizer(view).

Nodes:
    morris_aima_results: This node handles tracking the results of running a morris_aima project, intercepting messages and analyzing the data over time.
        Each environment may have it's own results. The Controller node may access information from a results node, to send statistics/other relevant data to the visualizer, instead of calculating the info itself or making the Environment do so. 

        Services
            morris_aima_results/start: Start intercepting messages from Environment and doing calculations.
            morris_aima_results/summarize: Do final calculations, at the end of an environments life. Some results may only need to do calculations at the end of runtime, or need to put together all the info collected so far, where as other results may only update on each environment cycle, and not need to summarize.
            morris_aima_results/save: Save results to a text file.

