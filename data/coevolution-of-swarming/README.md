All of the data and the IPython Notebook we used to generate figures for for our paper, "Predator confusion is sufficient to evolve swarming behavior" are contained in the included `coevolution-of-swarming` directory.

------------------------------------------------------------------------------------------

If you have questions, contact us at:

Randal S. Olson (rso@randalolson.com)

Arend Hintze (hintze@msu.edu)

------------------------------------------------------------------------------------------

RUNNING IPYTHON NOTEBOOK

To run our IPython Notebook, you must first download the required packages:

* Python
* IPython
* IPython Notebook
* glob
* NumPy
* SciPy
* pandas

We recommend downloading the Enthought Python distribution, which has all of the packages you will need. There is a free version for people with an active .edu email here:

http://enthought.com/products/edudownload.php

or a smaller, free version for those without an active .edu email here:

http://enthought.com/products/epd_free.php

Once you have installed the required packages, open your terminal and navigate to the eos-predator-confusion directory. Once there, enter the command:

ipython notebook --pylab=inline

This will open up the IPython Notebook terminal. Select the notebook labeled "Predator Confusion Final Graphs." This notebook is entirely interactive, so feel free to explore the data yourself.

------------------------------------------------------------------------------------------

DIRECTORY LABELS

Each experiment has its own directory. The directory names are as follows:

* attack-efficiency-over-swarm-size: experiments measuring the predator's attack efficiency when attacking prey swarms of a given size

* functional-response-over-kill-delay: experiments measuring the predator functional response for varying predator handling times

* swarm-sd30: basic experiment with predator confusion seeded with pre-evolved predators

* swarm-sd30-nopredconfusion: basic experiment without predator confusion seeded with pre-evolved predators

* swarm-sd30-confusion-multiplierX: basic experiment with predator confusion, where X is the multiplier for the predator confusion effect mechanism (X / # prey)

* swarm-sd30-confusion-multiplierX-min-attack-efficiencyY: basic experiment with predator confusion, where X is the multiplier for the predator confusion effect mechanism (X / # prey) and Y is the minimum value that the predator attack efficiency can be

* swarm-sd30-randpred & swarm-sd30-nopredconfusion-randpred: basic experiment with and without predator confusion with randomly-seeded predators

* swarm-sd30-pvaX: predator confusion experiment with a different predator view angle, where X is the view angle. e.g., swarm-sd30-pva150 has a predator view angle of 150 deg.

* swarm-sd30-killdelayX: predator confusion experiment with a different kill delay, where X is the kill delay. (default kill delay is 10 in swarm-sd30.) e.g., swarm-sd30-killdelay5
has a predator kill delay of 5 simulation time steps.

* swarm-sd30-swarmsizeX: functional response experiments with swarming agents, where X is the swarm size. e.g., swarm-sd30-swarmsize20 has a swarm size of 20.

* swarm-sd30-swarmsizeX-gen1: functional response experiments with non-swarming agents, where X is the swarm size. e.g., swarm-sd30-swarmsize20-gen1 has a swarm size of 20.

------------------------------------------------------------------------------------------

CSV FILE FORMAT

Each csv file is one replicate for that experiment. The number following the word "run" in the csv file name is the random number generator seed for that replicate. e.g., run24LOD.csv has a random number generator seed of 24.

In the LOD files, there will be a single entry for each ancestor in the final best swarm agent's LOD. LOD files will be in csv format with the column headers listed at the top. Column headers are in the following order:

* generation: the generation the ancestor was born

* prey_fitness: the fitness of the ancestor prey

* predator_fitness: the fitness of the ancestor predator

* num_alive_end: the number of surviving prey at the end of the fitness evaluation

* avg_bb_size: the average bounding box size of the swarm during the simulation

* var_bb_size: the variance in the bounding box size of the swarm during the simulation

* avg_shortest_dist: the average distance from every prey agent to the prey agent closest to it

* swarm_density_count: the average number of prey agents within safety distance units of each other

* prey_neurons_connected_prey_retina: the number of conspecific sensor neurons that the prey Markov network brain is connected to

* prey_neurons_connected_predator_retina: the number of predator sensor neurons that the prey Markov network brain is connected to

* predator_neurons_connected_prey_retina: the number of prey sensor neurons that the predator Markov network brain is connected to

* mutual_info: the average amount of mutual information between all of the prey's navigation angles and the predator's navigation angle

* num_attacks: the number of attacks the predator made on prey during the simulation

------------------------------------------------------------------------------------------

OTHER FILES

Some directories, such as swarm-sd30, also have .genome and .dot files.

Generally, we save Markov network brain files as .genome files. These files contain integer values which encode the Markov network brain.

DOT files are the picture representations of Markov network brain structure and connectivity. We recommend using the Graphviz software to view these images.

------------------------------------------------------------------------------------------