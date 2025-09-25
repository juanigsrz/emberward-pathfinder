# emberward-pathfinder
A collection of algorithms to solve the network interdiction problem in a 2d square grid, inspired by tower defense games where the player has to build an optimal maze to defend from enemies.

### MILP
**Mixed-integer linear programming:** formulates the grid as a linear model and finds the optimal answer with a solver (Gurobi). Takes a very long time on +medium grids.

### Genetic
**Genetic algorithm:** more of an experiment than not, but it may be a good "greedy" approach as it has some resemblance to what a human player does playing the game.

### Annealing
**Simulated annealing:** finds decent solutions quickly depending on the parameters, seems too random to find the *good* ones.
