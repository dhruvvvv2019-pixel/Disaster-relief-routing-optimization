# Disaster-relief-routing-optimization
This repository implements an AI-based solution to the Disaster Relief Helicopter Routing Problem, formulated and solved as a search and optimization problem.

The objective is to efficiently allocate a fleet of helicopters to deliver relief packages to flood-affected villages while:

1.Maximizing the total number of people aided

2.Respecting helicopter capacity and operational range constraints

3.Minimizing logistical strain caused by inefficient routing and allocation

The problem is modeled as a state-space search, where each state represents a partial or complete allocation of relief deliveries across helicopters. The solution applies heuristic-guided search techniques to explore feasible allocations and converge toward an optimal or near-optimal solution under given constraints.

# Key Features

1.Formal formulation of a real-world disaster response scenario as a search problem

2.Constraint handling for:

  Helicopter load capacity

  Maximum flight range

  Limited relief package availability

  Optimization of a composite objective function balancing:

3.Aid maximization

4.Logistical efficiency

5.Modular C++ implementation suitable for extension and experimentation
