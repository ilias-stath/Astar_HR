# Automated-EV-Charging
## Overview
This project was created for the Eurobot 2025 contest and it is an implementation of the Astar pathfinding algorithm. In the context of the contest it was used to help the robot navigate the playable area when it encountered
the enemy player and the process of the simple pathfinding failed. Due to it being slower than we wanted, about 500ms -> 1.5s to find optimal path, it couldn't be the main pathfinding tool, but we hope to improve it in the future. <br>

## Advantages
This implementation has some advantages that give it diversabillity and flexibility.
### 1. Automatic point generation
This implementation features an automatic point generation method. The user chosses only the lenght and width of the grid and the algorithm uses them to create an imaginery grid of the area. This reduces by a lot the memory needed because you don't physically store the grid.
### 2. Step mechanism
This feature is really unique because it enables the user to dynamically choose the distance between the points created. This gives him the opportunity to explore the imaginery grid with as much precision as he wants. So the user for example can create a grid of 10x10 meters and search every 1cm of it or create a grid of 100x300 mm and search every 1mm of it. It is important to note


## Getting started
1. 

2. 

## Running the example
There are 2 examples provided here, one that you can run in a .<br>

1. Launch the simulated enviroment
