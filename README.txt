## How to Build

    cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
    cmake --build build

## How to Run

	build/BoidSimulation

Welcome to my assignment! The above steps should build the project. 

I have implemented the first two bonuses:
1. Press UP and DOWN keys to increase or decrease the count of boids.
2. Use the SPACE key to enable follow the leader. Use the LEFT and RIGHT keys to move the leader.

The parameters of the boids can be tweeked by changing the numbers in 'options.ini' in the main directory.
the filie is layed out as such:
Avoid Scale
Align Scale
Cohesion Scale
Obstacle Avoid Scale
Follow the Leader Scale