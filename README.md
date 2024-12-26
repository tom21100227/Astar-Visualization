# A-star Visualization for Robot Path Planning

This project performs A* search on a robot that have 3 degrees of freedom (x,y, theta). And navigated the robot from start point to goal point across user defined polyogn obstacles. Originally developed for CSCI 3250 - Computational Geometry at Bowdoin College. All class related code except for OpenGL are removed.

[Short Demo](https://github.com/user-attachments/assets/fe274a32-4bda-45ec-b54b-6dff36ed5e9c)

[Full Walkthrough](https://github.com/user-attachments/assets/07eb456c-5bbc-44e2-9579-c7c150e6beaf)

## Getting Started

### For Regular

Compile the code by running the following command in the terminal:

```sh
$ make
```

Run the code
```sh
$ ./motion
```

### With Audio.

First, ensure that you installed the required libraries (SFML). If not, run the following command in the terminal:

```sh
$ brew install sfml
```

Compile the code by running the following command in the terminal:

```sh
$ make audio
```

Run the program
```sh
./motion_with_audio
```

## Use Instruction

Run program with `./motion` or `./motion_with_audio`. You might need to configure the include path if you didn't use homebrew to install SFML.

### Draw Obstacles
- Press `s` to start drawing obstacles.
- Press `n` to start drawing the next obstacle.
- Press `z` to undo last polygon.
- Press `e` when you're done drawing the obstacle.

### Draw Robot (and start point).
- Once you pressed `e` to finish drawing the obstacle, the next point you click will be the robot's starting point, and the first vertex of the robot.
- Click the next point to draw the second vertex of the robot.
- Press `e` to finish drawing the robot.

### Draw Goal Point
- Click the goal point.

### Run A* Search
- Press `v` to run A* search.
- Press `t` to switch between the three visualization modes (No visualization, states as points, or states as robots).

Note that visualization is EXTREMELY slow. Even though path planning is in a separate thread, the visualization would froze path planning (mutex).
For reference:
![Visualization](./img/Screenshot%202024-12-19%20at%2022.36.33.png)

### Replay
- Once path planning is done, you can press `r` to replay the path.
