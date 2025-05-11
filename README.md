# Multiplayer Robot Soccer Simulation

## Overview

**Multiplayer Robot Soccer Simulation** is a senior design project that explores real-time autonomous robot coordination in a simulated soccer environment using Webots. This project applies the WYSIWIS (What You See Is What I See) paradigm to maintain a shared and synchronized game state across all robot agents.

Each robot is fully autonomous and uses built-in AI to make real-time decisions based on the game state. A centralized server (the Webots Supervisor) is responsible for maintaining global state consistency, assigning roles, and managing inter-robot communication.

## Project Goals

- Role Management & Coordination
  Dynamically assign and manage robot roles such as Striker, Midfielder, and Goalie, ensuring balanced team behavior and specialization.

- Autonomous AI & Decision-Making
  Implement intelligent behavior for each role using finite state machines (FSMs) that respond to real-time game conditions.

- Real-Time State Synchronization
  Maintain a synchronized game state across all autonomous robot clients using reliable TCP networking.

- Networked Simulation Architecture
  Explore and apply multiplayer networking models and distributed simulation techniques to improve timing accuracy and communication robustness.

- Artificial Force Injection & Adaptation
  Simulate external disturbances such as wind, terrain shifts, or network latency to test the adaptability and resilience of robot behaviors.

- WYSIWIS Environment Consistency
  Ensure What You See Is What I See consistency so that all agents operate with a shared and unified view of the game world.

## Technologies Use

- Webots – 3D robotics simulation platform used to create the virtual soccer field and robot environment.

- Python – Programming language used to implement robot AI, game logic, and server communication.

- TCP Sockets – Used for real-time communication between the centralized supervisor and each autonomous robot, ensuring synchronized game state across clients.

## Getting Started

1. **Install Webots**  
   Download and install Webots from [https://cyberbotics.com](https://cyberbotics.com).

2. **Install Python**  
   Make sure Python 3.x is installed on your system.  
   In Webots, go to `Preferences → Python command` and set it to the full path to your Python executable.

3. **Clone the Repository**

   ```bash
   git clone https://github.com/KevinZheng0701/MultiplayerSoccerGame.git

   ```

4. **Open the Simulation in Webots**  
   Launch Webots and open the `.wbt` file in the `worlds/` directory.

5. **Run the Simulation**  
   Press the **Play** button in Webots to start the simulation.
