# Autonomous Navigation with Artificial Potential Fields

This project implements autonomous navigation strategies for holonomic (integrator) and non-holonomic (unicycle) robots in a two-dimensional environment using **Artificial Potential Fields (APF)**. The goal is to guide the robot towards a predefined target position while avoiding obstacles through a combination of attractive and repulsive forces.

## Authors
- Luke Keating
- Iñaki Román

## Date
25 November 2024

## Table of Contents
- [Introduction](#introduction)
- [Methodology](#methodology)
  - [Artificial Potential Fields](#artificial-potential-fields)
  - [Robot Models](#robot-models)
- [Simulations](#simulations)
  - [Single Integrator Robot](#single-integrator-robot)
  - [Non-Holonomic (Unicycle) Robot](#non-holonomic-unicycle-robot)
- [Scenarios](#scenarios)
  - [Point Obstacles](#point-obstacles)
  - [Circular Obstacles](#circular-obstacles)
- [Conclusions](#conclusions)
- [Requirements](#requirements)
- [How to Run](#how-to-run)

## Introduction

Obstacle avoidance is crucial for autonomous vehicles and robots. Among various navigation methods, **Artificial Potential Fields (APF)** are known for their simplicity and effectiveness in guiding robots toward a goal while avoiding obstacles. This project explores APF-based navigation for two robot types:

- **Holonomic robots** (can move freely in any direction)
- **Non-holonomic robots** (motion constrained, like a unicycle model)

## Methodology

### Artificial Potential Fields

The navigation strategy combines:
- **Attractive potentials**: Pull the robot toward the goal.
- **Repulsive potentials**: Push the robot away from obstacles.

The total potential field is defined as:

U(p) = α * Uatt(p, p_goal) + β * Urep(p, p_obs)

Where:
- `Uatt`: Attractive potential
- `Urep`: Repulsive potential
- `α` and `β`: Scalars adjusting the influence of each field

The control law for robot navigation uses the negative gradient of the total potential field:
u = -∇U(p)
### Robot Models

1. **Single Integrator Robot**: Directly controls velocity in x and y directions.
2. **Non-Holonomic Unicycle Robot**: Controls linear and angular velocity with constraints.

## Simulations

Simulations are performed in **MATLAB**, visualizing robot trajectories and vector fields.

### Single Integrator Robot
- Starts at `[2, 1]`
- Avoids a point obstacle at `[4, 4]`
- Reaches the goal at `[6, 6]`

### Non-Holonomic (Unicycle) Robot
- Starts at `[2, 1]` with velocity `1` and heading `π/4`
- Avoids multiple obstacles while respecting motion constraints
- Follows references for velocity and heading derived from APF gradients

## Scenarios

### Point Obstacles

Navigation with single or multiple point obstacles to demonstrate basic obstacle avoidance.

### Circular Obstacles

Scenarios include:
- Single circular obstacle
- Multiple circular obstacles

The repulsive potential adapts to obstacle shapes using the distance to the nearest collision point.
