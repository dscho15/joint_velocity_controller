# joint_velocity_controller

## The purpose

The Franka Emika Pnada has several available controller interfaces for the user through their API. One of these, the joint-velocity-controller interface, is to be implemented for a project I'm currently working on in collaboration with Cheng Fang, Assistent Professor at SDU Robotics.

## Proposal

- Two interfaces are designed: one utilizing euler angles and the other quaternions.

### Common

The two interfaces calculates the jacobian and uses the moore-penrose pseudoinverse to determine the desired joint-velocities.

### Euler Angles

The interface 

```math
  q_d = J_{a}^{-1} \left( \dot{v} + K_p e \right)
```
