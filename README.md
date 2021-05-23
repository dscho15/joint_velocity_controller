# Joint Velocity Controller

## The purpose

The Franka Emika Panda has several available controller interfaces for the user. One of these, the joint-velocity-controller interface, is implemented for a project I am currently working on in collaboration with Cheng Fang, whom is a Assistent Professor at SDU Robotics.

## Proposal

- Two interfaces are designed: one utilizing euler angles and the other quaternions.

### Common

The two interfaces calculates the jacobian and uses the moore-penrose pseudoinverse to determine desired joint-velocities.

### Euler Angles

### Quaternions
