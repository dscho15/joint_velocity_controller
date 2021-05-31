# Joint Velocity Controller

## The purpose

The Franka Emika Panda has several available controller interfaces for the user. One of these, the joint-velocity-controller interface, is implemented for a project I am currently working on in collaboration with Cheng Fang, whom is a Assistent Professor at SDU Robotics.

## Proposal

- Two interfaces are designed, one utilizing euler angles and the other quaternions.

## Common

The interface would normally compute the jacobian $J$, then use the moore-penrose pseudoinverse $J^+$ to determine desired joint-velocities.

$$
v = J(q)\dot{q}
$$

$$
\dot{q}=J^+(q)\dot{q}
$$

however, it does not take the dynamical hardware limits of the robot into account. In order to satisfy the constraints of the hardware, we propose a quadratic program of the form

$$
\begin{matrix}
\text{minimize} \; \; & \frac{1}{2} x^T H x + g^Tx \\
\text{subject to} & Gx\leq h \\
                  & Ax = b \\
                  & lb \leq x \leq ub
\end{matrix}
$$

where 
> $x=\dot{q}$, is the joint velocities

> $H=J(q)^TJ(q)$, is a positive-semi-definite matrix.

> $g^T = -J(q)^T\dot{x}_d$, is a byproduct of least-squares multiplication. Notice $\dot{x}_d$

> $lb \; \text{and} \; ub$, is the lower bounds and upper bounds of the joint velocities $\dot{q}$, they are computed at each timestamp.

> We do not utilize the G-term nor the A-term, so they can be ignored as it is of now.

## Dependencies

Wrapper around Quadratic Programming (QP) solvers in Python, with a unified interface.
    
    sudo apt install python3-dev
    pip3 install qpsolvers
    pip3 install osqp
    pip3 install flake8