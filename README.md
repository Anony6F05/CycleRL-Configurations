# CycleRL Configurations

## **Physical Parameters and Hyperparameters of the Simulation Environment (Isaac Sim)**  

| Parameter Category | Symbol | Value/Range |
| :--- | :---: | :--- |
| **_Inertial Properties (from USD)_** | | |
| Frame Mass | $m_{frame}$ | 18.5 kg |
| Handlebar Mass | $m_{handle}$ | 1.9 kg |
| Front / Rear Wheel Mass | $m_{wheel}$ | 2.5 kg / 4.5 kg |
| Frame Inertia ($I_{xx}, I_{yy}, I_{zz}$) | $I_{frame}$ | [0.95, 1.12, 0.45] kg$\cdot$m$^2$ |
| Center of Gravity Height | $h_{CoG}$ | 0.65 m |
| **_Geometric Kinematics_** | | |
| Wheelbase | $L$ | 1.1 m |
| Wheel Radius | $r_{wheel}$ | 0.33 m |
| Steering Axis Tilt (Caster Angle) | $\alpha$ | 20.0$^{\circ}$ |
| Mechanical Trail | $t_{trail}$ | 0.11 m |
| Fork Offset | $l_{offset}$ | 0.01 m |
| Max Steering Angle | $\delta_{max}$ | $\pm 45^{\circ}$ |
| **_Actuator Dynamics_** | | |
| Steering Motor Max Torque | $\tau_{steer}^{max}$ | 12.0 N$\cdot$m |
| Steering Joint Damping | $d_{steer}$ | 0.5 N$\cdot$m$\cdot$s/rad |
| Drive Motor Max Torque | $\tau_{drive}^{max}$ | 40.0 N$\cdot$m |
| Physics Simulation Step | $dt_{sim}$ | 0.05 s (50 Hz) |
| **_Contact & Surface Interaction_** | | |
| Surface Friction Coefficient | $\mu$ | $\mathcal{U}(0.5, 1.5)$ |
| Restitution (Bounciness) | $e$ | 0.0 |
| Contact Stiffness | $k_{contact}$ | $10^4$ N/m |
| Linear/Angular Damping | $\zeta$ | 0.05 / 0.05 |
| **_Simulation Hyperparameters_** | | |
| Parallel Environments | $N_{env}$ | 4,096 |
| Control Frequency | $f_{ctrl}$ | 50 Hz |
| Episode Duration | $T_{ep}$ | 64 s (3,200 steps) |
| Total Training Epochs | $N_{epoch}$ | 5,000 |

> *Note: Inertial values are estimated from the USD model. Friction coefficients are randomized during training as part of the domain randomization strategy.*


## Training Configurations

| Hyperparameters                  | Value                           |
| -------------------------------- | ------------------------------- |
| **_PPO Hyperparameters_**        |                                 |
| Learning rate                    | 1 × 10^-4 (cosine annealing)    |
| Discount factor γ                | 0.99                            |
| GAE parameter λ                  | 0.95                            |
| Policy clip ratio ε              | 0.2                             |
| Value function coefficient       | 1.0                             |
| **_Network Architecture_**       |                                 |
| Actor network                    | 5-layer MLP [512, 256, 128, 64] |
| Critic network                   | 5-layer MLP [512, 256, 128, 64] |
| Activation function              | ELU                             |
| **_Simulation Hyperparameters_** |                                 |
| Parallel environments            | 4,096                           |
| Simulation frequency             | 50 Hz                           |
| Episode duration                 | 64 seconds (3,200 steps)        |
| Total training epochs            | 5,000                           |

> Note: MLP includes input-to-hidden and hidden-to-output transformations for 8-dimensional input and 2-dimensional output.


## Specifications of Hardware Platform

| Component                 | Specification                   |
| ------------------------- | ------------------------------- |
| **Mechanical Platform**   |                                 |
| Base vehicle              | Modified Mountain Bike          |
| Wheelbase                 | 1,100 mm                        |
| Total mass                | 25.0 kg (including electronics) |
| Center of gravity height  | 0.65 m                          |
| **Computational System**  |                                 |
| Main processor            | NVIDIA Jetson Orin NX           |
| Memory                    | 16 GB LPDDR5                    |
| Storage                   | 512 GB NVMe SSD                 |
| Operating system          | Ubuntu 20.04                    |
| **Sensing and Actuation** |                                 |
| IMU                       | WheelTec N100                   |
| Steering servo motor      | Unitree GO-M8010-6              |
| Hub motor                 | BaFang RM G020.500.D 12         |
| Power system              | 48V 9.6Ah lithium battery       |
