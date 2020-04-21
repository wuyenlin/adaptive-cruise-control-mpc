## Model Predictive Control- Final Project

A model predictive control (MPC) approach is implemented on a basic Adaptive Cruise Control (ACC) system. A vehicle is moving with a constant velocity and a following vehicle approaches the preceding vehicle and should maintain the same velocity. Using an MPC controller, the required stability with the specified input constraints and the target velocity for a constant preceding vehicle velocity is achieved. The outcome is compared with that using an Linear Quadratic Regulator (LQR).
Through simulations in MATLAB, it is shown that the proposed MPC strategy is able to maintain a vehicle in designated constraints.

![](https://i.imgur.com/hwmI7oa.png)

This final project is a part of requirement of course SC42125 Model Predictive Control at TU Delft and is completed by Nikhil Nagendra and Yen-Lin Wu. 

## Reference
```
@article{Takahama,
  title={Model Predictive Control Approach to Design Practical Adaptive Cruise Control for Traffic Jam},
  author={Taku Takahama and Daisuke Akasaka},
  journal={International Journal of Automotive Engineering},
  volume={9},
  number={3},
  pages={99-104},
  year={2018},
  doi={10.20485/jsaeijae.9.3_99}
}
@mastersthesis{ACC_SG,
title={Design of a Hybrid Adaptive Cruise Control Stop\-\&\-Go System},
  author={R.A.P.M. van den Bleek},
year={2007},
school={Technische Universiteit Eindhoven},
address={5612 AZ Eindhoven}
}
@book{textbook,
author = {Rawlings, J. and Mayne, D.Q. and Diehl, Moritz},
publisher = {Nob Hill Publishing},
year = {2017},
month = {01},
title = {Model Predictive Control: Theory, Computation, and Design}
}
```