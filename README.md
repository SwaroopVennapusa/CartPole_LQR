# CartPole_LQR

This repo is the implementation of LQR Control for https://github.com/linZHank/invpend_experiment

### Instructions

Launch the gazebo environment

```roslaunch invpend_control load_invpend.launch```

To implement LQR control

```rosrun invpend_control cartpole_lqr.py```


### Tuning LQR

- Set the desired state by changing the X coordinate in the below matrix

```self.desired_state = np.matrix([1,0,0,0]).T```

- Tune the Q and R matrix accordingly
  - The diagonal elements in the Q matrix are the penalizing factor for each element of the state vecor.
  - The R matrix is a tuning parameter that determines the relative importance of the control input. Higher value means less aggressive.
  
  
# Plot

![LQR1](https://user-images.githubusercontent.com/126584953/222604314-1cc6b146-b1b9-43ac-ba61-d031371f95c3.png)

  
# Results


https://user-images.githubusercontent.com/126584953/222477133-a233ef06-157b-4734-bd87-b6885be02e64.mp4



