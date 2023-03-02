# CartPole_LQR

THis repo is the implementation of LQR Control for https://github.com/linZHank/invpend_experiment

### Instructions

Clone the repo

`https://github.com/SwaroopVennapusa/CartPole_LQR.git`

Launch the gazebo environment

`roslaunch invpend_control load_invpend.launch`

To implement LQR control

`rosrun invpend_control cartpole_lqr.py`


### Tuning LQR

- Set the desired state by changing the X coordinate in the below matrix

'self.desired_state = np.matrix([1,0,0,0]).T'

- Tune the Q and R matrix accordingly
  - The diagonal elements in the Q matrix are the penalizing factor for each element of the state vecor.
  - The R matrix is a tuning parameter that determines the relative importance of the control input. Higher value means less aggressive.
  
  
  
# Results

(https://youtu.be/QuxuTJ5_fQQ)
