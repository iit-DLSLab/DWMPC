from gym_quadruped.quadruped_env import QuadrupedEnv
import pydwmpc
import numpy as np
import time


robot_name = "aliengo"   # "aliengo", "mini_cheetah", "go2", "hyqreal", ...
scene_name = "flat"
robot_feet_geom_names = dict(FR='FR',FL='FL', RR='RR' , RL='RL')
robot_leg_joints = dict(FR=['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', ],
                        FL=['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', ],
                        RR=['RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', ],
                        RL=['RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'])
mpc_frequency = 100.0
state_observables_names = tuple(QuadrupedEnv.ALL_OBS)  # return all available state observables

sim_frequency = 200.0
env = QuadrupedEnv(robot=robot_name,
                   scene=scene_name,
                   sim_dt = 1/sim_frequency,  # Simulation time step [s]
                   ref_base_lin_vel=0.0, # Constant magnitude of reference base linear velocity [m/s]
                   ground_friction_coeff=1.5,  # pass a float for a fixed value
                   base_vel_command_type="human",  # "forward", "random", "forward+rotate", "human"
                   state_obs_names=state_observables_names,  # Desired quantities in the 'state'
                   )
obs = env.reset(random=False)

env.render()

mpc = pydwmpc.Dwmpc()
mpc.init()
mpc.startWalking()    
timer = 0
tau = pydwmpc.DoubleVector([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
des_q = pydwmpc.DoubleVector([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
des_dq = pydwmpc.DoubleVector([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
contact = pydwmpc.DoubleVector([0.0,0.0,0.0,0.0])

Kp = 20
Kd = 3
# mpc.setGaitParam(0.8,0.6,1)
mpc.setStepHeight(0.1)
# mpc.goHandStand()

while True:
    
    qpos = env.mjData.qpos
    qvel = env.mjData.qvel
   
    if timer % (sim_frequency / mpc_frequency) == 0 or timer == 0:
        
        foot_op = np.array([env.feet_pos('world').FL, env.feet_pos('world').FR, env.feet_pos('world').RL, env.feet_pos('world').RR],order="F")
        contact_op = np.array([float(env.feet_contact_state()[0].FL), float(env.feet_contact_state()[0].FR), float(env.feet_contact_state()[0].RL), float(env.feet_contact_state()[0].RR)])
        
        quat = np.zeros(4)
        quat[0] = qpos[4]
        quat[1] = qpos[5]
        quat[2] = qpos[6]
        quat[3] = qpos[3]

        ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()
        p = qpos[:3].copy()
        q = qpos[7:].copy()

        dp = qvel[:3].copy()
        omega = env.base_configuration[:3,:3]@qvel[3:6]
        dq = qvel[6:].copy()

        # q[0] = - q[0]
        # q[6] = - q[6]
        
        # dq[0] = - dq[0]
        # dq[6] = - dq[6]

        mpc.run(p,
            quat,
            q,
            dp,
            omega,
            dq,
            1/mpc_frequency,
            contact_op,
            foot_op,
            env.heading_orientation_SO3.transpose()@ref_base_lin_vel,
            ref_base_ang_vel,
            np.array([0.0, 0.0, 0.0, 1.0]),
            contact,
            tau,
            des_q,
            des_dq)
        
        mpc.prepare()
        
        # tau[0] = - tau[0]
        # tau[6] = - tau[6]
        # des_q[0] = - des_q[0]
        # des_q[6] = - des_q[6]
        # des_dq[0] = - des_dq[0]
        # des_dq[6] = - des_dq[6]
    action_torque = tau + Kp*(des_q.getList() - qpos[7:]) + Kd*(des_dq.getList() - qvel[6:])
    action = np.zeros(env.mjModel.nu)
    action[env.legs_tau_idx.FL] = action_torque[:3]
    action[env.legs_tau_idx.FR] = action_torque[3:6]
    action[env.legs_tau_idx.RL] = action_torque[6:9]
    action[env.legs_tau_idx.RR] = action_torque[9:]
    state, reward, is_terminated, is_truncated, info = env.step(action=action)
    timer += 1
    if is_terminated:
        pass
        # Do some stuff
    env.render()
env.close()