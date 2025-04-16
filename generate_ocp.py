from adam.casadi import KinDynComputations
import numpy as np
import casadi as cs
from acados_template import *
from liecasadi import SE3, SO3, SO3Tangent
from scipy.spatial.transform import Rotation as R

class ocp_formulation:

    def __init__(self,args):

        #Sub-Problem parameters
        self.joints_name_list_ = args['joints_name_list'] #list of actuated joints
        self.contact_frame_name_list_ = args['contact_frame_name_list'] #list of contact frames

        root_link = "trunk"

        #Whole-Body Problem parameters
        self.model_path_wb_ = args['model_path_wb'] #path to the urdf file
        self.joints_name_list_wb_ = args['joints_name_list_wb'] #list of actuated joints
        self.contact_frame_name_list_wb_ = args['contact_frame_name_list_wb'] #list of contact frames
        self.kinDyn_wb_ = KinDynComputations(self.model_path_wb_, self.joints_name_list_wb_, root_link) #compute the with ADAM the KinDyn values for the robot urdf

        # OCP parameters
        self.N_ = args['N'] #length of the horizon
        self.dt_ = args['dt'] # delta time between the integration node

        self.s_idx_ = args['s_idx'] # mapping joints position of the substistem in the right order

        self.nx_ = 0
        self.nu_ = 0

        self.lh = []
        self.uh = []

        # model = pinocchio.buildModelFromUrdf(self.model_path_wb_ )

        # self.cmodel = cpin.Model(model)
        # self.cdata =  self.cmodel.createData()

    def getOptimalProblem(self, dynamic_model="DISCRETE", model_name = "robot", make_model=True):

        print("Creating optimal control problem for the robot model: ", model_name)
        # if(dynamic_model == "CONTINUOUS"):
        #     print("changing to discrete dynamics model with semi-implicit euler, \n TO DO continuos")
            # dynamic_model = "DISCRETE"

        ocp = AcadosOcp()
        ocp.model.name = model_name

        n_joints = len(self.joints_name_list_)
        n_contact = len(self.contact_frame_name_list_)

        n_joints_wb = len(self.joints_name_list_wb_)
        n_contact_wb = len(self.contact_frame_name_list_wb_)

        s_idx =  self.s_idx_

        # Define the optimization variables depending on the dynamic model

                                            ##========  STATE ======== ##
        p =  cs.SX.sym("position",3,1) #trunk positon
        dp = cs.SX.sym("linear_speed",3,1) #trunk speed

        quat = cs.SX.sym("orientation",4,1) # psi in R^3 agular variation with the nominal
        omega = cs.SX.sym("angular_speed",3,1) #angular speed

        q = cs.SX.sym("joint_position",n_joints,1) #joint angular position
        dq = cs.SX.sym("joint_speed",n_joints,1) #joint speed

        tau = cs.SX.sym("joint_torque",n_joints,1) #joint torque

        grf = cs.SX.sym("ground_reaction_forces",3*n_contact,1) #Ground Reaction Forces
        grf_aux = cs.SX.sym("ground_reaction_forces_aux",3*n_contact,1) #Ground Reaction Forces (same dimension of the )
        grf_old = cs.SX.sym("grf_old",12,1)

        foot_ref = cs.SX.sym("foot_position_reference",3*(n_contact),1) #position of the leg
        contact_state = cs.SX.sym("contact_state",n_contact_wb,1) #contact state 1 in contact 0 if not

        quat_ref = cs.SX.sym("orientation_reference",4,1)  #quaternion reference for the orientation

        q_aux = cs.SX.sym("joint_position_aux",n_joints_wb,1) #joint angular position for the auxiliary joints
        dq_aux = cs.SX.sym("joint_speed_aux",n_joints_wb,1) #joint speed for the auxiliary joints

        dt = cs.SX.sym("dt",1,1) #Ground discretization time

        state = cs.vertcat(p,quat,q,dp,omega,dq)
        nx = 3+4+n_joints+3+3+n_joints
        self.nx_ = nx

        control = cs.vertcat(tau,grf,grf_aux)
        nu =  n_joints + 3*n_contact + 3*n_contact
        # control = cs.vertcat(tau,grf)
        # nu =  n_joints + 3*n_contact
        self.nu_ = nu

        parameter = cs.vertcat(contact_state,foot_ref,q_aux,dq_aux,quat_ref,dt,grf_old)

        ocp.model.x = state
        ocp.model.u = control
        ocp.model.p = parameter
        ocp.parameter_values = np.zeros((n_contact_wb +
                                         3*n_contact +
                                         n_joints_wb+
                                         n_joints_wb+
                                         4 +
                                         1 +
                                         12,1))

                                            ##========  DYNAMICS ======== ##

        ##========SYSTEM WHOLE BODY DYNAMICS DISCRETE========

        M_wb = self.kinDyn_wb_.mass_matrix_fun()
        C_wb = self.kinDyn_wb_.coriolis_term_fun()
        G_wb = self.kinDyn_wb_.gravity_term_fun()

        M = cs.SX.zeros(6+n_joints,6+n_joints)
        C = cs.SX.zeros(6+n_joints,1)
        G = cs.SX.zeros(6+n_joints,1)

        q_wb = cs.vertcat(q_aux[:3*s_idx],q,q_aux[n_joints+3*s_idx:]) #add the auxiliary joints to the state joints
        dq_wb = cs.vertcat(dq_aux[:3*s_idx],dq,dq_aux[n_joints+3*s_idx:]) #add the auxiliary joints speed to the state joints
        if s_idx == 0:
            grf_wb = cs.vertcat(grf,grf_old[6:]+grf_aux)
        else:
            grf_wb = cs.vertcat(grf_old[:6]+grf_aux,grf)
        w_H_b =SE3(p,quat).transform()

        # alpha0 = np.array([0.3,0.2,0.28,0.3,0.2,0.28])
        # alpha1 = np.array([0.45,0.5,0.9,0.45,0.5,0.9])
        # alpha2 = np.array([0.03,0.025,0.02,0.03,0.025,0.02])
        # for idx in range(0,n_joints) :
        #     tau_friction[idx] = cs.tanh(dq_wb[3*s_idx+idx]/0.03)*(alpha0[idx]) + alpha2[idx]*dq_wb[3*s_idx+idx]
        torque = cs.vertcat(np.zeros((6,1)),tau)
        ext_torque = cs.SX.zeros(6+n_joints)
        for idx in range(0,n_contact_wb) :
            if idx == s_idx or idx == (n_contact-1)+s_idx:
                J = self.kinDyn_wb_.jacobian_fun(self.contact_frame_name_list_wb_[idx])
                torque_wb = (contact_state[idx]*J(w_H_b,q_wb)[:3,:].T@grf_wb[3*idx:3+3*idx])
                ext_torque[:6] += torque_wb[:6]
                ext_torque[6:] += torque_wb[6+3*s_idx:6+3*s_idx+n_joints]
            else:
                J = self.kinDyn_wb_.jacobian_fun(self.contact_frame_name_list_wb_[idx])
                ext_torque[:6] += (contact_state[idx]*J(w_H_b,q_wb)[:3,:].T@grf_wb[3*idx:3*(idx+1)])[:6]

        #extraxt the Mass matrix of the subsystem from the the whole body mass matrix with the auxiliary joints

        M[:6,:6] = M_wb(w_H_b,q_wb)[:6,:6] #floating base
        M[6:6+n_joints,6:6+n_joints] = M_wb(w_H_b,q_wb)[6+3*s_idx:6+3*s_idx+n_joints,6+3*s_idx:6+3*s_idx+n_joints]
        M[:6,6:6+n_joints] = M_wb(w_H_b,q_wb)[:6,6+3*s_idx:6+3*s_idx+n_joints]
        M[6:6+n_joints,:6] = M_wb(w_H_b,q_wb)[6+3*s_idx:6+3*s_idx+n_joints,:6]

        C[:6] = C_wb(w_H_b,q_wb,cs.vertcat(dp,omega),dq_wb)[:6]
        C[6:] = C_wb(w_H_b,q_wb,cs.vertcat(dp,omega),dq_wb)[6+3*s_idx:6+3*s_idx+n_joints]

        G[:6] = G_wb(w_H_b,q_wb)[:6]
        G[6:] = G_wb(w_H_b,q_wb)[6+3*s_idx:6+3*s_idx+n_joints]

        inv_M = cs.inv(M)

        _v_next = cs.vertcat(dp,omega,dq) + inv_M@(- C - G + torque + ext_torque)*dt

        ##========INTERGRATOR========
        #semi-implicit euler
        _p_next = cs.SX.sym("p_next",n_joints + 7,1)
        _p_next[:3] = p + _v_next[:3]*dt
        _p_next[3:7] = (SO3Tangent(_v_next[3:6] * dt) + SO3(quat)).as_quat().coeffs()
        _p_next[7:] = q + _v_next[6:]*dt
        # _tau_next = tau + dtau*dt
        expr_phi = cs.vertcat(_p_next,_v_next)

        ocp.model.disc_dyn_expr = expr_phi

                                            ##========CONSTRAINTS========##

        ## === NON SLIPPING CONDITION ===
        eps_sc = 5e-3
        expr_sc = cs.SX.ones(3*n_contact,1)
        v = cs.vertcat(dp,omega,dq)
        kk = 0
        for frame in self.contact_frame_name_list_:
            J = self.kinDyn_wb_.jacobian_fun(frame)
            J_sb = J(w_H_b,q_wb)[:3,:6+n_joints]
            J_sb[:,6:] = J(w_H_b,q_wb)[:3,6+3*s_idx:6+3*s_idx+n_joints]
            expr_sc[3*kk:3+3*kk] = contact_state[s_idx+kk]*J_sb@v #zero linear speed of the foot when on the ground
            kk = kk + 1
        ng_sc = 3*n_contact
        lh_sc = np.zeros(ng_sc) - eps_sc
        uh_sc = np.zeros(ng_sc) + eps_sc

        # ## === Z reference for the foot ===
        #CAN BE REMOVED FOR SPEED
        eps_zc = 5e-3
        expr_zc = cs.SX.zeros(n_contact,1)
        kk = 0
        for frame in self.contact_frame_name_list_:
            H = self.kinDyn_wb_.forward_kinematics_fun(frame)
            expr_zc[kk] = (H(w_H_b,q_wb)[2,3]-foot_ref[3*(kk)+2]) #z position of the foot
            kk = kk + 1
        lh_zc = np.zeros(n_contact) - eps_zc
        uh_zc = np.zeros(n_contact) + eps_zc
        ng_zc = n_contact

        ## == FRICTION CONE ===
        mu = 0.5
        Fmin = 0
        Fmax = 500
        expr_fc = cs.SX.ones(5*n_contact_wb,1)
        uh_fc = np.zeros(5*n_contact_wb)
        lh_fc = np.zeros(5*n_contact_wb)
        # idx = s_idx
        kk = 0
        for idx in range(n_contact_wb):
                expr_fc[5*kk : 5 + 5*kk] = cs.vertcat(mu*grf_wb[3*idx+2]+grf_wb[3*idx],
                                                     mu*grf_wb[3*idx+2]-grf_wb[3*idx],
                                                     mu*grf_wb[3*idx+2]+grf_wb[3*idx+1],
                                                     mu*grf_wb[3*idx+2]-grf_wb[3*idx+1],
                                                     grf_wb[3*idx+2])
                kk = kk + 1
        uh_fc = np.ones(5*n_contact_wb)*Fmax
        lh_fc = np.array([0,0,0,0,Fmin]*n_contact_wb)

        ng_fc = 5*n_contact_wb

        #  ## == FRICTION CONE ===
        # mu = 0.5
        # Fmin = 0
        # Fmax = 500
        # expr_fc = cs.SX.ones(5*n_contact,1)
        # uh_fc = np.zeros(5*n_contact)
        # lh_fc = np.zeros(5*n_contact)
        # # idx = s_idx
        # kk = 0
        # for idx in range(n_contact):
        #         expr_fc[5*kk : 5 + 5*kk] = cs.vertcat(mu*grf[3*idx+2]+grf[3*idx],
        #                                              mu*grf[3*idx+2]-grf[3*idx],
        #                                              mu*grf[3*idx+2]+grf[3*idx+1],
        #                                              mu*grf[3*idx+2]-grf[3*idx+1],
        #                                              grf[3*idx+2])
        #         kk = kk + 1
        # uh_fc = np.ones(5*n_contact)*Fmax
        # lh_fc = np.array([0,0,0,0,Fmin]*n_contact)

        # ng_fc = 5*n_contact

        ## === torque limits === ##
        expr_tc = tau
        lh_tc = -np.array([44,44,44]*n_contact)
        uh_tc = np.array([44,44,44]*n_contact)
        ng_tc = n_joints

        ## === Kinematic limit == ## TO DO put real limits
        expr_kc = q
        lh_kc = np.array([-1.22173,-1.57,-2.77507 ]*n_contact)
        uh_kc = np.array([1.22173,1.57,-0.645772]*n_contact)
        ng_kc = n_joints


        ## === total constraint === ##

        # ocp.model.con_h_expr = cs.vertcat(expr_fc,expr_zc,expr_sc,expr_kc,expr_tc)
        # ocp.constraints.uh = np.concatenate((uh_fc,uh_zc,uh_sc,uh_kc,uh_tc))
        # ocp.constraints.lh = np.concatenate((lh_fc,lh_zc,lh_sc,lh_kc,lh_tc))
        
        self.lh = ocp.constraints.lh
        print(self.lh.shape)
        self.uh = ocp.constraints.uh
        # ng_c = ng_sc + ng_fc + ng_zc

        # ocp.model.con_h_expr = cs.vertcat(expr_fc,expr_sc)
        # ocp.constraints.uh = np.concatenate((uh_fc,uh_sc))
        # ocp.constraints.lh = np.concatenate((lh_fc,lh_sc))
        # self.lh = np.concatenate((lh_fc,lh_sc))
        # self.uh = np.concatenate((uh_fc,uh_sc))
        # ng_c = ng_sc + ng_fc
        ocp.model.con_h_expr = expr_sc
        ocp.constraints.uh = uh_sc
        ocp.constraints.lh = lh_sc
        self.lh = ocp.constraints.uh 
        self.uh =  ocp.constraints.lh
        ng_c = ng_sc  
        # ocp.model.con_h_expr_0 = cs.vertcat(expr_fc,expr_tc)
        # ocp.constraints.lh_0 = np.concatenate((lh_fc,lh_tc))
        # ocp.constraints.uh_0 = np.concatenate((uh_fc,uh_tc))

        # ocp.model.con_h_expr_0 = expr_fc
        # ocp.constraints.lh_0 = lh_fc
        # ocp.constraints.uh_0 = uh_fc

        # ## === add slack for non liear constraints === ##
        ocp.constraints.lsh = np.zeros(ng_c)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
        ocp.constraints.ush = np.zeros(ng_c)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
        ocp.constraints.idxsh = np.array(range(ng_c))    # Jsh

        ocp.cost.zl = 1000 * np.ones((ng_c,)) # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        ocp.cost.Zl = 1 * np.ones((ng_c,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        ocp.cost.zu = 1000 * np.ones((ng_c,))
        ocp.cost.Zu = 1 * np.ones((ng_c,))

        # ocp.constraints.lsh_0 = np.zeros(ng_fc)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
        # ocp.constraints.ush_0 = np.zeros(ng_fc)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
        # ocp.constraints.idxsh_0 = np.array(range(ng_fc))    # Jsh

        # ocp.cost.zl_0 = 1000 * np.ones((ng_fc,)) # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        # ocp.cost.Zl_0 = 1 * np.ones((ng_fc,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        # ocp.cost.zu_0 = 1000 * np.ones((ng_fc,))
        # ocp.cost.Zu_0 = 1 * np.ones((ng_fc,))
        #                                     ##========OBJECTIVE FUNCTION========##

        cost_type = 'NONLINEAR_LS'
        # cost_type = 'LINEAR_LS'

        ocp.cost.cost_type = cost_type
        ocp.cost.cost_type_e = cost_type


        ## running cost (for state and control)
        kk = 0
        foot = cs.SX.zeros(3*n_contact,1)
        for frame in self.contact_frame_name_list_:
            foot[kk:kk+3] = self.kinDyn_wb_.forward_kinematics_fun(frame)(w_H_b,q_wb)[:3,3]
            kk = kk + 3

        angle_error = (SO3(quat) - SO3(quat_ref)).vec

        # 1-3 2-3
        # if s_idx == 0:
        #     consensus = cs.vertcat(dp,omega)
        # else:
        #     consensus = cs.vertcat(-dp,-omega)
        consensus = cs.vertcat(dp,omega)

        ocp.model.cost_y_expr = cs.vertcat(p,angle_error,q,dp,omega,dq,foot,tau,grf_wb,consensus)

        ## final cost (only state)
        ocp.model.cost_y_expr_e = cs.vertcat(p,angle_error,q,dp,omega,dq,foot)

        #COST
        weight_postion = np.array([1e1,1e1,5e4])
        weight_angle = 1e0*np.ones(3)
        weight_angular_speed = 1e1*np.ones(3)
        weight_linear_speed = 2e2*np.ones(3)

        ### TRACKING COST FOOT
        weight_foot_pos = 1e5*np.ones(3*n_contact)
        ### POSTUREAL REGULARIZATION COSTweight_joint
        weight_joint = 1e2*np.ones(n_joints)
        weight_joint_vel = 1e1*np.ones(n_joints)
        ### TORQUE COST
        weight_torque = 1e-2*np.ones(n_joints)

        ### CONTROL INPUT REGULARIZATION COST
        weight_grond_reaction_forces = 1e-2*np.ones(3*n_contact)
        rho = [2]
        ### CONSENSUS COST
        weight_consensus_wrench = np.array(rho*6)


        Q = np.concatenate((weight_postion,weight_angle,weight_joint,weight_linear_speed,weight_angular_speed,weight_joint_vel,weight_foot_pos))
        R = np.concatenate((weight_torque,weight_grond_reaction_forces,weight_grond_reaction_forces,weight_consensus_wrench))
        W = np.diag(np.concatenate((Q,R)))

        ocp.cost.W = W
        ocp.cost.W_e = np.diag(Q)

        ocp.cost.yref = np.zeros(cs.SX.size(ocp.model.cost_y_expr,1))
        ocp.cost.yref_e = np.zeros(cs.SX.size(ocp.model.cost_y_expr_e,1))
        for idx in range(5*n_contact_wb):
            ocp.formulate_constraint_as_L2_penalty(expr_fc[idx],1e5,uh_fc[idx],lh_fc[idx])
                                                ## == INITIAL CONDITION == ##
        ocp.constraints.x0 = np.zeros((nx,1))
                                                  ## == SOLVER OPTIONS == ##
        tol = 1e-3
        shooting_nodes = np.zeros(self.N_+1)
        for(idx) in range(self.N_+1):
            if idx < 3:
                shooting_nodes[idx] = idx*0.01
            else:
                shooting_nodes[idx] = shooting_nodes[idx-1] + 0.03
                
        # shooting_nodes = np.linspace(0,self.N_*self.dt_,self.N_+1)
        print(shooting_nodes)
        tol = 1e-3

        ocp.dims.N = self.N_
        ocp.solver_options.shooting_nodes = shooting_nodes
        # ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES , PARTIAL_CONDENSING_HPIPM
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # GAUSS_NEWTON, EXACT
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI or SQP
        ocp.solver_options.qp_solver_warm_start = 2
        # ocp.solver_options.regularize_method = 'CONVEXIFY'
        ocp.solver_options.hpipm_mode = 'SPEED_ABS'
        ocp.solver_options.nlp_solver_max_iter = 1
        # ocp.solver_options.globalization = "MERIT_BACKTRACKING"
        ocp.solver_options.ext_fun_compile_flags = '-O3'
        ocp.solver_options.integrator_type = "DISCRETE"
        # ocp.solver_options.tol = tol
        ocp.solver_options.print_level = 0

        ocp.solver_options.tf = shooting_nodes[self.N_]
        # ocp.translate_to_feasibility_problem(keep_x0=True, keep_cost=True)
        if make_model :
            ocp_solver =  AcadosOcpSolver(ocp, json_file = model_name+'acados_ocp.json')

        else :
            ocp_solver =  AcadosOcpSolver(ocp, json_file = model_name+'acados_ocp.json', build = False, generate = True)


        return ocp_solver
    
args = {}
make_model = True
N = 15
dt = 0.01
problems = []

                                ###################### FRONT MODEL ########################
# # The joint list
# args['joints_name_list'] = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint','FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint']
# #list of the frame for the contacts
# args['contact_frame_name_list'] = ['FR_foot','FL_foot']
# #path for the complete model
# args['model_path_wb'] = '/home/iit.local/lamatucci/DWMPC/urdf/go1.urdf'
# # The joint list
# args['joints_name_list_wb'] = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
#     'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
#     'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
#     'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
# ]
# args['contact_frame_name_list_wb'] = ['FR_foot','FL_foot','RR_foot','RL_foot']

args['joints_name_list'] = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint','FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint']
#list of the frame for the contacts
args['contact_frame_name_list'] = ['FL_foot','FR_foot']
#path for the complete model
args['model_path_wb'] = '/home/iit.local/lamatucci/DWMPC/urdf/aliengo.urdf'
# The joint list
args['joints_name_list_wb'] = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]
args['contact_frame_name_list_wb'] = ['FL_foot','FR_foot','RL_foot','RR_foot']

args['s_idx'] = 0

joints_name_list = args['joints_name_list_wb']
contact_frame_name_list = args['contact_frame_name_list_wb']
model_path = args['model_path_wb']
n_contact = len(contact_frame_name_list)
# robot state dimension
n_joints = len(joints_name_list)
args['N'] = N # Horizon lenght
args['dt'] = dt # delta time between the integration node

front = ocp_formulation(args)
front_solver = front.getOptimalProblem(model_name = "front")
                                ###################### BACK MODEL ########################
# The joint list
args['joints_name_list'] = [
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
#list of the frame for the contacts
args['contact_frame_name_list'] = ['RL_foot','RR_foot']

args['s_idx'] = 2

back = ocp_formulation(args)
back_solver = back.getOptimalProblem(model_name = "back")