#!/usr/bin/env python3
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
#sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))

import rospy
import tf.transformations
from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import Twist, Transform, PoseStamped, TwistStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


import do_mpc
import numpy as np
import casadi as ca
from itertools import chain
from typing import Dict, List, Optional, Tuple


from scripts.llm import LLM, Optimization
from config.config import ControllerConfig



class Controller:
    def __init__(self, env_info:Tuple[List]=([{"name":"robot"}], [{"name":"cube"}]), cfg=ControllerConfig) -> None:
        self.cfg = cfg

        # init info of robots and objects
        self.robots_info, self.objects_info = env_info

        # init model dynamics
        self.init_model()

        # init controllerf
        self.init_controller()

        # init ros node
        self.init_node()

        # init variables and expressions
        self.init_expressions()

        # home ee position TODO: put in config
        self.home_position = None
        # home ee orientation TODO: put in config
        self.home_quaternion = [0.924, -0.383, 0., 0.]


        # gripper fingers offset for constraints 
        self.gripper_offsets = [np.array([0., -0.048, 0.]), np.array([0., 0.048, 0.]), np.array([0., 0., 0.048])]

    def init_node(self):
        # node
        rospy.init_node('controller', anonymous=True)
        # subs
        self.ee_pose_sub = Subscriber('/ee_pose', PoseStamped)
        self.ee_twist_sub = Subscriber('/ee_twist', TwistStamped)
        self.synced_subs = ApproximateTimeSynchronizer([self.ee_pose_sub, self.ee_twist_sub],
                                                       queue_size=1,
                                                       slop=0.2, # TODO: set this param in the config
                                                       allow_headerless=False)
        self.synced_subs.registerCallback(self.set_x0) # sync subs 
        self.TP_message_sub = rospy.Subscriber('/TP_message', String, self.TP_callback, queue_size=1)
        self.OD_message_sub = rospy.Subscriber('/OD_message', String, self.OD_callback, queue_size=1)
        # pubs
        self.ee_trajectory_pub = rospy.Publisher('/ee_trajectory', MultiDOFJointTrajectory, queue_size=1)
        

    def init_model(self):
        # inti do_mpc model
        self.model = do_mpc.model.Model(self.cfg.model_type) 

        """ Parameter """
        # simulation time
        self.t = self.model.set_variable('parameter', 't')
        # position of objects
        self.objects = {} 
        for o in self.objects_info:
            self.objects[o['name']] = self.model.set_variable(var_type='_p', var_name=o['name'], shape=(3,1))

        # home position [x y, z]
        self.x0 = {}
        #for r in self.robots_info:
        #  self.x0[f'x0{r["name"]}'] = self.model.set_variable(var_type='_p', var_name=f'x0{r["name"]}', shape=(3,1))
        
        # gripper pose [x, y, z, theta, gamma, psi]       
        self.pose = []    

                
        # position (x, y, z)
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(self.cfg.nx,1))
        self.psi = self.model.set_variable(var_type='_x', var_name='psi', shape=(1,1))
        self.dx = self.model.set_variable(var_type='_x', var_name='dx', shape=(self.cfg.nx,1))
        self.dpsi = self.model.set_variable(var_type='_x', var_name='dpsi', shape=(1,1))
        self.u = self.model.set_variable(var_type='_u', var_name='u', shape=(self.cfg.nu,1))
        self.u_psi = self.model.set_variable(var_type='_u', var_name='u_psi', shape=(1,1))
        # system dynamics
        self.model.set_rhs('x', self.x + self.dx * self.cfg.dt)
        self.model.set_rhs('psi', self.psi + self.dpsi * self.cfg.dt)
        self.model.set_rhs('dx', self.u)
        self.model.set_rhs('dpsi', self.u_psi)
            
        # setup model
        self.model.setup()
    
    def set_objective(self, mterm: ca.SX = ca.DM([[0]])): # TODO: not sure if ca.SX is the right one
        # objective terms
        regularization = 0
        regularization += ca.norm_2(self.x[2] - 0.2)**2 # TODO: remove this, it's just for testing
        regularization += .4 * ca.norm_2(self.dx)**2
        #regularization += .4*ca.norm_2(ca.cos(self.psi[i]) - np.cos(r['euler0'][-1]))**2 # TODO 0.1 is harcoded
        mterm = mterm + regularization # TODO: add psi reference like this -> 0.1*ca.norm_2(-1-ca.cos(self.psi_right))**2
        lterm = 0.4*mterm
        # state objective
        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        # input objective
        u_kwargs = {'u':1.}#, 'u_psi':1.} 
        self.mpc.set_rterm(**u_kwargs)

    def set_constraints(self, nlp_constraints: Optional[List[ca.SX]] = None):

        # base constraints (state)
        self.mpc.bounds['lower','_x', 'x'] = np.array([-3., -3., 0.0]) # stay above table
        #self.mpc.bounds['upper','_x', f'psi{r["name"]}'] = np.pi/2 * np.ones((1, 1))   # rotation upper bound
        #self.mpc.bounds['lower','_x', f'psi{r["name"]}'] = -np.pi/2 * np.ones((1, 1))  # rotation lower bound

        # base constraints (input)
        self.mpc.bounds['upper','_u', 'u'] = self.cfg.hu * np.ones((self.cfg.nu, 1))  # input upper bound
        self.mpc.bounds['lower','_u', 'u'] = self.cfg.lu * np.ones((self.cfg.nu, 1))  # input lower bound
        #self.mpc.bounds['upper','_u', 'u_psi'] = np.pi * np.ones((1, 1))   # input upper bound
        #self.mpc.bounds['lower','_u', 'u_psi'] = -np.pi * np.ones((1, 1))  # input lower bound

        if nlp_constraints == None: 
            return

        for i, constraint in enumerate(nlp_constraints):
            self.mpc.set_nl_cons(f'const{i}', expr=constraint, ub=0., 
                                soft_constraint=True, 
                                penalty_term_cons=self.cfg.penalty_term_cons)

    def init_mpc(self):
        # init mpc model
        self.mpc = do_mpc.controller.MPC(self.model)
        # setup params
        setup_mpc = {'n_horizon': self.cfg.T, 't_step': self.cfg.dt, 'store_full_solution': False}
        # setup mpc
        self.mpc.set_param(**setup_mpc)
        self.mpc.settings.supress_ipopt_output() # => verbose = False

    def init_controller(self):
        # init
        self.init_mpc()
        # set functions
        # TODO: should the regularization be always applied?
        self.set_objective()
        self.set_constraints()
        # setup
        self.mpc.set_uncertainty_values(t=np.array([0.])) # init time to 0
        self.mpc.setup()
        self.mpc.set_initial_guess()

    def init_expressions(self):
        # init variables for python evaluation
        self.eval_variables = {"ca":ca, "np":np, "t":self.t} # python packages

        #self.R = [] # rotation matrix for angle around z axis
        #for i in range(len(self.robots_info)):
        # rotation matrix
        self.R = np.array([[ca.cos(self.psi), -ca.sin(self.psi), 0],
                            [ca.sin(self.psi), ca.cos(self.psi), 0],
                            [0, 0, 1.]])

    def set_t(self, t:float):
        """ Update the simulation time of the MPC controller"""
        self.mpc.set_uncertainty_values(t=np.array([t]))

    def set_x0(self, ee_pose:PoseStamped, ee_twist:TwistStamped):
        # position kinematics
        x = np.array([ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z])
        dx = np.array([ee_twist.twist.linear.x, ee_twist.twist.linear.y, ee_twist.twist.linear.z])
        # angular kinematics
        quaternion = [ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z, ee_pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
        psi = np.array([euler[2]])
        # set mpc states
        self.home_position = x
        self.mpc.x0 = np.concatenate((x, psi, dx, [0])) # TODO: dpsi harcoded to 0. maybe remove completely rotation control


    def init_states(self, observation:Dict[str, np.ndarray], t:float):
        """ Set the values the MPC initial states and variables """
        # set mpc x0
        self.set_x0(observation)
        # set variable parameters
        parameters = {'t': [t]}
        parameters = parameters | {o['name']: [observation[o['name']]] for o in self.objects_info}
        self.mpc.set_uncertainty_values(**parameters)

    def reset(self, observation: Dict[str, np.ndarray], t:float = 0) -> None:
        """
        observation: robot observation from simulation containing position, angle and velocities 
        """
        # TODO
        self.init_states(observation, t)
        return

    def _eval(self, code_str: str, offset=np.zeros(3)):
        #TODO the offset is still harcoded
        # put together variables for python code evaluation:    
        
        # initial state of robots before applying any action
        x0 = {'x0': self.home_position} 
        # robot variable states (decision variables in the optimization problem)
        robots_states = {}
        #for i, r in enumerate(self.robots_info):
        robots_states['x'] = self.x + self.R@offset
        robots_states['dx'] = self.dx
        
        eval_variables = {**self.eval_variables, **robots_states, **self.objects, **x0}
        # evaluate code
        evaluated_code = eval(code_str, eval_variables)
        return evaluated_code

    
    def retrieve_trajectory(self):
        # retrieve state trajectory
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = ["end_effector"]  # Replace with your actual joint/frame names

        for i in range(len(self.mpc.opt_x_num['_x', :, 0, 0])):
            transform = Transform()
            transform.translation.x = self.mpc.opt_x_num['_x', :, 0, 0][i].toarray()[0].squeeze()
            transform.translation.y = self.mpc.opt_x_num['_x', :, 0, 0][i].toarray()[1].squeeze()
            transform.translation.z = self.mpc.opt_x_num['_x', :, 0, 0][i].toarray()[2].squeeze()
            transform.rotation.x = self.home_quaternion[0]
            transform.rotation.y = self.home_quaternion[1]
            transform.rotation.z = self.home_quaternion[2]
            transform.rotation.w = self.home_quaternion[3]

            # Create Twist for velocity
            twist = Twist()
            twist.linear.x = self.mpc.opt_x_num['_x', :, 0, 0][i].toarray()[4].squeeze()
            twist.linear.y = self.mpc.opt_x_num['_x', :, 0, 0][i].toarray()[5].squeeze()
            twist.linear.z = self.mpc.opt_x_num['_x', :, 0, 0][i].toarray()[6].squeeze()
            twist.angular.x = 0.
            twist.angular.y = 0.
            twist.angular.z = 0.

            # Build trajectory point
            traj_point = MultiDOFJointTrajectoryPoint()
            traj_point.transforms = [transform]
            traj_point.velocities = [twist]
            traj_point.time_from_start = rospy.Duration(self.cfg.dt * i, 0)

            # append to trajectory points
            trajectory.points.append(traj_point)

        return trajectory

    def step(self):
        # run optimization
        _ = self.mpc.make_step(self.mpc.x0).squeeze()
        # retrieve and publish trajecotry
        trajectory = self.retrieve_trajectory()
        self.ee_trajectory_pub.publish(trajectory)

    
    def apply_gpt_message(self, optimization: Optimization, observation: Dict[str, np.ndarray]) -> None:
        # init mpc newly
        self.init_mpc()
        # apply constraint function
        # NOTE use 1e-6 when doing task L 
        regulatization = 0#1 * ca.norm_2(self.dpsi)**2 #+ 0.1 * ca.norm_2(self.psi - np.pi/2)**2
        self.set_objective(self._eval(optimization.objective) + regulatization)
        # set base constraint functions
        constraints = []
        # positive equality constraint
        constraints += [self._eval(c) for c in optimization.equality_constraints]
        # negative equality constraint
        constraints += [-self._eval(c) for c in optimization.equality_constraints]
        # inequality constraints
        inequality_constraints = [[*map(lambda const: self._eval(c, const), self.gripper_offsets)] for c in optimization.inequality_constraints]
        constraints += list(chain(*inequality_constraints))
        # set constraints
        self.set_constraints(constraints)
        # setup
        self.mpc.set_uncertainty_values(t=np.array([0.])) # TODO this is badly harcoded
        self.mpc.setup()
        self.mpc.set_initial_guess()
        return
    

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # publish current pose
            self.step()
            rate.sleep()



if __name__ == "__main__":
    controller = Controller()
    controller.run()
