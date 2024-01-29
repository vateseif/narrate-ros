#!/usr/bin/env python3
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))

import rospy
import tf.transformations
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from l2c.srv import GetEEState


import do_mpc
import numpy as np
import casadi as ca
from time import sleep
from math import pi, sin
from itertools import chain
from typing import Dict, List, Optional, Tuple


from llm import Optimization
from config import ControllerConfig

class Controller:
    def __init__(self, env_info:Tuple[List]=([{"name":"robot"}], [{"name":"cube"}]), cfg=ControllerConfig) -> None:
        self.cfg = cfg

        # init info of robots and objects
        self.robots_info, self.objects_info = env_info

        # init ros node
        self.init_node()

        # init model dynamics
        self.init_model()

        # init controller
        self.init_controller()

        # init variables and expressions
        self.init_expressions()

        # gripper fingers offset for constraints 
        self.gripper_offsets = [np.array([0., -0.048, 0.]), np.array([0., 0.048, 0.]), np.array([0., 0., 0.048])]

    def init_node(self):
        rospy.init_node('controller', anonymous=True)
        rospy.wait_for_service('get_x0') # wait for service to be available
        # subs
        self.get_x0 = rospy.ServiceProxy('get_x0', GetEEState)

        self.target_pose_publisher = rospy.Publisher('/target_pose', Pose, queue_size=10)
        

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
        
        """ State and input variables """
        #self.x = []       # gripper position (x,y,z)
        #self.psi = []     # gripper psi (rotation around z axis)
        #self.dx = []      # gripper velocity (vx, vy, vz)
        #self.dpsi = []    # gripper rotational speed
        #self.u = []       # gripper control (=velocity)
        #self.u_psi = []   # gripper rotation control (=rotational velocity)
        
        
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
        #regularization += ca.norm_2(self.x[i] - r['x0'])**2
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

    def set_pose(self, pose):
        # TODO: use the pose to initialize x0
        pass

    def set_x0(self, observation: Dict[str, np.ndarray]):
        x0 = []
        self.pose = []
        #for r in self.robots_info: # TODO set names instead of robot_0 in panda
        obs = observation['robot'] # observation of each robot
        x = obs[:3]
        psi = np.array([obs[5]])
        dx = obs[6:9]
        x0 = np.concatenate((x, dx)) # TODO dpsi is harcoded to 0 here
        self.pose = obs[:6]
        # set x0 in MPC
        self.mpc.x0 = x0

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

    def _eval(self, code_str: str, observation: Dict[str, np.ndarray], offset=np.zeros(3)):
        #TODO the offset is still harcoded
        # put together variables for python code evaluation:    
        
        # initial state of robots before applying any action
        x0 = {'x0': observation['robot'][:3]} 
        # robot variable states (decision variables in the optimization problem)
        robots_states = {}
        #for i, r in enumerate(self.robots_info):
        robots_states['x'] = self.x + self.R@offset
        robots_states['dx'] = self.dx
        
        eval_variables = {**self.eval_variables, **robots_states, **self.objects, **x0}
        # evaluate code
        evaluated_code = eval(code_str, eval_variables)
        return evaluated_code

    def _solve(self) -> List[np.ndarray]:
        """ Returns a list of controls, 1 for each robot """
        # solve mpc at state x0
        u0 = self.mpc.make_step(self.mpc.x0).squeeze()
        # compute action for each robot
        action = []
        #for i in range(len(self.robots_info)):
        ee_displacement = u0[0:3]     # positon control
        """
        theta_regularized = self.pose[i][3] if self.pose[i][3]>=0 else self.pose[i][3] + 2*np.pi 
        theta_rotation = [(np.pi - theta_regularized)*1.5]
        gamma_rotation = [-self.pose[i][4] * 1.5]  # P control for angle around y axis # TODO: 1. is a hardcoded gain
        psi_rotation = [u0[4*i+3]]            # rotation control
        """
        #action = np.concatenate((ee_displacement, theta_rotation, gamma_rotation, psi_rotation))
        
        return u0

    def step(self):

        t = rospy.Time.now().to_sec()
        euler_displacement = (0., 0., 0.)
        position_displacement = Point(0.0, 0.0, 0.1*sin(t))
        quaternion_displacement = Quaternion(*tf.transformations.quaternion_from_euler(*euler_displacement))
        
        pose_displacement = Pose(position_displacement, quaternion_displacement)

        self.target_pose_publisher.publish(pose_displacement)

        """
        if not self.mpc.flags['setup']:
            return [np.zeros(6) for i in range(len(self.robots_info))]  # TODO 6 is hard-coded here
        return self._solve()
        """
    
    def apply_gpt_message(self, optimization: Optimization, observation: Dict[str, np.ndarray]) -> None:
        # init mpc newly
        self.init_mpc()
        # apply constraint function
        # NOTE use 1e-6 when doing task L 
        regulatization = 0#1 * ca.norm_2(self.dpsi)**2 #+ 0.1 * ca.norm_2(self.psi - np.pi/2)**2
        self.set_objective(self._eval(optimization.objective, observation) + regulatization)
        # set base constraint functions
        constraints = []
        # positive equality constraint
        constraints += [self._eval(c, observation) for c in optimization.equality_constraints]
        # negative equality constraint
        constraints += [-self._eval(c, observation) for c in optimization.equality_constraints]
        # inequality constraints
        inequality_constraints = [[*map(lambda const: self._eval(c, observation, const), self.gripper_offsets)] for c in optimization.inequality_constraints]
        constraints += list(chain(*inequality_constraints))
        # set constraints
        self.set_constraints(constraints)
        # setup
        self.mpc.set_uncertainty_values(t=np.array([0.])) # TODO this is badly harcoded
        self.mpc.setup()
        self.mpc.set_initial_guess()
        return
    

    def run(self):
        rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            # publish current pose
            x0 = self.get_x0()
            rospy.loginfo(x0)
            self.step()
            rate.sleep()



if __name__ == "__main__":
    controller = Controller()
    #controller.set_x0({
    #    "robot": np.zeros((12,))
    #})
    #print(controller._solve())
    controller.run()
