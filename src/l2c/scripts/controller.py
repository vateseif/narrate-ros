#!/usr/bin/env python3
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
#sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))

import rospy
import tf.transformations as tft
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import Twist, Transform, PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


import do_mpc
import time
import numpy as np
import casadi as ca
from itertools import chain
from typing import Dict, List, Optional, Tuple

from dynamic_reconfigure.server import Server
from l2c.cfg import ControllerConfig
from l2c.msg import Optimization, NamedPoseArrayStamped

FREQUENCY_VIZ = 5.0  # ros
PUBLISH_VIZ_MARKERS = True

USE_OUTLIER_DETECTION = True
N_OLD_POSES = 5

BASE_LINK_NAME = "panda_link0"

def wrap_psi(psi):
    # NOTE: psi is wrapped this way because of the Franka's EE joint limits and the 90deg 
    # rotational equivariance of the cubes.
    """Subtract or add pi/2 to an angle until it is within the range [0, pi/2]"""
    psi = psi % (2 * np.pi)
    while (psi > np.pi / 2) or (psi < 0):
        if psi > np.pi / 2:
            psi -= np.pi/2
        elif psi < 0:
            psi += np.pi/2
    return psi


class Object:
	def __init__(self, name:str, position:np.ndarray=np.zeros((3,1)), psi:float=0., size:float=0.) -> None:
		self.name = name
		self.position = position
		self.psi = psi
		self.size = size


class Controller:
    def __init__(self, env_info:Tuple[List]=([{"name":"robot"}], [{"name":"cube"}])) -> None:
        self.READY = False
        self.is_gripper_closed = False
        self.init_node()
        
        # init info of robots and objects
        self.robots_info, self.objects_info = env_info       

        self._base_frame = BASE_LINK_NAME

        self._trajectory = []

        self.init_model()

        self.init_controller()

        self.init_expressions()

        self.home_position = np.array([0.3, 0., 0.5])
        self.home_orientation = np.array([0., 0., 0.])
        self.home_quaternion = np.array([1., 0., 0., 0.])


        # collision spheres for Panda Hand
        self.gripper_offsets = [(np.array([0., -0.048, 0.001]), 0.0125), (np.array([0., 0.048, 0.001]), 0.0125),
                                (np.array([0., -0.06, 0.03]), 0.0125), (np.array([0., 0.06, 0.03]), 0.0125),
                            (np.array([0., 0.09, 0.067]), 0.03), (np.array([0., -0.09, 0.067]), 0.03),
                            (np.array([0., 0.09/2, 0.067]), 0.03), (np.array([0., -0.09/2, 0.067]), 0.03),
                            (np.array([0., 0.0, 0.067]), 0.03)
                            ]
        self.gripper_offsets_load = [(np.array([0., 0., 0.]), 0.025)]

        self.optimization_objective_str = None
        self.previous_cost = float('inf')
        self.improvement = float('inf')
        self.latest_pose_target = None
        self.prev_pose_msg = None
        self.objects_dict = {}
        self.mpc_already_converged = False
        self.received_first_optimization_message = False
        self.mpc_done_published = False

        self.old_poses = []

        if PUBLISH_VIZ_MARKERS:
            self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.solvetime_publisher = rospy.Publisher('/controller_solvetime', String, queue_size=10)


    def update_config(self, config, level):
        print("Updating config")
        self.cfg = config
        return config
    
    def get_gripper_offsets(self,):
        if self.is_gripper_closed:
            gripper_offset = self.gripper_offsets + self.gripper_offsets_load
        else:
            gripper_offset = self.gripper_offsets
        return gripper_offset

    def pub_viz_collision_spheres_gripper(self,):
        gripper_offsets = self.get_gripper_offsets()
        
        for i, gripper_offset in enumerate(gripper_offsets):
            marker = Marker()
            marker.header.frame_id = 'panda_EE'
            marker.header.stamp = rospy.Time.now()
            marker.ns = "gripper_spheres"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = gripper_offset[0][0]
            marker.pose.position.y = -gripper_offset[0][1]
            marker.pose.position.z = -gripper_offset[0][2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * gripper_offset[1]
            marker.scale.y = 2 * gripper_offset[1]
            marker.scale.z = 2 * gripper_offset[1]
            marker.color.a = 0.7
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0            
            self.marker_publisher.publish(marker)

    def init_node(self):
        # node
        rospy.init_node('controller', anonymous=True)
        self.srv = Server(ControllerConfig, self.update_config)
        # subs
        self.ee_pose_sub = Subscriber('/ee_pose', PoseStamped)
        self.ee_twist_sub = Subscriber('/ee_twist', TwistStamped)
        self.joint_states_sub = Subscriber('/joint_states', JointState, callback=self.joint_states_callback, queue_size=1)
        self.cubes_pose_sub = rospy.Subscriber('/perception/cubes/pose', NamedPoseArrayStamped, self.cubes_pose_callback, queue_size=1)
        self.synced_subs = ApproximateTimeSynchronizer([self.ee_pose_sub, self.ee_twist_sub],
                                                       queue_size=1,
                                                       slop=0.01,
                                                       allow_headerless=False)
        self.synced_subs.registerCallback(self.set_x0) # sync subs
        self.OD_message_sub = rospy.Subscriber('/optimization', Optimization, self.apply_gpt_message, queue_size=1)
        self.manual_opt_trigger_sub = rospy.Subscriber('/manual_optimization_trigger', Empty, self.manual_opt_trigger_callback, queue_size=1)
        # pubs
        self.vis_trajectory_pub = rospy.Publisher("/planned_trajectory", MarkerArray, queue_size=1)
        self.optimization_trigger_pub = rospy.Publisher('/optimization_trigger', Empty, queue_size=1)
        self.ee_trajectory_pub = rospy.Publisher('/ee_trajectory', MultiDOFJointTrajectory, queue_size=10)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
        self.mpc_done_pub = rospy.Publisher('/optimization_trigger', Empty, queue_size=1)


    def init_model(self):
        self.model = do_mpc.model.Model(self.cfg["model_type"])
        self.t = self.model.set_variable('parameter', 't')

        self.objects = {} 
        for o in self.objects_info:
            position = self.model.set_variable(var_type='_p', var_name=o['name']+'_position', shape=(3,1))
            psi = self.model.set_variable(var_type='_p', var_name=o['name']+'_psi')
            size = self.model.set_variable(var_type='_p', var_name=o['name']+'_size')
            obj = Object(o['name'], position, psi, size)
            self.objects[o['name']] = obj

        self.x0 = {}
        self.pose = []

        # position (x, y, z)
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(self.cfg["nx"],1))
        self.psi = self.model.set_variable(var_type='_x', var_name='psi', shape=(1,1))
        self.dx = self.model.set_variable(var_type='_x', var_name='dx', shape=(self.cfg["nx"],1))
        self.dpsi = self.model.set_variable(var_type='_x', var_name='dpsi', shape=(1,1))
        self.u = self.model.set_variable(var_type='_u', var_name='u', shape=(self.cfg["nu"],1))
        self.u_psi = self.model.set_variable(var_type='_u', var_name='u_psi', shape=(1,1))
        # system dynamics
        self.model.set_rhs('x', self.x + self.dx / self.cfg["frequency"])
        self.model.set_rhs('psi', self.psi + self.dpsi / self.cfg["frequency"])
        self.model.set_rhs('dx', self.u)
        self.model.set_rhs('dpsi', self.u_psi)

        self.model.setup()

    def set_objective(self, mterm: ca.SX = ca.DM([[0]])):
        regularization = 0
        regularization += 1e-4 * ca.norm_2(self.dx)**2
        regularization += .0002 * ca.norm_2(ca.sin(self.psi) * ca.cos(self.psi))**2  # make the gripper prefer to be either at 0 or pi/2
        
        mterm = mterm + regularization
        lterm = mterm / 2
        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        u_kwargs = {'u':1e-1, 'u_psi':1e-4}
        self.mpc.set_rterm(**u_kwargs)

    def set_constraints(self, nlp_constraints: Optional[List[ca.SX]] = None):

        self.mpc.bounds['lower','_x', 'x'] = np.array([0.0, -0.75, 0.0])
        self.mpc.bounds['upper','_x', 'x'] = np.array([0.75, 0.75, 1.0])
        self.mpc.bounds['upper','_x', 'psi'] = np.pi * 0.55 * np.ones((1, 1))   # rotation upper bound
        self.mpc.bounds['lower','_x', 'psi'] = -np.pi * 0.55 * np.ones((1, 1))  # rotation lower bound

        self.mpc.bounds['upper','_u', 'u'] = self.cfg["hu"] * np.ones((self.cfg["nu"], 1))  # input upper bound
        self.mpc.bounds['lower','_u', 'u'] = self.cfg["lu"] * np.ones((self.cfg["nu"], 1))  # input lower bound
        self.mpc.bounds['upper','_u', 'u_psi'] = self.cfg["hu_psi"] * np.ones((1, 1))   # input upper bound
        self.mpc.bounds['lower','_u', 'u_psi'] = self.cfg["lu_psi"] * np.ones((1, 1))  # input lower bound

        if nlp_constraints == None:
            return

        for i, constraint in enumerate(nlp_constraints):
            self.mpc.set_nl_cons(f'const{i}', expr=constraint, ub=0.,
                                soft_constraint=True,
                                penalty_term_cons=self.cfg["penalty_term_cons"])

    def init_mpc(self):
        self.mpc = do_mpc.controller.MPC(self.model)
        setup_mpc = {'n_horizon': self.cfg["T"], 't_step': 1.0 / self.cfg["frequency"], 'store_full_solution': False}
        self.mpc.set_param(**setup_mpc)
        self.mpc.settings.supress_ipopt_output() # => verbose = False

    def init_controller(self):
        self.init_mpc()
        self.set_objective()
        self.set_constraints()
        self.mpc.set_uncertainty_values(t=np.array([0.]))
        self.mpc.setup()
        self.mpc.set_initial_guess()

    def init_expressions(self):
        self.eval_variables = {"ca":ca, "np":np, "t":self.t} # python packages
        self.R = np.array([[ca.cos(self.psi), -ca.sin(self.psi), 0],
                            [ca.sin(self.psi), ca.cos(self.psi), 0],
                            [0, 0, 1.]])

    def set_t(self, t:float):
        """ Update the simulation time of the MPC controller"""
        self.mpc.set_uncertainty_values(t=np.array([t]))

    def joint_states_callback(self, joint_states:JointState):
        j = np.array(joint_states.position)
        gripper_spacing = j[-2:].sum()
        self.is_gripper_closed = gripper_spacing < self.cfg.gripper_spacing_closed_threshold

    def set_x0(self, ee_pose:PoseStamped, ee_twist:TwistStamped):
        if not self.READY: self.READY=True
        x = np.array([ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z])
        dx = np.array([ee_twist.twist.linear.x, ee_twist.twist.linear.y, ee_twist.twist.linear.z])
        quaternion = np.array([ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z, ee_pose.pose.orientation.w])
        euler = tft.euler_from_quaternion(quaternion, axes='sxyz')
        psi = np.array([euler[2]])
        psi = wrap_psi(psi)

        self.home_position = x
        self.home_orientation = euler
        self.home_quaternion = quaternion
        self.mpc.x0 = np.concatenate((x, psi, dx, [0]))

    def manual_opt_trigger_callback(self, msg:Empty):
        self.mpc_already_converged = True

    def cubes_pose_callback(self, cubes_pose:NamedPoseArrayStamped):
        parameters = {'t': [rospy.Time.now().to_sec()]}
        for named_pose in cubes_pose.named_poses:
            name = named_pose.name
            xyz = np.array([named_pose.pose.position.x, named_pose.pose.position.y, named_pose.pose.position.z])
            parameters[name+'_position'] = [xyz]

            quaternion = (named_pose.pose.orientation.x, named_pose.pose.orientation.y, named_pose.pose.orientation.z, named_pose.pose.orientation.w)
            euler = tft.euler_from_quaternion(quaternion, axes='sxyz')
            psi = euler[2]
            psi = wrap_psi(psi)
            parameters[name+'_psi'] = np.array([psi])
            parameters[name+'_size'] = np.array([0.05]) # TODO: hardcoded size

        self.objects_dict = parameters
        self.mpc.set_uncertainty_values(**parameters)

    def reset(self, observation: Dict[str, np.ndarray], t:float = 0) -> None:
        """
        observation: robot observation from simulation containing position, angle and velocities
        """
        self.init_states(observation, t)
        return

    def _eval(self, code_str: str, offset=(np.zeros(3), 0.)):
        collision_xyz, collision_radius = offset
        x0 = {'x0': self.home_position, 'psi0': self.home_orientation[2]}
        robots_states = {}
        robots_states['x'] = self.x + self.R@collision_xyz
        robots_states['dx'] = self.dx
        robots_states['psi'] = self.psi

        eval_variables = {**self.eval_variables, **robots_states, **self.objects, **x0}
        evaluated_code = eval(code_str, eval_variables) + collision_radius
        return evaluated_code


    def retrieve_trajectory(self):
        self._trajectory = [] # for visualization
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = ["end_effector"]  # Replace with your actual joint/frame names
        trajectory_point = MultiDOFJointTrajectoryPoint()


        for i in range(len(self.mpc.opt_x_num['_x', :, 0, 0])):
            transform = Transform()
            x = self.mpc.opt_x_num['_x', :, 0, 0][i].toarray()
            transform.translation.x = x[0].squeeze()
            transform.translation.y = x[1].squeeze()
            transform.translation.z = x[2].squeeze()
            theta = -np.pi # x axis rotation
            gamma = 0. # y axis rotation
            psi = x[3].squeeze() # z axis rotation
            quaternion = tft.quaternion_from_euler(theta, gamma, psi)

            transform.rotation.x = quaternion[0]
            transform.rotation.y = quaternion[1]
            transform.rotation.z = quaternion[2]
            transform.rotation.w = quaternion[3]

            pos = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
            ori = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
            self._trajectory.append((pos, ori))

            twist = Twist()
            twist.linear.x = x[4].squeeze()
            twist.linear.y = x[5].squeeze()
            twist.linear.z = x[6].squeeze()
            twist.angular.x = 0.
            twist.angular.y = 0.
            twist.angular.z = 0.

            trajectory_point.transforms.append(transform)
            trajectory_point.velocities.append(twist)

        trajectory.points.append(trajectory_point)

        return trajectory
    
    def outlier_detection(self, pose_msg:PoseStamped):
        if len(self.old_poses) < N_OLD_POSES:
            self.old_poses.append(pose_msg)
            return True
        
        old_pose = np.array([0., 0., 0.])
        for p in self.old_poses:
            old_pose += np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
        old_pose /= len(self.old_poses)
        new_pose = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        distance = np.linalg.norm(old_pose - new_pose)
        if distance > self.cfg.outlier_detection_threshold:
            rospy.logwarn(f"Outlier detected. Distance: {distance}")
            return False
        
        self.old_poses.append(pose_msg)
        return True

    def pub_solvetime(self, solvetime:float):
        msg = String()
        msg.data = str(solvetime)
        self.solvetime_publisher.publish(msg)

    def step(self, event):
        if (not self.mpc.flags['setup']) or (not self.received_first_optimization_message):
            return
        st = time.time()
        _ = self.mpc.make_step(self.mpc.x0)
        et = time.time()
        try:
            trajectory = self.retrieve_trajectory()
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = BASE_LINK_NAME
            pose_msg.header.stamp = rospy.Time.now()
            transform = trajectory.points[0].transforms[self.cfg["delay_compensation_steps"]]
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = transform.translation.x, transform.translation.y, transform.translation.z
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w

            if USE_OUTLIER_DETECTION:
                acceptable = self.outlier_detection(pose_msg)
                if not acceptable:
                    return

            if self.mpc_already_converged:
                self.target_pose_pub.publish(self.prev_pose_msg)
            else:
                self.prev_pose_msg = pose_msg
                self.target_pose_pub.publish(pose_msg)
                self.ee_trajectory_pub.publish(trajectory)

            self.pub_solvetime(et-st)

        except Exception as e:
            rospy.logwarn(f"Trajectory couldn't be retrieved. {e}")
            return

    def vis_trajectory(self, event):
        all_points = MarkerArray()
        for i, pose in enumerate(self._trajectory):
            point = Marker()
            point.header.frame_id = self._base_frame
            point.header.stamp = rospy.Time.now()
            point.id = i
            point.type = Marker.ARROW
            point.action = Marker.ADD
            point.pose.position.x = pose[0][0]
            point.pose.position.y = pose[0][1]
            point.pose.position.z = pose[0][2]
            point.pose.orientation.x = pose[1][0]
            point.pose.orientation.y = pose[1][1]
            point.pose.orientation.z = pose[1][2]
            point.pose.orientation.w = pose[1][3]
            point.scale.x = 0.05
            point.scale.y = 0.01
            point.scale.z = 0.01
            point.color.a = 0.7
            point.color.r = 0.0
            point.color.g = 1.0
            point.color.b = 0.0
            all_points.markers.append(point)

        self.vis_trajectory_pub.publish(all_points)

        if PUBLISH_VIZ_MARKERS:
            self.pub_viz_collision_spheres_gripper()

    def reset_heuristics(self):
        self.previous_cost = float('inf')
        self.improvement = float('inf')
        self.optimization_objective_str = None
        self.mpc_already_converged = False
        self.mpc_done_published = False
        self.time_s = time.time()

    def update_relative_improvement(self, cost:float, ic_cost:float):
        cost = float(cost)
        ic_cost = float(ic_cost)
        self.improvement = abs((ic_cost - cost) / ic_cost)

    def eval_cost(self,):
        if self.optimization_objective_str is not None:
            try:
                _x = self.mpc.opt_x_num['_x', self.cfg["delay_compensation_steps"], 0, 0].toarray().flatten()
                _x0 = self.mpc.opt_x_num['_x', 0, 0, 0].toarray().flatten()
                x = _x[:3]
                x0 = _x0[:3]
                psi = _x[[3]]
                psi0 = _x0[[3]]
                eval_vars = {
                    **self.eval_variables,
                    **self.objects_dict,
                    'x': x,
                    'x0': x0,
                    'psi': psi,
                    'psi0': psi0,
                }
                cost = eval(
                    self.optimization_objective_str,
                    eval_vars
                )
                ic_eval_vars = {
                    **self.eval_variables,
                    **self.objects_dict,
                    'x': x0,
                    'x0': x0,
                    'psi': psi0,
                    'psi0': psi0,
                }
                ic_cost = eval(
                    self.optimization_objective_str,
                    ic_eval_vars
                )
                return float(cost), float(ic_cost)
            except Exception as e:
                rospy.logwarn(f"Cost couldn't be evaluated. {e}")
                return float('inf'), float('inf')
        else:
            return float('inf'), float('inf')

    def mpc_converged(self, cost):
        if self.mpc_already_converged:
            return True
        
        if self.cfg.termination_activate_heuristics:
            c1 = cost < self.cfg.termination_cost_thr
            c2 = (self.improvement < self.cfg.termination_rel_impr_thr) and (cost < self.cfg.termination_rel_impr_thr_max_cost)
            c3 = (time.time() - self.time_s) > self.cfg.termination_time_thr

            mpc_converged = False
            if self.cfg.termination_use_cost and c1:
                print(f"--- Cost threshold reached.")
                mpc_converged = True
            if self.cfg.termination_use_rel_impr and c2:
                print(f"--- Relative improvement threshold reached.")
                mpc_converged = True
            if self.cfg.termination_use_time and c3:
                print(f"--- Time threshold reached.")
                mpc_converged = True
            self.mpc_already_converged = mpc_converged
    
            if mpc_converged and not self.mpc_done_published:
                self.mpc_done_published = True
                self.mpc_done_pub.publish(Empty())
        
            return mpc_converged
        return False

    def apply_gpt_message(self, optimization: Optimization) -> None:
        self.received_first_optimization_message = True
        rospy.logwarn(f"{self.objects_dict=}")
        for name, el in self.objects_dict.items():
            if 'psi' in name:
                rospy.logwarn(f"{name}: {np.rad2deg(el)}")
        self.init_mpc()
        self.reset_heuristics()
        self.optimization_objective_str = optimization.objective
        regulatization = 0#1 * ca.norm_2(self.dpsi)**2 #+ 0.1 * ca.norm_2(self.psi - np.pi/2)**2
        objective = self._eval(optimization.objective) + regulatization
        self.set_objective(objective)
        constraints = []
        constraints += [self._eval(c) for c in optimization.equality_constraints]
        constraints += [-self._eval(c) for c in optimization.equality_constraints]
        gripper_offsets = self.get_gripper_offsets()
        inequality_constraints = [[*map(lambda const: self._eval(c, const), gripper_offsets)] for c in optimization.inequality_constraints]
        constraints += list(chain(*inequality_constraints))
        self.set_constraints(constraints)
        self.mpc.set_uncertainty_values(t=np.array([0.])) # TODO this is badly harcoded
        self.mpc.setup()
        self.mpc.set_initial_guess()
        print(f"Optimization problem applied to MPC controller.")
        return


    def run(self):
        print(f"Running MPC controller at (target) {self.cfg['frequency']} Hz")
        while not self.READY: rospy.sleep(1)
        # run mpc at 10Hz TODO: add to config
        rospy.Timer(rospy.Duration(1.0 / self.cfg['frequency']), self.step)
        # run trajectory visual at 5Hz TODO: add to config
        rospy.Timer(rospy.Duration(1.0 / FREQUENCY_VIZ), self.vis_trajectory)

        rospy.spin()

if __name__ == "__main__":
    # TODO: this is a hardcoded example, change it with the actual environment info
    env_info = (
        [{"name": "robot"}],
        [{"name": "red_cube"},
         {"name": "blue_cube"},
         {"name": "green_cube"},
         {"name": "yellow_cube"}]
    )
    controller = Controller(env_info)
    rospy.loginfo("MPC was succesfully setup!")
    controller.run()
