import numpy as np
from abc import ABC, abstractmethod
from trajectory import Trajectory
import importlib.util
import do_mpc
from casadi import *
import numpy as np
from trajectory import Trajectory
import datetime
from helper import bcolors

class LocalPlanner(ABC):
    @abstractmethod
    def plan(self, robot):
        pass


class MPC:
    def __init__(self,
                 trajectory: Trajectory,
                 # vehicle,  # note: uncomment this line if 'vehicle' is intended to be a parameter
                 n_horizon=10,  # number of prediction steps in the horizon
                 t_step=0.1,  # time step between each prediction step
                 n_robust=1,  # number of robustness constraints
                 r=1e-5,  # regularization term
                 model_type='continuous',  # type of model (e.g., 'continuous' or 'discrete')
                 max_vel=2,  # maximum linear velocity
                 max_phi=0.8):  # maximum steering angle

        # initialize MPC parameters
        self.n_horizon = n_horizon
        self.t_step = t_step
        self.n_robust = n_robust
        self.max_vel = max_vel
        self.max_phi = max_phi
        self.r = r
        self.lr = 0.9 / 2  # half of the wheelbase
        self.L = 0.9  # wheelbase
        self.delta_max = np.pi/3

        self.buffer = 0.8  # buffer distance
        self.look_ahead_time = 1.0  # look-ahead time for planning
        self.K_pos = 0.5  # proportional gain for position control
        self.K_yaw = 0.1  # proportional gain for yaw control
        self.K_delta = 0.3  # proportional gain for steering control
        self.w_track = 1.0  # weight for tracking error
        self.w_progress = 0.0  # weight for tracking progress
        self.w_avoid = 0.0  # weight for obstacle avoidance
        self.min_dist = np.inf  # minimum distance (initialized as infinity)
        self.trajectory = trajectory  # reference trajectory for the MPC

        self.model_type = model_type  # type of model (e.g., 'continuous' or 'discrete')
        self.model = do_mpc.model.Model(model_type)
        self.start_t  = datetime.datetime.now()

    def setup_model(self, model_type):
        # initialize the MPC model
        self.model = do_mpc.model.Model(model_type)

        # define model variables
        self.x = self.model.set_variable(
            var_type='_x', var_name='x', shape=(1, 1))
        self.y = self.model.set_variable(
            var_type='_x', var_name='y', shape=(1, 1))
        self.yaw = self.model.set_variable(
            var_type='_x', var_name='yaw', shape=(1, 1))
        self.delta = self.model.set_variable(
            var_type='_x', var_name='delta', shape=(1, 1))

        # define inputs
        self.v = self.model.set_variable(var_type='_u', var_name='v')
        self.phi = self.model.set_variable(var_type='_u', var_name='phi')

        # calculate beta
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.L)

        # set model right-hand side equations
        self.model.set_rhs('x', self.v * np.cos(self.yaw + self.beta))
        self.model.set_rhs('y', self.v * np.sin(self.yaw + self.beta))
        self.model.set_rhs('yaw', self.v * np.tan(self.delta)
                        * np.cos(self.beta) / self.L)
        self.model.set_rhs('delta', self.phi)

        # setup the model
        self.model.setup()

        # configure the MPC controller
        self.mpc = do_mpc.controller.MPC(model=self.model)
        # self.mpc.settings.suppress_ipopt_output()

        # optimizer parameters
        setup_mpc = {
            'n_horizon': self.n_horizon,
            't_step': self.t_step,
            'n_robust': self.n_robust,
            'store_full_solution': True,
        }

        self.mpc.set_param(**setup_mpc)
        
        # set state bounds for x, y, yaw, delta
        self.mpc.bounds['lower', '_x', 'delta'] = -self.delta_max
        self.mpc.bounds['upper', '_x', 'delta'] = self.delta_max
        self.mpc.bounds['lower', '_x', 'yaw'] = -2*np.pi
        self.mpc.bounds['upper', '_x', 'yaw'] = 2*np.pi

        # set min R
        self.R_min = self.L / np.tan(np.pi/6)

        # set input bounds for v, phi
        self.mpc.bounds['lower', '_u', 'v'] = 0.
        self.mpc.bounds['upper', '_u', 'v'] = self.max_vel
        self.mpc.bounds['lower', '_u', 'phi'] = -self.max_phi
        self.mpc.bounds['upper', '_u', 'phi'] = self.max_phi

    def plan(self, robot):

        state = robot.state
        centers_obstacles = robot.extract_obstacles(self.L + self.buffer)

        # extracting necessary state variables
        u = state.get_forward_velocity()
        yaw = state.get_yaw()
        rear_x = state.get_rear_x()
        rear_y = state.get_rear_y()

        # finding the look ahead index from the trajectory
        if u < 0.5:
            look_ahead_dist = 0.5
        else:
            look_ahead_dist = u * self.look_ahead_time
        # print(look_ahead_dist)

        # finding the closest index on the trajectory
        dx = self.trajectory.cx - rear_x
        dy = self.trajectory.cy - rear_y

        closest_index = np.argmin(np.hypot(dx, dy))

        # closest index on the trajectory
        ind = closest_index
        while look_ahead_dist > \
                state.distance_from_rear_axle(self.trajectory.cx[ind], self.trajectory.cy[ind]):
            if (ind + 1) >= len(self.trajectory.cx):
                break  # not exceed goal
            ind += 1

        # index normalization making sure we don't overshoot the goal.
        if ind < len(self.trajectory.cx):
            tx = self.trajectory.cx[ind]
            ty = self.trajectory.cy[ind]
        else:  # toward goal
            tx = self.trajectory.cx[-1]
            ty = self.trajectory.cy[-1]
            ind = len(self.trajectory.cx) - 1
        # print(tx, ty)

        x_target = np.array([tx, ty, self.trajectory.ct[ind]])
        self.setup_model(model_type=self.model_type)
        # objective function
        alpha = np.arctan2(x_target[1] - rear_y, x_target[0] - rear_x) - yaw
        ld = np.linalg.norm(x_target[:2]-state.position[:2], ord=2)
        delta_ref = np.arctan(2*self.L*np.sin(alpha)/ld)

        # define weights for costs
        v_ref = self.max_vel
        # costs
        J_track = self.K_pos*((x_target[0] - self.x)**2 + (x_target[1] - self.y)**2) + \
            self.K_yaw*(x_target[2] - self.yaw)**2 + \
            self.K_delta*(delta_ref - self.delta)
        J_progress = (self.v - v_ref)**2
        # J_avoid = 1.0**2 - (self.x - 5)**2 - (self.y - 0.1)**2
        J_avoid = 0.0
        mterm = self.w_track*J_track + self.w_avoid*J_avoid
        lterm = mterm + self.w_progress*J_progress

        # getting the optimal step
        x0 = np.append(state.position, state.steering)
        self.mpc.x0 = x0

        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        # penalty for control inputs
        self.mpc.set_rterm(v=self.r, phi=self.r)

        if centers_obstacles is not None:
            for i, center in enumerate(centers_obstacles):
                self.mpc.set_nl_cons(expr_name="obstacle"+str(i+1), expr=(
                    self.L + self.buffer)**2 - (self.x - center[0])**2 - (self.y - center[1])**2, ub=0)
                dist = np.linalg.norm(x0[0:2]-center, ord=2) - self.L/2 - 0.4
                if dist < self.min_dist:
                    self.min_dist = dist
                    print(self.min_dist)
                # self.min_dist = np.min(dist, self.min_dist) - self.L/2
                # print("################## MIN DIST:", self.min_dist)
        # setup mpc
        self.mpc.setup()
        self.mpc.set_initial_guess()
        # configure simulator
        simulator = do_mpc.simulator.Simulator(self.model)
        # params for simulator
        simulator.set_param(t_step=self.t_step)

        start = datetime.datetime.now()         # current time
        # solving the optimization problem
        u0 = self.mpc.make_step(x0)
        end = datetime.datetime.now()           # current time
        delta = end - start
        print(bcolors.OKGREEN + "\nMPC plan time:",
              delta.total_seconds(), "s" + bcolors.ENDC)
        total_duration = end - self.start_t
        print(bcolors.OKGREEN + "\nDuration:", total_duration.total_seconds(), "s" + bcolors.ENDC)
        u0 = u0.ravel()
        return u0
class PurePursuit(LocalPlanner):
    def __init__(self, trajectory: Trajectory, max_vel = 5):
        self.trajectory = trajectory
        #print("Pure pursuit input trajectory",trajectory) 
        self.max_vel = max_vel        
        self.look_ahead_time = 1.
        self.speed_factor =  0.30
        self.look_ahead_thresh = 0.1
        self.stop_thresh = 0.1
        self.Kp = 3.

    def plan(self, robot):
        state = robot.state
        # extracting necessary state variables
        u = state.get_forward_velocity()
        yaw = state.get_yaw()
        rear_x = state.get_rear_x()
        rear_y = state.get_rear_y()

        # finding the look ahead index from the trajectory
        look_ahead_dist = u * self.look_ahead_time
        
        # finding the closest index on the trajectory
        dx = self.trajectory.cx - rear_x
        dy = self.trajectory.cy - rear_y
        
        closest_index = np.argmin(np.hypot(dx, dy))
        
        # closest index on the trajectory
        ind = closest_index
        while look_ahead_dist > \
                state.distance_from_rear_axle(self.trajectory.cx[ind], self.trajectory.cy[ind]):
            if (ind + 1) >= len(self.trajectory.cx):
                break  # not exceed goal
            ind += 1
        
        # index normalization making sure we don't overshoot the goal.
        if ind < len(self.trajectory.cx):
            tx = self.trajectory.cx[ind]
            ty = self.trajectory.cy[ind]
        else:  # toward goal
            tx = self.trajectory.cx[-1]
            ty = self.trajectory.cy[-1]
            ind = len(self.trajectory.cx) - 1
        #print(tx, ty)
        
        # Calculate the slope of look ahead distance line which would be alpha.
        # If the car was heading along the velocity vector then that would be it but the 
        # car has a yaw from the heading vector and hence we need to subtract this
        # from slope to get actual alpha
        alpha = np.arctan2(ty - rear_y, tx - rear_x) - yaw

        # checking whether lookahead distance is large enough
        # if look_ahead_dist > self.look_ahead_thresh:
            # steering angle = invtan(2Lsin(alpha)/Ld)
            # as established steering control law for pure pursuit controller
        delta = np.arctan2(2.0 * state.L * np.sin(alpha), look_ahead_dist)
        # else:
            #print("warning: small lookahead distance!")
            # delta = state.steering  # no steering input needed
        
        cmd_omega =  self.Kp * float(delta - state.steering[0])
        goal_dist = state.get_distance(self.trajectory.cx[-1], self.trajectory.cy[-1])
        
        if goal_dist > self.stop_thresh:
            cmd_vel = self.speed_factor * self.max_vel
            self.trajectory.cx = self.trajectory.cx[closest_index:]
            self.trajectory.cy = self.trajectory.cy[closest_index:]
        else:
            cmd_vel = 0
            cmd_omega = 0        
        
        cmd = np.array([cmd_vel, cmd_omega])
        return cmd
        # return np.array([cmd_vel, 0.1])

class DummyLocalPlanner(LocalPlanner):
    def __init__(self):
        super().__init__()

    def plan(self, robot):
        # returns zero velocities
        cmd = np.array([0.0, 0.0])
        return cmd
