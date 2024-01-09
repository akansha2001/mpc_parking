import importlib.util
import do_mpc
from casadi import *
import numpy as np
from trajectory import Trajectory
        
class MPC:
    def __init__(self,
                 trajectory: Trajectory,
                #  vehicle,
                 n_horizon=10,
                 t_step=0.1,
                 n_robust=1,
                 r=1e-5,
                 model_type='continuous',
                 max_vel=4.0,
                 max_phi = 0.4):  # TODO: check discrete too

        self.model = do_mpc.model.Model(model_type)
        self.n_horizon = n_horizon
        self.t_step = t_step
        self.n_robust = n_robust
        self.r = r
        self.lr = 0.494/2
        self.L = 0.494
        self.buffer = 1.0
        self.model_type = model_type
        self.trajectory = trajectory
        self.look_ahead_time = 2.0
        self.max_vel = max_vel
        self.max_phi = max_phi
        self.counter = 1
        self.K_pos = 1


    def setup_model(self, model_type):
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

        # # define time varying parameters (velocity obstacles)
        # car_radius = L/2
        # centers_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(positions),1))
        # radii_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(radii),1))
        # set right hand side equation

        self.beta = np.arctan(self.lr*np.tan(self.delta)/self.L)
        self.model.set_rhs('x', self.v*np.cos(self.yaw + self.beta))
        self.model.set_rhs('y', self.v*np.sin(self.yaw + self.beta))
        self.model.set_rhs('yaw', self.v*np.tan(self.delta)*np.cos(self.beta)/self.L)
        self.model.set_rhs('delta', self.phi)

         # setup model
        self.model.setup()

        # configure the MPC controller
        self.mpc = do_mpc.controller.MPC(model=self.model)
        self.mpc.settings.supress_ipopt_output()
        # optimizer parameters
        setup_mpc = {
            'n_horizon': self.n_horizon,
            't_step': self.t_step,
            'n_robust': self.n_robust,
            'store_full_solution': True,
        }

        self.mpc.set_param(**setup_mpc)

        # constraints
        # bounds
        # SET STATE BOUNDS FOR x,y,yaw,delta: HARDCODED
        self.mpc.bounds['lower', '_x', 'delta'] = -np.pi/4
        self.mpc.bounds['upper', '_x', 'delta'] = np.pi/4
        self.mpc.bounds['lower', '_x', 'yaw'] = -2*np.pi
        self.mpc.bounds['upper', '_x', 'yaw'] = 2*np.pi

        # SET MIN R
        self.R_min = self.L/np.tan(np.pi/6) 

        # SET INPUT BOUNDS FOR v,phi: HARDCODED
        self.mpc.bounds['lower', '_u', 'v'] = 0.
        self.mpc.bounds['upper', '_u', 'v'] = self.max_vel
        self.mpc.bounds['lower', '_u', 'phi'] = -self.max_phi
        self.mpc.bounds['upper', '_u', 'phi'] = self.max_phi


    def plan(self, robot): # CHANGED FUNCTION DEFINITION TOO

        state = robot.state
        centers_obstacles = robot.extract_obstacles(self.L + self.buffer)

        # extracting necessary state variables
        u = state.get_forward_velocity()
        yaw = state.get_yaw()
        rear_x = state.get_rear_x()
        rear_y = state.get_rear_y()

        # finding the look ahead index from the trajectory
        if u < 0.5:
            look_ahead_dist = 1.0
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
        #print(tx, ty)
            
        x_target = np.array([tx,ty,self.trajectory.ct[ind]])
        self.setup_model(model_type=self.model_type)
        # objective function
        alpha = np.arctan2(x_target[1] - rear_y, x_target[0] - rear_x) - yaw
        ld = np.linalg.norm(x_target[:2]-state.position[:2],ord=2)
        delta_ref = np.arctan(2*self.L*np.sin(alpha)/ld)

        # define weights for costs
        # HARDCODED
        # TODO: tune

        K_yaw = 1.0
        K_delta = 0.0

        w_track = 1.0
        w_progress = 0.0
        w_avoid = 0.0 # try 16

        # HARDCODED
        v_ref = 4.0
        # costs
        # TODO: saturate
        J_track = self.K_pos*((x_target[0] - self.x)**2 + (x_target[1] - self.y)**2) + \
                 K_yaw*(x_target[2] - self.yaw)**2 + K_delta*(delta_ref - self.delta)
        J_progress = (self.v- v_ref)**2
        # J_avoid = 1.0**2 - (self.x - 5)**2 - (self.y - 0.1)**2
        J_avoid = 0.0
        mterm = w_track*J_track +  w_avoid * J_avoid
        lterm = mterm + w_progress*J_progress 

        # TODO: ensure that delta can be used in the objective function
        # getting the optimal step
        x0 = np.append(state.position, state.steering)
        self.mpc.x0 = x0
        
        # mterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        # lterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        # penalty for control inputs
        self.mpc.set_rterm(v= self.r, phi=self.r)    

        if centers_obstacles is not None:
            for i,center in enumerate(centers_obstacles):
                self.mpc.set_nl_cons(expr_name="obstacle"+str(i+1),expr=(self.L + self.buffer)**2 - (self.x - center[0])**2 - (self.y - center[1])**2,ub=0)
        
        # setup mpc
        self.mpc.setup()
        self.mpc.set_initial_guess()
        # configure simulator
        simulator = do_mpc.simulator.Simulator(self.model)
        # params for simulator
        simulator.set_param(t_step=self.t_step)
        
        u0 = self.mpc.make_step(x0) 
        u0 = u0.ravel()
        print("\n\ncmd: ", u0)
        return u0