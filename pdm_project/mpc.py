
# TODO: 1. check with dummy target (1D)                                 -   Akansha, Karthik
# TODO: 2. check with dummy target (2D)                                 -   Akansha, Karthik
# TODO: 3. set up non-linear inequality constraints for obstascles      -   Akansha
# TODO: 4. set up TVP to account for moving obstacles                   -   Karthik
# TODO: 5.

import importlib.util
import do_mpc
from casadi import *
import numpy as np
from trajectory import Trajectory
        
class MPC:
    def __init__(self,
                 trajectory: Trajectory,
                #  vehicle,
                 n_horizon=7,
                 t_step=0.1,
                 n_robust=1,
                 r=1e-5,
                 model_type='continuous'):  # TODO: check discrete too

        self.model = do_mpc.model.Model(model_type)
        self.n_horizon = n_horizon
        self.t_step = t_step
        self.n_robust = n_robust
        self.r = r
        self.lr = 0.494/2
        self.L = 0.494
        self.model_type = model_type

    def setup_model(self, model_type):
        self.model2 = do_mpc.model.Model(model_type)
        # define model variables
        self.x = self.model2.set_variable(
            var_type='_x', var_name='x', shape=(1, 1))
        self.y = self.model2.set_variable(
            var_type='_x', var_name='y', shape=(1, 1))
        self.yaw = self.model2.set_variable(
            var_type='_x', var_name='yaw', shape=(1, 1))
        self.delta = self.model2.set_variable(
            var_type='_x', var_name='delta', shape=(1, 1))
        
        # define inputs
        self.v = self.model2.set_variable(var_type='_u', var_name='v')
        self.phi = self.model2.set_variable(var_type='_u', var_name='phi')

        # # define time varying parameters (velocity obstacles)
        # car_radius = L/2
        # centers_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(positions),1))
        # radii_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(radii),1))
        # set right hand side equation

        self.beta = np.arctan(self.lr*np.tan(self.delta)/self.L)
        self.model2.set_rhs('x', self.v*np.cos(self.yaw + self.beta))
        self.model2.set_rhs('y', self.v*np.sin(self.yaw + self.beta))
        self.model2.set_rhs('yaw', self.v*np.tan(self.delta)*np.cos(self.beta)/self.L)
        self.model2.set_rhs('delta', self.phi)

         # setup model
        self.model2.setup()

        # configure the MPC controller
        self.mpc2 = do_mpc.controller.MPC(self.model2)
        # optimizer parameters
        setup_mpc = {
            'n_horizon': self.n_horizon,
            't_step': self.t_step,
            'n_robust': self.n_robust,
            'store_full_solution': True,
        }

        self.mpc2.set_param(**setup_mpc)

        # constraints
        # bounds
        # SET STATE BOUNDS FOR x,y,yaw,delta: HARDCODED
        self.mpc2.bounds['lower', '_x', 'delta'] = -np.pi/4
        self.mpc2.bounds['upper', '_x', 'delta'] = np.pi/4
        self.mpc2.bounds['lower', '_x', 'yaw'] = -2*np.pi
        self.mpc2.bounds['upper', '_x', 'yaw'] = 2*np.pi

        # SET MIN R
        self.R_min = self.L/np.tan(np.pi/6) 

        # SET INPUT BOUNDS FOR v,phi: HARDCODED
        self.mpc2.bounds['lower', '_u', 'v'] = 0.
        self.mpc2.bounds['upper', '_u', 'v'] = 1.
        self.mpc2.bounds['lower', '_u', 'phi'] = -0.4
        self.mpc2.bounds['upper', '_u', 'phi'] = 0.4

    def plan(self, state, x_target, obstacles = None):
        self.setup_model(model_type=self.model_type)
        # objective function
        alpha = np.arctan2(x_target[1] - state.position[1], x_target[0] - state.position[0]) - state.position[2]
        ld = np.linalg.norm(x_target[:2]-state.position[:2],ord=2)
        delta_ref = np.arctan(2*self.L*np.sin(alpha)/ld)


        # define weights for costs
        # HARDCODED
        # TODO: tune
        K_pos = 1.0
        K_yaw = 1.0
        K_delta = 1.0

        w_track = 1.0
        w_progress = 0.0
        w_avoid = 1.0

        # HARDCODED
        v_ref = 4.0
        # costs
        # TODO: saturate
        J_track = K_pos*((x_target[0] - self.x)**2 + (x_target[1] - self.y)**2) + \
                 K_yaw*(x_target[2] - self.yaw)**2 + K_delta*(delta_ref - self.delta)
        J_progress = (self.v- v_ref)**2
        J_avoid = 1.0**2 - (self.x - 5)**2 - (self.y - 0.1)**2
        
        mterm = w_track*J_track +  w_avoid*J_avoid
        lterm = mterm + w_progress*J_progress 

        

        # TODO: ensure that delta can be used in the objective function
        # getting the optimal step
        x0 = np.append(state.position, state.steering)
        self.mpc2.x0 = x0
        
        # mterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        # lterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        self.mpc2.set_objective(mterm=mterm, lterm=lterm)
        # penalty for control inputs
        self.mpc2.set_rterm(v= self.r, phi=self.r)
        # HARDCODED circle avoidance constraint

        # TODO: set phi bound
        self.mpc2.set_nl_cons("omega_lb", self.phi - 0.7 * self.v/self.R_min, 0)
        self.mpc2.set_nl_cons("omega_ub", -self.phi - 0.7 * self.v/self.R_min, 0)
        # self.mpc2.set_nl_cons("circle",     1.0**2 - (self.x - 5)**2 - (self.y - 0)**2, 0)
        
        # setup mpc
        self.mpc2.setup()
        self.mpc2.set_initial_guess()
        # configure simulator
        simulator = do_mpc.simulator.Simulator(self.model2)
        # params for simulator
        simulator.set_param(t_step=self.t_step)
        
        u0 = self.mpc2.make_step(x0) 
        print("\n\n\n\n", u0)
        return u0 