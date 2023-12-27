
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
                 r=1e-2,
                 model_type='continuous'):  # TODO: check discrete too

        self.model = do_mpc.model.Model(model_type)
        self.n_horizon = n_horizon
        self.t_step = t_step
        self.n_robust = n_robust
        self.r = r
        self.lr = 0.494/2
        self.L = 0.494
        self.setup_model(model_type=model_type)


    def get_control_input(self, N, x_init, x_target, obstacles):

        # get positions, velocities and radii for obstacles
        positions = [obstacle.position() for obstacle in obstacles]
        velocities = [obstacle.velocity() for obstacle in obstacles]
        radii = [obstacle.radius() for obstacle in obstacles]

        # define model variables
        x = self.model.set_variable(var_type='_x', var_name='x', shape=(1, 1))
        y = self.model.set_variable(var_type='_x', var_name='y', shape=(1, 1))
        yaw = self.model.set_variable(
            var_type='_x', var_name='yaw', shape=(1, 1))
        delta = self.model.set_variable(
            var_type='_x', var_name='delta', shape=(1, 1))

        # define inputs
        v = self.model.set_variable(var_type='_u', var_name='v')
        phi = self.model.set_variable(var_type='_u', var_name='phi')

        # model parameters
        # CHECK VEHICLE PRIUS

        # # define time varying parameters (velocity obstacles)
        # car_radius = L/2
        # centers_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(positions),1))
        # radii_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(radii),1))

        # set right hand side equation
        beta = np.arctan(self.lr*np.tan(delta)/self.L)
        self.model.set_rhs('x', v*np.cos(yaw + beta))
        self.model.set_rhs('y', v*np.sin(yaw + beta))
        self.model.set_rhs('yaw', v*np.tan(delta)*np.cos(beta)/L)
        self.model.set_rhs('delta', phi)

        # setup model
        self.model.setup()

        # configure the MPC controller
        mpc = do_mpc.controller.MPC(self.model)

        # optimizer parameters
        setup_mpc = {
            'n_horizon': self.n_horizon,
            't_step': self.t_step,
            'n_robust': self.n_robust,
            'store_full_solution': True,
        }
        mpc.set_param(**setup_mpc)

        # objective function
        mterm = (x_target[0] - x)**2 + \
            (x_target[1] - y)**2 + (x_target[2] - yaw)**2
        lterm = (x_target[0] - x)**2 + \
            (x_target[1] - y)**2 + (x_target[2] - yaw)**2

        # TODO: ensure that delta can be used in the objective function
        # mterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        # lterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        mpc.set_objective(mterm=mterm, lterm=lterm)

        # penalty for control inputs
        mpc.set_rterm(v=self.r, phi=self.r)

        # constraints

        # bounds
        # SET STATE BOUNDS FOR x,y,yaw,delta: HARDCODED
        mpc.bounds['lower', '_x', 'delta'] = -np.pi/6
        mpc.bounds['upper', '_x', 'delta'] = np.pi/6
        mpc.bounds['lower', '_x', 'yaw'] = -2*np.pi
        mpc.bounds['upper', '_x', 'yaw'] = 2*np.pi
        # SET INPUT BOUNDS FOR v,phi: HARDCODED
        mpc.bounds['lower', '_u', 'v'] = 0.
        mpc.bounds['upper', '_u', 'v'] = 5.
        mpc.bounds['lower', '_u', 'phi'] = -1.
        mpc.bounds['upper', '_u', 'phi'] = 1.

        # inequality constraints for dynamic obstacles
        # TO DO

        mpc.setup()

        # configure simulator
        simulator = do_mpc.simulator.Simulator(self.model)

        # params for simulator
        simulator.set_param(t_step=self.t_step)

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
        self.mpc2.bounds['lower', '_x', 'delta'] = -np.pi/6
        self.mpc2.bounds['upper', '_x', 'delta'] = np.pi/6
        self.mpc2.bounds['lower', '_x', 'yaw'] = -2*np.pi
        self.mpc2.bounds['upper', '_x', 'yaw'] = 2*np.pi
        # SET INPUT BOUNDS FOR v,phi: HARDCODED
        self.mpc2.bounds['lower', '_u', 'v'] = 0.
        self.mpc2.bounds['upper', '_u', 'v'] = 5.
        self.mpc2.bounds['lower', '_u', 'phi'] = -1.
        self.mpc2.bounds['upper', '_u', 'phi'] = 1.

    def plan(self, state, x_target, obstacles = None):
        # objective function
        mterm = (x_target[0] - self.x)**2 + \
            (x_target[1] - self.y)**2 + (x_target[2] - self.yaw)**2
        lterm = (x_target[0] - self.x)**2 + \
            (x_target[1] - self.y)**2 + (x_target[2] - self.yaw)**2

        # TODO: ensure that delta can be used in the objective function
        # getting the optimal step
        x0 = np.append(state.position, state.steering)
        self.mpc2.x0 = x0
        # self.mpc2.set_initial_guess()
        
        # mterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        # lterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        self.mpc2.set_objective(mterm=mterm, lterm=lterm)
        # penalty for control inputs
        self.mpc2.set_rterm(v=self.r, phi=self.r)
        # setup mpc
        self.mpc2.setup()
        # configure simulator
        simulator = do_mpc.simulator.Simulator(self.model2)
        # params for simulator
        simulator.set_param(t_step=self.t_step)

        u0 = self.mpc2.make_step(x0) 
        print("\n\n\n\n", u0)
        return u0 