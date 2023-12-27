import importlib.util
import do_mpc
from casadi import *
import numpy as np

class MPC:
    def __init__(self,n_horizon = 7, t_step = 0.1, n_robust = 1, r=1e-2, model_type='continuous'):
        
        self.model = do_mpc.model.Model(model_type)
        self.n_horizon = n_horizon
        self.t_step = t_step
        self.n_robust = n_robust
        self.r = r

    def get_control_input(self,vehicle, N, x_init, x_target,obstacles):

        # get positions, velocities and radii for obstacles
        positions = [obstacle.position() for obstacle in obstacles]
        velocities = [obstacle.velocity() for obstacle in obstacles]
        radii = [obstacle.radius() for obstacle in obstacles]

        model = self.model

        # define model variables
        x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
        y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
        yaw = model.set_variable(var_type='_x', var_name='yaw', shape=(1,1))
        delta = model.set_variable(var_type='_x', var_name='delta', shape=(1,1))

        # define inputs
        v = model.set_variable(var_type='_u', var_name='v')
        phi = model.set_variable(var_type='_u', var_name='phi')


        # model parameters
        # CHECK VEHICLE PRIUS
        lr = vehicle.lr
        L  = vehicle.L

        # # define time varying parameters (velocity obstacles)
        # car_radius = L/2
        # centers_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(positions),1))
        # radii_t = model.set_variable(var_type='_tvp', var_name='centers_t',shape=(len(radii),1))

        # set right hand side equation
        beta = np.arctan(lr*np.tan(delta)/L)
        model.set_rhs('x', v*np.cos(yaw + beta))
        model.set_rhs('y', v*np.sin(yaw + beta))
        model.set_rhs('yaw', v*np.tan(delta)*np.cos(beta)/L)
        model.set_rhs('delta', phi)

        # setup model
        model.setup()

        # configure the MPC controller
        mpc = do_mpc.controller.MPC(model)

        # optimizer parameters
        setup_mpc = {
            'n_horizon': self.n_horizon,
            't_step': self.t_step,
            'n_robust': self.n_robust,
            'store_full_solution': True,
        }
        mpc.set_param(**setup_mpc)

        # objective function
        mterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2
        lterm = (x_target[0] - x)**2 + (x_target[1] - y)**2 + (x_target[2] - yaw)**2 + delta**2

        mpc.set_objective(mterm=mterm, lterm=lterm)

        # penalty for control inputs
        mpc.set_rterm(v=self.r,phi=self.r)

        # constraints

        # bounds
        # SET STATE BOUNDS FOR x,y,yaw,delta: HARDCODED
        mpc.bounds['lower','_x', 'delta'] = -np.pi/6
        mpc.bounds['upper','_x', 'delta'] = np.pi/6
        mpc.bounds['lower','_x', 'yaw'] = -2*np.pi
        mpc.bounds['upper','_x', 'yaw'] = 2*np.pi
        # SET INPUT BOUNDS FOR v,phi: HARDCODED
        mpc.bounds['lower','_u', 'v'] = 0.
        mpc.bounds['upper','_u', 'v'] = 5.
        mpc.bounds['lower','_u', 'phi'] = -1.
        mpc.bounds['upper','_u', 'phi'] = 1.

        # inequality constraints for dynamic obstacles
        # TO DO

        mpc.setup()

        # configure simulator
        simulator = do_mpc.simulator.Simulator(model)

        # params for simulator
        simulator.set_param(t_step = self.t_step)


        


