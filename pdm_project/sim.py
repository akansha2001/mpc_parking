from environment import *

if __name__ == "__main__":
    num_runs=2
    for i in range(num_runs):
        parking_lot = ParkingLotEnv(stat_obs_flag=False, dyn_obs_flag=True)
        parking_lot.run_env()