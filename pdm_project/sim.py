from environment import *

if __name__ == "__main__":
    num_runs=2
    for i in range(num_runs):
        parking_lot = ParkingLotEnv(stat_obs_flag=True, dyn_obs_flag=False)
        parking_lot.run_env()