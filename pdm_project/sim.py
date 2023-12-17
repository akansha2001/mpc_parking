from environment import *

if __name__ == "__main__":
    parking_lot = ParkingLotEnv(stat_obs_flag=True, dyn_obs_flag=False)
    parking_lot.run_env()