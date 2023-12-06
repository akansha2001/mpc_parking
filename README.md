# PDM_Project
Planning and Decision Making Project Q2
## Installation
1. Install `gym_urdf_envs`
    * Installation from pip (Recommended) 
        ```bash
        pip3 install urdfenvs
        ```
    * Install from source (not ideal)
        ```bash
        cd ~
        git clone https://github.com/maxspahn/gym_envs_urdf.git
        cd gym_envs_urdf
        pip3 install .
        ```

    Make sure to install the lib using `pip3 install .`!
## Documentation
* Check out `docs/gymnasium.md` for the environment documentation
* To understand the format of the data coming from the environment, through the variable `obs`, check `docs/obs_example.json`.

## Usage
### Running the Simulation
```bash
cd pdm_project
python3 sim.py
```
### Using the Environment in your code

```python
# Import required libraries
import gymnasium as gym
from urdfenvs.robots.prius import Prius
import numpy as np

# Import the ParkingLotEnv class from your script
from environment import ParkingLotEnv

# Create an instance of the environment
env = ParkingLotEnv(render=True)

# Run the environment simulation
env.run_env()
```