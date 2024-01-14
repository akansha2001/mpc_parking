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
        For this method, make sure to install the lib using `pip3 install .`!
2. Install `do-mpc`
    ```bash
    pip install do_mpc[full]
    ```
    You can test the installation by:
    ```bash
    cd pdm_project/tests
    python3 test_mpc.py
    ```

    If you encounter an error regarding the `OpenSSL` package, try this:
    ```bash
    pip install pip --upgrade
    pip install pyopenssl --upgrade
    ```
    
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

### Installing OMPL 
* Download OMPL script from https://ompl.kavrakilab.org/installation.html 
```bash
cd Downloads
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh --python
cd ompl-1.6.0/demos
python3 RigidBodyPlanning.py
```
* You may run into an error regarding no module named util
```bash
sudo apt update
sudo apt install pypy3
```

* Another error you can encounter is regarding no module named base or geometric for which the following has to be done:
1) Open ompl-1.6.0/py-bindings/generate_bindings.py
2) Delete line containing self.ompl_ns.class_(f'SpecificParam< {self.string_decl} >').rename('SpecificParamString') in the code

```python
    try:
        self.ompl_ns.class_(f'SpecificParam< {self.string_decl} >').rename('SpecificParamString')
    except:
            try:
                self.ompl_ns.class_(f'SpecificParam< std::string >').rename('SpecificParamString')
            except:
                self.ompl_ns.class_(f'SpecificParam< std::basic_string< char > >').rename('SpecificParamString')
```

3) After this open the downloaded script install-ompl-ubuntu.sh and comment out all the wget lines inside install_ompl()