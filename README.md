# PDM_Project
Planning and Decision Making Project Q2
## Installation
1. Install `gym_urdf_envs`
    ```bash
    git clone https://github.com/maxspahn/gym_envs_urdf.git
    ```
    Follow the README for installation instructions and make sure to install the lib using `pip3 install .`
## Documentation
### General
* For data coming from the environment, through the variable `obs`, check `docs/obs_example.json`.

### OpenAI Gymnasium Documentation

The main API methods that users of this class need to know are:

    step() - Updates an environment with actions returning the next agent observation, the reward for taking that actions, if the environment has terminated or truncated due to the latest action and information from the environment about the step, i.e. metrics, debug info.

    reset() - Resets the environment to an initial state, required before calling step. Returns the first agent observation for an episode and information, i.e. metrics, debug info.

    render() - Renders the environments to help visualise what the agent see, examples modes are “human”, “rgb_array”, “ansi” for text.

    close() - Closes the environment, important when external software is used, i.e. pygame for rendering, databases

