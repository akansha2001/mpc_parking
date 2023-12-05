
## Environment Docs
<!-- Shift + Ctrl + V opens up the preview -->

## OpenAI Gymnasium Documentation
The main API methods that users of this class need to know are:

    step() - Updates an environment with actions returning the next agent observation, the reward for taking that actions, if the environment has terminated or truncated due to the latest action and information from the environment about the step, i.e. metrics, debug info.

    reset() - Resets the environment to an initial state, required before calling step. Returns the first agent observation for an episode and information, i.e. metrics, debug info.

    render() - Renders the environments to help visualise what the agent see, examples modes are “human”, “rgb_array”, “ansi” for text.

    close() - Closes the environment, important when external software is used, i.e. pygame for rendering, databases

## Spawning Cars

## Spawning Obstacles