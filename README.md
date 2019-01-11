# Ingredients for Medical Robotics Research
This repository includes simulated robotic environments and tools for use in teaching agents various tasks required for medical procedures and surgeries.

## Getting Started
The environments are built using [MuJoCo](https://github.com/openai/mujoco-py#obtaining-the-binaries-and-license-key) and [Gym](https://github.com/openai/gym), meaning that both of those packages and their dependencies are required prior to using Assist.

#### Setup and Installation
Clone this repository on your system and install Assist using the following commands:

    
    git clone https://github.com/cyrilzakka/assist
    cd gym-assist
    pip install -e .
    

#### Testing the Environments

    
    import gym
    import gym_assist
    
    env = gym.make('Suture-v0')
    env.reset()
    
    for _ in range(1000):
        env.render()
        env.step(env.action_space.sample()) #take a random action
    
    env.close()
    


## Citation
    @misc {
          Cyrilzakka,
          author = {Zakka, Cyril},
          title = {Ingredients for Medical Robotics Research},
          year = {2018},
          publisher = {GitHub},
          journal = {GitHub repository},
          howpublished = {\url{https://github.com/cyrilzakka/Assist}},
    }
