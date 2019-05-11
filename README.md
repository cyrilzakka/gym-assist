# Ingredients for Medical Robotics Research
Despite the increasing prevalence of robotic surgical systems in operating rooms, little work has been done to automate their tasks or capabilities. Assist is an open-source interface to machine learning tasks that aims to provide medical researchers with the ability to quickly prototype and train machine learning algorithms in simulation, before deploying them to real-world settings.

## Getting Started
This repository includes simulated robotic environments and tools for use in teaching agents various tasks required for medical procedures and surgeries.
The environments are built using [MuJoCo](https://github.com/openai/mujoco-py#obtaining-the-binaries-and-license-key) and [Gym](https://github.com/openai/gym), meaning that both of those packages and their dependencies are required prior to using Assist.

#### Setup and Installation
Clone this repository on your system and install Assist using the following commands:

    
    git clone https://github.com/cyrilzakka/gym-assist
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

![alt text](https://cyrilzakka.github.io/assets/posts/post1/suture.jpg)


## Citation
    @misc {
          Cyrilzakka,
          author = {Zakka, Cyril},
          title = {Ingredients for Medical Robotics Research},
          year = {2019},
          publisher = {GitHub},
          journal = {GitHub repository},
          howpublished = {\url{https://github.com/cyrilzakka/gym-assist}},
    }
