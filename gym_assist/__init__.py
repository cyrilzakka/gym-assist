import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='Suture-v0',
    entry_point='gym_assist.envs:SutureEnv',
)