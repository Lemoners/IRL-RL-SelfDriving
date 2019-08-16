import os
from threading import Thread
import random
import time
import numpy as np
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from mytorcs.env.mytorcs_sim import MyTorcsController
from mytorcs.env.mytorcs_proc import MyTorcsProcess


class MyTorcsEnv(gym.Env):
    """
    open ai env for torcs
    """

    meta = {
        "render.modes": ["human", "rgb_array"],
    }

    ACTION_NAMES = ["steer", "accelerate", "brake", "info"]
    STEER_LIMIT_LEFT = -1.0
    STEER_LIMIT_RIGHT = 1.0
    ACCELERATE_MIN = 0.0
    ACCELERATE_MAX = 1.0
    BRAKE_MIN=0.0
    BRAKE_MAX=1.0
    VAL_PER_OBS = 26 #obs的维度 

    def __init__(self, time_step=0.05, frame_skip=2):

        print("starting MyTorcs env")
         # start simulation subprocess
        self.proc = MyTorcsProcess()
        try:
            self.exe_path = "torcs"
        except:
            print("Missing torcs environment var. you must start sim manually")
            self.exe_path = None
        
        #port
        self.port = ('127.0.0.1', 9999);
            
        #no render
        self.headless = True

        self.proc.start(self.exe_path, headless=self.headless, port=self.port)

        # start simulation com
        self.viewer = MyTorcsController(time_step=time_step, port=self.port)


        # action_space
        self.action_space = spaces.Box(low=np.array([self.STEER_LIMIT_LEFT, self.ACCELERATE_MIN, self.BRAKE_MIN, -10]),
            high=np.array([self.STEER_LIMIT_RIGHT, self.ACCELERATE_MAX, self.BRAKE_MAX, 10]), dtype=np.float32 )

        # obs data
        self.observation_space = spaces.Box(-10, 10, [self.VAL_PER_OBS], dtype=np.float32)

        # simulation related variables.
        self.seed()

        # Frame Skipping
        self.frame_skip = frame_skip

        # wait until loaded
        self.viewer.wait_until_loaded()

        


    def __del__(self):
        self.close()

    def close(self):
        self.proc.quit()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    #!!
    #action的最后一维没有被使用
    def step(self, action):
        for i in range(self.frame_skip):
            self.viewer.take_action(action)
            print("excute", action)
            observation, reward, done, info = self.viewer.observe()
        return observation, reward, done, info

    def reset(self):
        # print("Into env reset")
        self.viewer.reset()
        # print("rec date", self.viewer.first_receive_data)
        observation, reward, done, info = self.viewer.observe(reset=True)
        # print("Out env reset with obs:", observation)
        time.sleep(1)
        return observation



    def is_game_over(self):
        return self.viewer.is_game_over()