#! /usr/bin/env python

import gym
import mytorcs
import subprocess
env = gym.make("MyTorcs-v0")
env.seed(0)

EPISODE = 100

for i in range(EPISODE):
    print("new Episode")
    ob = env.reset(on_driving_reset=True)
    print("Env reset done")
    i = 0
    while (i < 50):
        action = [0.0,1.0,0.0, 0.0]
        ob, reward, done, info = env.step(action)
        i += 1
        print(i)
        if done:
            break;

env.close()

x






# import subprocess

# a = subprocess.Popen("torcs")
# # a.kill()
# print("a-----:", a.poll())

















