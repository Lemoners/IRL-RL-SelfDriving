#! /usr/bin/env python

import gym
import mytorcs
import subprocess
env = gym.make("MyTorcs-v0")
env.seed(0)

EPISODE = 100

for i in range(EPISODE):
    print("new Episode")
    ob = env.reset()
    print("Env reset done")

    while True:
        action = [0.0,1.0,0.0, 0.0]
        ob, reward, done, info = env.step(action)

        if done:
            break;

env.close()

# # from gym import envs
# # envids = [spec.id for spec in envs.registry.all()]
# # for envid in sorted(envids):
# #     if("torcs" in str(envid).lower()):
# #         print(envid)






# import subprocess

# a = subprocess.Popen("torcs")
# # a.kill()
# print("a-----:", a.poll())

















