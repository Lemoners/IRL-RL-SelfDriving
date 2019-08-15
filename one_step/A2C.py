import numpy as np
import tensorflow as tf
import gym
from network import Actor, Critic, ACNet
OUTPUT_GRAPH = False
MAX_EPISODE = 5000
MAX_EP_STEPS = 2000   # maximum time step in one episode
RENDER = False  # rendering wastes time
GAMMA = 0.9     # reward discount in TD error
LR_A = 0.001    # learning rate for actor
LR_C = 0.01     # learning rate for critic

env = gym.make('CartPole-v0')
env.seed(0)  # reproducible
env = env.unwrapped

N_F = env.observation_space.shape[0]
N_A = env.action_space.n
print(N_A) 
sess = tf.Session()
# actor = Actor(sess, observation_dim=N_F, action_dim=N_A, lr=LR_A)
# critic = Critic(sess, observation_dim=N_F, lr=LR_C)
model = ACNet(sess, observation_dim=N_F, action_dim=N_A, lr=LR_A)

sess.run(tf.global_variables_initializer())

res = []
for i_episode in range(MAX_EPISODE):
    s = env.reset()
    timesteps = 0
    track_r = []
    while True:
        # env.render()
        a = model.act(s)
        s_, r, done, info = env.step(a)
        if done:
            r = -20
        track_r.append(r)

        model.learn(s, a, r, s_)

        s = s_
        timesteps += 1
        if done or timesteps >= MAX_EP_STEPS:
            ep_rs_sum = sum(track_r)
            #moving average
            if 'running_reward' not in globals():
                running_reward = ep_rs_sum
            else:
                running_reward = running_reward * 0.95 + ep_rs_sum * 0.05
            if i_episode % 100 == 0:
                print(ep_rs_sum)
            break

