import socket
import random
from struct import *
import traceback
import collections
import numpy as np
import tensorflow as tf
from network import Actor, Critic

class preprocess(object):

    def function(self, pos_info, time, stuck):
        tmp = 23
        width = pos_info[25] + pos_info[26]
        l = pos_info[0:19] + pos_info[23:25]  #sensor, angle, tomid
        l = list(l)
        l = l + [pos_info[tmp + 6], pos_info[tmp + 7], pos_info[tmp + 8], pos_info[50]] #speedX,Y,Z,rpm,23+4+4 = 31
        r = l[21]*np.cos(l[19])

        for i in range(19):
            l[i] = l[i]/200

        l[19] = l[19]/3.1416   #angle
        l[20] = l[20]/width   #tomiddle

        for i in range(21, 24):
            l[i] = l[i] * 3.6/300

        l[24] = l[24]/10000

        if l[20] < -1 + 0.3:
            r = -10
            stuck = 1
        if l[20] > 1 - 0.3:
            r = -10
            stuck = 1
        if np.cos(l[19]*3.1416) < 0:
            r = -10
            stuck = 1 
        l = tuple(l)
        return r, l, stuck

def main():
    ip_port = ('127.0.0.1', 9999)
    s = socket.socket()
    s.bind(ip_port)
    s.listen()
    stuck = 0
    time = 0
    pre = preprocess()

    OUTPUT_GRAPH = False
    MAX_EPISODE = 500
    MAX_EP_STEPS = 2000   # maximum time step in one episode
    RENDER = False  # rendering wastes time
    GAMMA = 0.9     # reward discount in TD error
    LR_A = 0.001    # learning rate for actor
    LR_C = 0.01     # learning rate for critic
    N_F = 26
    N_A = 4

    sess = tf.Session()
    actor = Actor(sess, observation_dim=N_F, action_dim=N_A, lr=LR_A)
    critic = Critic(sess, observation_dim=N_F, lr=LR_C)
    sess.run(tf.global_variables_initializer())
    while(True):
        conn, addr = s.accept()
        # epsilon -= 1.0 / explore
        recv_data = conn.recv(1024)
        if not recv_data:
            break
        pos_info = unpack('51f', recv_data)
        r, info, stuck = pre.function(pos_info, time, stuck)

        state= list(info)
        state.append(stuck)
        state = np.array(state)
        while(True):
            # try:
                action = actor.act(state)
                conn.send(pack('4f', action, 1, 0, 1.0))

                recv_data = conn.recv(1024)
                if not recv_data:
                    break
                pos_info = unpack('51f', recv_data)
                r, info, stuck = pre.function(pos_info, time, stuck)

                #steer, acc, 
                state_= list(info)
                state_.append(stuck)
                state_ = np.array(state_)

                state = state_
            # except:
            #     print("fail")

if __name__ == '__main__':
    main()
