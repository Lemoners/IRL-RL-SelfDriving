import socket
import random
from struct import *
import traceback
import collections
import numpy as np
import tensorflow as tf

class preprocess(object):

    def function(self, pos_info, time, stuck):
        tmp = 23
        width = pos_info[25] + pos_info[26]
        #tmp_l = pos_info[26]
        l = pos_info[0:19] + pos_info[23:25]  #sensor, angle, tomid
        l = list(l)
        l = l + [pos_info[tmp + 6], pos_info[tmp + 7], pos_info[tmp + 8], pos_info[50]] #speedX,Y,Z,rpm,23+4+4 = 31
        #r = l[21]*np.cos(l[19]) - np.abs(l[21]*np.sin(l[19])) - l[21]*np.abs(l[20])
        r = l[21]*np.cos(l[19])

        for i in range(19):
            l[i] = l[i]/200

        l[19] = l[19]/3.1416   #angle
        l[20] = l[20]/width   #tomiddle

        for i in range(21, 24):
            l[i] = l[i] * 3.6/300

        l[24] = l[24]/10000

        # if (time % 20 == 0):
        #     for i in range(0, 24):
        #         print(l[i], end = ' ')
        #     print(l[24])
        ##l = pos_info[0:tmp]
        '''for i in range(4):
            if pos_info[tmp + 4*i] == 1:
                l = l + (1.0, 0.0, 0.0)
            if pos_info[tmp + 4*i] == 2:
                l = l + (0.0, 1.0, 0.0)
            if pos_info[tmp + 4*i] == 3:
                l = l + (0.0, 0.0, 1.0)
            l = l + pos_info[tmp + 4*i + 1: tmp + 4*i + 4]
        l = l + pos_info[25] + pos_info[26]
        l = list(l)
        #normal
        #angle and tostart
        l[0] = l[0]/3.1415926
        #disttomid/left/right
        l[1] = l[1]/width
        l[2] = l[2]/width
        l[3] = l[3]/width
        #fuel and weight
        l[4] = l[4]/100.0
        l[5] = l[5]/1500.0
        #speed
        l[6] = l[6]/300.0
        l[7] = l[7]/300.0
        l[8] = l[8]/300.0
        #kfiction and length
        l[13] = l[13]/10
        l[14] = l[14]/600
        l[13 + 6] = l[13 + 6]/10
        l[14 + 6] = l[14 + 6]/1000
        l[13 + 6*2] = l[13 + 6*2]/10
        l[14 + 6*2] = l[14 + 6*2]/1000
        l[13 + 6*3] = l[13 + 6*3]/10
        l[14 + 6*3] = l[14 + 6*3]/1000

        print('state------>%.3f %.3f %.3f %.3f %.3f %.3f %.3f' %(np.cos(l[0]*3.1415926), np.sin(l[0]*3.1415926), l[14], l[15]*1000, l[16], l[17], l[20]*1000))
        r = l[6]*np.cos(l[0]*3.1415926)*300 - np.abs(l[6]*np.sin(l[0]*3.1415926)*300)
        '''
        if l[20] < -1 + 0.3:
            r = -10
            stuck = 1
        if l[20] > 1 - 0.3:
            r = -10
            stuck = 1
        #if time > TIMELIMIT:
        #    r = -10
        #    stuck = 1
        if np.cos(l[19]*3.1416) < 0:
            r = -10
            stuck = 1 
        #if time > 300:
        #    if l[6] < 5.0/300:
        #        r = -1
        #        stuck = 1
        l = tuple(l)
        return r, l, stuck
np.random.seed(2)
tf.set_random_seed(2)  # reproducible


class Actor(object):
    def __init__(self, sess, n_features, action_bound, lr=0.0001):

        self.sess = sess
        self.s = tf.placeholder(tf.float32, [1, n_features], "state")
        self.a = tf.placeholder(tf.float32, None, name="act")
        self.td_error = tf.placeholder(tf.float32, None, name="td_error")  # TD_error

        l1 = tf.layers.dense(
            inputs=self.s,
            units=30,  # number of hidden units
            activation=tf.nn.relu,
            kernel_initializer=tf.random_normal_initializer(0., .1),  # weights
            bias_initializer=tf.constant_initializer(0.1),  # biases
            name='l1'
        )

        mu = tf.layers.dense(
            inputs=l1,
            units=1,  # number of hidden units
            activation=tf.nn.tanh,
            kernel_initializer=tf.random_normal_initializer(0., .1),  # weights
            bias_initializer=tf.constant_initializer(0.1),  # biases
            name='mu'
        )

        sigma = tf.layers.dense(
            inputs=l1,
            units=1,  # output units
            activation=tf.nn.softplus,  # get action probabilities
            kernel_initializer=tf.random_normal_initializer(0., .1),  # weights
            bias_initializer=tf.constant_initializer(1.),  # biases
            name='sigma'
        )
        global_step = tf.Variable(0, trainable=False)
        # self.e = epsilon = tf.train.exponential_decay(2., global_step, 1000, 0.9)
        self.mu, self.sigma = tf.squeeze(mu*2), tf.squeeze(sigma+0.1)
        self.normal_dist = tf.distributions.Normal(self.mu, self.sigma)

        self.action = tf.clip_by_value(self.normal_dist.sample(1), action_bound[0], action_bound[1])

        with tf.name_scope('exp_v'):
            log_prob = self.normal_dist.log_prob(self.a)  # loss without advantage
            self.exp_v = log_prob * self.td_error  # advantage (TD_error) guided loss
            # Add cross entropy cost to encourage exploration
            self.exp_v += 0.01*self.normal_dist.entropy()

        with tf.name_scope('train'):
            self.train_op = tf.train.AdamOptimizer(lr).minimize(-self.exp_v, global_step)    # min(v) = max(-v)

    def learn(self, s, a, td):
        s = s[np.newaxis, :]
        feed_dict = {self.s: s, self.a: a, self.td_error: td}
        _, exp_v = self.sess.run([self.train_op, self.exp_v], feed_dict)
        return exp_v

    def choose_action(self, s):
        s = s[np.newaxis, :]
        return self.sess.run(self.action, {self.s: s})  # get probabilities for all actions


class Critic(object):
    def __init__(self, sess, n_features, lr=0.01):
        self.sess = sess
        with tf.name_scope('inputs'):
            self.s = tf.placeholder(tf.float32, [1, n_features], "state")
            self.v_ = tf.placeholder(tf.float32, [1, 1], name="v_next")
            self.r = tf.placeholder(tf.float32, name='r')

        with tf.variable_scope('Critic'):
            l1 = tf.layers.dense(
                inputs=self.s,
                units=30,  # number of hidden units
                activation=tf.nn.relu,
                kernel_initializer=tf.random_normal_initializer(0., .1),  # weights
                bias_initializer=tf.constant_initializer(0.1),  # biases
                name='l1'
            )

            self.v = tf.layers.dense(
                inputs=l1,
                units=1,  # output units
                activation=None,
                kernel_initializer=tf.random_normal_initializer(0., .1),  # weights
                bias_initializer=tf.constant_initializer(0.1),  # biases
                name='V'
            )

        with tf.variable_scope('squared_TD_error'):
            self.td_error = tf.reduce_mean(self.r + GAMMA * self.v_ - self.v)
            self.loss = tf.square(self.td_error)    # TD_error = (r+gamma*V_next) - V_eval
        with tf.variable_scope('train'):
            self.train_op = tf.train.AdamOptimizer(lr).minimize(self.loss)

    def learn(self, s, r, s_):
        s, s_ = s[np.newaxis, :], s_[np.newaxis, :]

        v_ = self.sess.run(self.v, {self.s: s_})
        td_error, _ = self.sess.run([self.td_error, self.train_op],
                                          {self.s: s, self.v_: v_, self.r: r})
        return td_error


OUTPUT_GRAPH = False
MAX_EPISODE = 1000
MAX_EP_STEPS = 200
DISPLAY_REWARD_THRESHOLD = -100  # renders environment if total episode reward is greater then this threshold
RENDER = False  # rendering wastes time
GAMMA = 0.9
LR_A = 0.001    # learning rate for actor
LR_C = 0.01     # learning rate for critic

N_S = 26
A_BOUND = 1

sess = tf.Session()

actor = Actor(sess, n_features=N_S, lr=LR_A, action_bound=[-A_BOUND, A_BOUND])
critic = Critic(sess, n_features=N_S, lr=LR_C)

sess.run(tf.global_variables_initializer())


ip_port = ('127.0.0.1', 9999)
s = socket.socket()
s.bind(ip_port)
s.listen()

pre = preprocess()

while(True):
    conn, addr = s.accept()
    # epsilon -= 1.0 / explore
    stuck = 0
    time = 0
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
            # action = actor.choose_action(state)
            action = 1

            conn.send(pack('4f', action, 1, 0, time))
            print("send once")
            recv_data = conn.recv(1024)
            if not recv_data:
                break
            pos_info = unpack('51f', recv_data)
            r, info, stuck = pre.function(pos_info, time, stuck)

            #steer, acc, 
            state_= list(info)
            state_.append(stuck)
            state_ = np.array(state_)

            # td_error = critic.learn(state, r, state_)
            # actor.learn(state, action, td_error)
            if stuck:
                time = 10

            state = state_
        # except:
        #     print("fail")

# if __name__ == '__main__':
#     main()
