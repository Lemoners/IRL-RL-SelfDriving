import tensorflow as tf
import numpy as np
OUTPUT_GRAPH = False
MAX_EPISODE = 500
MAX_EP_STEPS = 2000   # maximum time step in one episode
RENDER = False  # rendering wastes time
GAMMA = 0.9     # reward discount in TD error
LR_A = 0.001    # learning rate for actor
LR_C = 0.01     # learning rate for critic
class Actor(object):
    def __init__(self, sess, observation_dim, action_dim, lr=0.001):
        self.sess = sess
        self.s = tf.placeholder(tf.float32, [1, observation_dim], "state")
        self.a = tf.placeholder(tf.int32, None, "action")
        self.td_error = tf.placeholder(tf.float32, None, "td_error")  # TD_error

        with tf.variable_scope('Actor'):
            l1 = tf.layers.dense(self.s, 64, tf.nn.relu, name='l1')
            self.acts_prob = tf.layers.dense(l1, action_dim, tf.nn.softmax, name='acts_prob')

        with tf.variable_scope('exp_v'):
            log_prob = tf.log(self.acts_prob[0, self.a])
            self.exp_v = tf.reduce_mean(log_prob * self.td_error)  # advantage (TD_error) guided loss

        with tf.variable_scope('train'):
            self.train_op = tf.train.AdamOptimizer(lr).minimize(-self.exp_v)

    def learn(self, s, a, td):
        feed_dict = {self.s: s[None], self.a: a, self.td_error: td}
        _, exp_v = self.sess.run([self.train_op, self.exp_v], feed_dict)
        return exp_v

    def act(self, s):
        probs = self.sess.run(self.acts_prob, {self.s: s[None]})  
        return np.random.choice(np.arange(probs.shape[1]), p=probs.ravel())   # return a int


class Critic(object):
    def __init__(self, sess, observation_dim, lr=0.01):
        self.sess = sess
        self.s = tf.placeholder(tf.float32, [1, observation_dim], "state")
        self.v_ = tf.placeholder(tf.float32, [1, 1], "v_next")
        self.r = tf.placeholder(tf.float32, None, 'r')

        with tf.variable_scope('Critic'):
            l1 = tf.layers.dense(self.s, 64, tf.nn.relu, name='l1')
            self.v = tf.layers.dense(l1, 1, name='V')

        with tf.variable_scope('squared_TD_error'):
            self.td_error = self.r + GAMMA * self.v_ - self.v
            self.loss = tf.square(self.td_error)

        with tf.variable_scope('train'):
            self.train_op = tf.train.AdamOptimizer(lr).minimize(self.loss)

    def learn(self, s, r, s_):
        v_ = self.sess.run(self.v, {self.s: s_[None]})
        td_error, _ = self.sess.run([self.td_error, self.train_op],
                                          {self.s: s[None], self.v_: v_, self.r: r})
        return td_error
    
class ACNet(object):
    def __init__(self, sess,  observation_dim, action_dim, lr=0.001):
        self.sess = sess
        self.s = tf.placeholder(tf.float32, [1, observation_dim], "state")
        self.a = tf.placeholder(tf.int32, None, "action")
        self.v_ = tf.placeholder(tf.float32, [1, 1], "v_next")
        self.r = tf.placeholder(tf.float32, None, 'r')
        l1 = tf.layers.dense(self.s, 64, tf.nn.relu, name='l1')
        self.v = tf.layers.dense(l1, 1, name='V')

        self.td_error = self.r + GAMMA * self.v_ - self.v
        self.mse_loss = 0.5*tf.square(self.td_error)

        self.acts_prob = tf.layers.dense(l1, action_dim, tf.nn.softmax, name='acts_prob')
        log_prob = tf.log(self.acts_prob[0, self.a])
        self.exp_v = -tf.reduce_mean(log_prob * self.td_error)  # advantage (TD_error) guided 
        
        self.loss = 0.5 * self.mse_loss + self.exp_v

        self.train_op = tf.train.AdamOptimizer(lr).minimize(self.loss)

    def learn(self, s, a, r, s_):
        v_ = self.sess.run(self.v, {self.s: s_[None]})
        feed_dict = {self.s: s[None], self.a: a,  self.r: r, self.v_: v_}
        self.sess.run(self.train_op, feed_dict)

    def act(self, s):
        probs = self.sess.run(self.acts_prob, {self.s: s[None]})  
        return np.random.choice(np.arange(probs.shape[1]), p=probs.ravel())   # return a int