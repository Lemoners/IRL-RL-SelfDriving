import tensorflow as tf
import numpy as np
import gym
np.random.seed(3)
tf.set_random_seed(3)  # reproducible

running_reward = 0
class Model(object):
    def __init__(self, sess, observation_dim, action_dim, env, lr=1e-3, batch_size=None):
        '''
        x: input states
        lr: learning rate
        clip_norm: unknown
        '''
        self.ep_nums =  0
        self.x = tf.placeholder(tf.float32, shape = [None, observation_dim])
        self.lr = lr
        self.env = env
        self.sess = sess
        self.trainer = tf.train.RMSPropOptimizer(learning_rate=self.lr, decay=0.99, epsilon=1e-5)
        self.start_state = env.reset()
        self.start_reward = 0
        self.gamma = 0.9
        # actor and critic share several layers which extract the features of observation
        shared_dense = tf.layers.dense(self.x, 64, tf.nn.relu)

        # actor
        policy_logits  = tf.layers.dense(shared_dense, action_dim)
        self.policy = tf.nn.softmax(policy_logits)

        # critic
        self.value_function = tf.layers.dense(shared_dense, 1)

        '''
        R : discounted return
        A : actions
        ADV : advantage estimator
        '''
        self.R = tf.placeholder(tf.float32, shape=[batch_size])
        self.A = tf.placeholder(tf.int32, shape=[batch_size])
        self.ADV = tf.placeholder(tf.float32, shape=[batch_size])

        #loss function
        value_loss = 0.5 * tf.reduce_mean(tf.square(tf.squeeze(self.value_function) - self.R))

        self.tst1 = tf.squeeze(self.value_function) - self.R
        self.tst2 = self.ADV

        action_ont_hot = tf.one_hot(self.A, action_dim, dtype=tf.float32)
        neg_log_policy = -tf.log(tf.clip_by_value(self.policy, 1e-7, 1))
        policy_loss = tf.reduce_mean(tf.reduce_sum(neg_log_policy*action_ont_hot, axis=1)*self.ADV)

        entropy = tf.reduce_mean(tf.reduce_sum(self.policy*neg_log_policy, axis=1))
        self.total_loss =  0.5 * value_loss + policy_loss +  0.0*entropy

        #trainning operator
        #"norm" remains unknown
        local_vars = tf.trainable_variables()
        gradients = tf.gradients(self.total_loss, local_vars)
        global_norm = tf.global_norm(gradients)
        gradients, _ = tf.clip_by_global_norm(gradients, global_norm)
    
        self.train_op = self.trainer.apply_gradients(zip(gradients, local_vars))

    def train(self):
        obs, actions, returns, advs = self.run()
        feed_dict = {self.x:obs, self.A: actions, self.R: returns, self.ADV: advs}
        self.sess.run(self.train_op, feed_dict=feed_dict)
        print(self.sess.run(self.tst1, feed_dict=feed_dict))
        print(self.sess.run(self.tst2, feed_dict=feed_dict))
        print("-------------------------------")

    def act(self, s):
        '''
        return an action according to actor (distributional)
        '''
        actions = self.sess.run(self.policy, feed_dict={self.x : [s]})
        return np.random.choice(len(actions[0]), p=actions[0]) 

    def run(self, n_steps=1):
        '''
        n_step return
        '''
        s = self.start_state
        done = False
        states = []
        rewards = []
        actions = []
        total_reward = self.start_reward

        for _ in range(n_steps):
            a = self.act(s)
            s_next, r, done, _ = self.env.step(a)
            states.append(s) 
            if done:
                r = -20
            rewards.append(r)
            actions.append(a)
            total_reward += r
            s = s_next
            if done:
                self.ep_nums += 1
                s = self.env.reset()
                global running_reward
                if running_reward == 0:
                    running_reward = total_reward
                else:
                    running_reward = running_reward * 0.95 + total_reward * 0.05

                if (self.ep_nums % 100 == 0):
                    print(running_reward)

                total_reward = 0
                break
        
        self.start_state = s
        self.start_reward = total_reward

        R = 0
        if not done:
            R = self.sess.run(self.value_function, feed_dict={self.x: [s]})
        
        n = len(states)
        v = self.sess.run(self.value_function, feed_dict={self.x: states})

        rewards = np.zeros(n)
        advs = np.zeros(n)
        actions = np.array(actions)
        '''
        R: n-step return
        v: value function
        rewards : discounted accumulative rewards
        adv: advantage estimator
        '''
        for i in range(n - 1, -1, -1):
            R = rewards[i] + self.gamma * R
            rewards[i] = R
            advs[i] = R - v[i]

        return states, actions, rewards, advs

def learn():
    sess = tf.Session()

    env = gym.make("CartPole-v0")
    N_F = env.observation_space.shape[0]
    N_A = env.action_space.n
    model = Model(sess, N_F, N_A, env=env)

    sess.run(tf.global_variables_initializer())

    for i in range(100000):
        
        model.train()

learn()