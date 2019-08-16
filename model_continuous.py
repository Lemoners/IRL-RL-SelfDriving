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
        '''
        self.ep_nums =  0
        self.timesteps = 0
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
        mu = tf.layers.dense(shared_dense, action_dim)
        sigma = tf.layers.dense(shared_dense, action_dim, tf.nn.softplus) + 1e-5

        norm_dist = tf.distributions.Normal(mu, sigma)

        self.action = tf.squeeze(norm_dist.sample(1), axis=0)
        self.action = tf.clip_by_value(self.action, env.action_space.low[0], env.action_space.high[0])

        # critic
        self.value_function = tf.layers.dense(shared_dense, 1)

        '''
        R : discounted return
        A : actions
        ADV : advantage estimator
        '''
        self.R = tf.placeholder(tf.float32, shape=[batch_size])
        self.A = tf.placeholder(tf.float32, shape=[batch_size])
        self.ADV = tf.placeholder(tf.float32, shape=[batch_size])

        #loss function
        value_loss = 0.5 * tf.reduce_mean(tf.square(tf.squeeze(self.value_function) - self.R))

        neg_log_policy = -norm_dist.log_prob(self.A)
        policy_loss = tf.reduce_mean(neg_log_policy*self.ADV)
        entropy = norm_dist.entropy()

        # self.t1 = neg_log_policy
        # self.t2 = self.ADV
        self.total_loss = 0.5 * value_loss + policy_loss + 0.01*entropy

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
        # print(self.sess.run(self.total_loss, feed_dict=feed_dict))
        # print(self.sess.run(self.t1, feed_dict=feed_dict))
        # print(self.sess.run(self.t2, feed_dict=feed_dict))
        # print("-------------")

    def act(self, s):
        '''
        return an action according to actor (distributional)
        '''
        return self.sess.run(self.action, feed_dict={self.x : [s]})

    def run(self, n_steps=5):
        '''
        n_step return
        '''
        global running_reward
        s = self.start_state
        done = False
        states = []
        rewards = []
        actions = []
        total_reward = self.start_reward

        for _ in range(n_steps):
            # if running_reward > 500:
            # self.env.render()
            s = list(np.squeeze(s))
            a = float(np.squeeze(self.act(s)))

            s_next, r, done, _ = self.env.step([a])
            r/=10

            states.append(s) 
            # if done:
            #     r = -20
            rewards.append(r)
            actions.append(a)
            total_reward += r
            s = s_next
            if done:
                self.ep_nums += 1
                s = self.env.reset()
                
                if running_reward == 0:
                    running_reward = total_reward
                else:
                    running_reward = running_reward * 0.9 + total_reward * 0.1

                if (self.ep_nums % 10 == 0):
                    print(running_reward)

    
                total_reward = 0
                break
        s = list(np.squeeze(s))

        self.start_state = s
        self.start_reward = total_reward


        R = 0
        if not done:
            R = self.sess.run(self.value_function, feed_dict={self.x: [s]})
        
        n = len(states)
        v = self.sess.run(self.value_function, feed_dict={self.x: states})

        rewards = np.array(rewards)
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
    env = gym.make("Pendulum-v0")
    N_F = env.observation_space.shape[0]
    # N_A = env.action_space.n
    model = Model(sess, N_F, 1, env=env)

    sess.run(tf.global_variables_initializer())

    for i in range(10000000):
        
        model.train()

learn()