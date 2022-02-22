import tensorflow.compat.v1 as tf
import numpy as np
import random
from env_mbot import envmodel
import A2C



LR_A = 0.001    # learning rate for actor
LR_C = 0.01     # learning rate for critic
MAX_EPISODE = 100
DISPLAY_REWARD_THRESHOLD = 0.1  # renders environment if total episode reward is greater then this threshold
RENDER = False



env=envmodel()


action_dict = {0: [1.0, -1.0], 1: [1.0, -0.5], 2: [1.0, 0.0],
               3: [1.0, 0.5], 4: [1.0, 1.0], 5: [0.5, -1.0],
               6: [0.5, 0.0], 7: [0.5, 1.0], 8: [0.0, -1.0],
               9: [0.0, 0.0], 10: [0.0, 1.0]}

sess = tf.Session()

model_path = 'save_/filename.ckpt'#保存路径为相对路径的save文件夹,保存名为filename.ckpt

actor = A2C.Actor(sess, n_features=6, n_actions=11, lr=LR_A)
critic = A2C.Critic(sess, n_features=6, lr=LR_C)
saver = tf.train.Saver()
load_path = saver.restore(sess, model_path)
# actor = A2C.Actor(sess, n_features=4, n_actions=5, lr=LR_A)
# critic = A2C.Critic(sess, n_features=4, lr=LR_C)
# sess.run(tf.global_variables_initializer())





def train():
    for i_episode in range(MAX_EPISODE):
        agent_start_x=random.randint(-10, 10)
        agent_start_y=random.randint(-10, 10)
        goal_x=random.randint(-10, 10)
        goal_y=random.randint(-10, 10)
        env.reset_env(start=[agent_start_x,agent_start_y],goal=[goal_x,goal_y])
        observation = np.ones(6)
        observation[0]=agent_start_x
        observation[1]=agent_start_y
        observation[2]=goal_x
        observation[3]=goal_y
        observation=observation.reshape(-1)
        track_r = []
        for i in range(300):
            action = actor.choose_action(observation)
            env.step(action_dict[action])
            observation_,reward,done = env.get_env()

            # td_error = critic.learn(observation, reward, observation_)  # gradient = grad[r + gamma * V(s_) - V(s)]
            # actor.learn(observation, action, td_error)     # true_gradient = grad[logPi(s,a) * td_error]
            observation=observation_
            track_r.append(reward)

            if done or i>=299:
                ep_rs_sum=sum(track_r)
                print("episode:", i_episode, "  reward:", ep_rs_sum)
                break


if __name__ == '__main__':
    train()
    pass