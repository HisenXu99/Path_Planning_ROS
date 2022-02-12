import random
from env_mbot import envmodel
# import tensorflow.compat.v1 as tf
# import A2C
import  numpy as np
import time

LR_A = 0.01    # learning rate for actor
LR_C = 0.01     # learning rate for critic
MAX_EPISODE = 4000
DISPLAY_REWARD_THRESHOLD = 0.1  # renders environment if total episode reward is greater then this threshold
RENDER = False


env=envmodel()


action_dict = {0: [1.0, -1.0], 1: [1.0, -0.5], 2: [1.0, 0.0],
               3: [1.0, 0.5], 4: [1.0, 1.0], 5: [0.5, -1.0],
               6: [0.5, 0.0], 7: [0.5, 1.0], 8: [0.0, -1.0],
               9: [0.0, 0.0], 10: [0.0, 1.0]}

# sess = tf.Session()
# actor = A2C.Actor(sess, n_features=4, n_actions=5, lr=LR_A)
# critic = A2C.Critic(sess, n_features=4, lr=LR_C)
# saver = tf.train.Saver()  #声明ta.train.Saver()类用于保存
# sess.run(tf.global_variables_initializer())


# def train():
#     for i_episode in range(MAX_EPISODE):
#         env.reset()
#         observation = np.ones(4)
#         observation=observation.reshape(-1)
#         track_r = []
#         for i in range(1000):
#             action = actor.choose_action(observation)
#             reward=env.step(action)
#             observation_, reward, done, info = env.last()


#             td_error = critic.learn(observation, reward, observation_)  # gradient = grad[r + gamma * V(s_) - V(s)]
#             actor.learn(observation, action, td_error)     # true_gradient = grad[logPi(s,a) * td_error]
#             observation=observation_

#             if done:
#                 ep_rs_sum=sum(track_r)
#                 print("episode:", i_episode, "  reward:", ep_rs_sum)
#                 if i_episode > 3900: RENDER = True  # rendering
#                 break

#     print("123")
#     save_path = saver.save(sess,'save/filename.ckpt')#保存路径为相对路径的save文件夹,保存名为filename.ckpt
#     print ("[+] Model saved in file: %s" % save_path)




if __name__ == '__main__':
    env.reset_env(start=[5.0, 5.0], goal=[-5.0,-5.0])
    for i_episode in range(MAX_EPISODE):
        action = action_dict[random.randint(0, 9)]
        print(action)
        reward=env.step(action)
        time.sleep(0.5)
    pass