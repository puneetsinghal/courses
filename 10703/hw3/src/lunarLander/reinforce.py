import sys
import argparse
import numpy as np
import tensorflow as tf
import keras
import gym
from IPython import embed
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from keras.callbacks import TensorBoard
except Exception as e:
    print("{} from keras.callback. This will prevent gathering data on tensorboard".format(e))

class Reinforce(object):
    # Implementation of the policy gradient method REINFORCE.

    def __init__(self, model, lr):
        self.model = model

        # TODO: Define any training operations and optimizers here, initialize
        #       your variables, or alternately compile your model here.  


    def train(self, env, gamma=1.0):
        # Trains the model on a single episode using REINFORCE.
        # TODO: Implement this method. It may be helpful to call the class
        #       method generate_episode() to generate training data.
        return

    def generate_episode(self, env, render=False):
        # Generates an episode by executing the current policy in the given env.
        # Returns:
        # - a list of states, indexed by time step
        # - a list of actions, indexed by time step
        # - a list of rewards, indexed by time step
        # TODO: Implement this method.
        numStates = env.observation_space.shape[0]
        numActions = env.action_space.n
        states = []
        actions = []
        rewards = []
        s = env.reset()
        done = False
        while not done:
            if render: 
                env.render()
            states.append(s)
            # a = np.argmax(self.model.predict(s.reshape(1,numStates)))
            # embed()
<<<<<<< HEAD
            prob = (1e6*self.model.predict(s.reshape(1,numStates))[0]).astype(int)
            prob = prob.astype(float)
            prob /= prob.sum()
            a = np.random.choice(numActions, 1, p=prob.tolist())[0]

=======
            prob = self.model.predict(s.reshape(1,numStates))[0]
            prob[0] += (1 - sum(prob))
            # prob /= prob.sum().astype(float)
            
            try:
                a = np.random.choice(numActions, 1, p=prob.tolist())[0]
            except:
                print(prob)
                for i in range(numActions):
                    if(np.isnan(prob[i])):
                        prob[i] = 0
                a = np.random.choice(numActions, 1, p=prob.tolist())[0]
>>>>>>> ea780d6c13f5805f3bb4ed4a3c4a7ac6ed773589
            s, r, done, _ = env.step(a)
            rewards.append(r)
            actions.append(a)
        if render:
            env.close()
        return states, actions, rewards

def parse_arguments():
    # Command-line flags are defined here.
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-config-path', dest='model_config_path',
                        type=str, default='LunarLander-v2-config.json',
                        help="Path to the model config file.")
    parser.add_argument('--num-episodes', dest='num_episodes', type=int,
                        default=50000, help="Number of episodes to train on.")
    parser.add_argument('--lr', dest='lr', type=float,
                        default=5e-4, help="The learning rate.")

    # https://stackoverflow.com/questions/15008758/parsing-boolean-values-with-argparse
    parser_group = parser.add_mutually_exclusive_group(required=False)
    parser_group.add_argument('--render', dest='render',
                              action='store_true',
                              help="Whether to render the environment.")
    parser_group.add_argument('--no-render', dest='render',
                              action='store_false',
                              help="Whether to render the environment.")
    parser.set_defaults(render=False)

    return parser.parse_args()


def main(args):
    # Parse command-line arguments.
    args = parse_arguments()
    model_config_path = args.model_config_path
    # num_episodes = args.num_episodes
    # lr = args.lr
    num_episodes = 50000
    lr = 5e-4
    render = args.render

    # Create the environment.
    env = gym.make('LunarLander-v2')
    
    # Load the policy model from file.
    with open(model_config_path, 'r') as f:
        model = keras.models.model_from_json(f.read())

    # TODO: Train the model using REINFORCE and plot the learning curve.
    Reinforce = Reinforce(model, lr)

if __name__ == '__main__':
    main(sys.argv)
