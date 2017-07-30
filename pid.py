#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 29 18:34:14 2017

@author: bill
"""
import gym
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

class Data():
    """tracks elements of the state"""
    def __init__(self):
        self.states = []
    
    def add(self,state):
        self.states.append(state)
        
    def graph(self):
        states = np.array(self.states).reshape(len(self.states),-1)
        plt.plot(states[:,0],label='x')
        plt.plot(states[:,1],label='y')
        plt.plot(states[:,2],label='vx')
        plt.plot(states[:,3],label='vy')
        plt.plot(states[:,4],label='theta')
        plt.plot(states[:,5],label='vtheta')
        plt.legend()
        plt.grid()
        plt.ylim(-1.1,1.1)
        plt.title('PID Control')
        plt.ylabel('Value')
        plt.xlabel('Steps')
        plt.show('pid.png')
    
def pid(state, params):
    """ calculates settings based on pid control """
    # PID parameters
    kp_alt = params[0]  # proportional altitude
    kd_alt = params[1]  # derivative altitude
    kp_ang = params[2]  # proportional angle
    kd_ang = params[3]  # derivative angle
    
    # Calculate setpoints (target values)
    alt_tgt = np.abs(state[0])
    ang_tgt = (.25*np.pi)*(state[0]+state[2])

    # Calculate error values
    alt_error = (alt_tgt - state[1])
    ang_error = (ang_tgt - state[4])
    
    # Use PID to get adjustments
    alt_adj = kp_alt*alt_error + kd_alt*state[3]
    ang_adj = kp_ang*ang_error + kd_ang*state[5]
        
    # Gym wants them as np array (-1,1)
    a = np.array([alt_adj, ang_adj])
    a = np.clip(a, -1, +1)
    
    # If the legs are on the ground we made it, kill engines
    if(state[6] or state[7]):
        a[:] = 0   
    return a

def run(params, env, verbose=False):
    """ runs an episode given pid parameters """
    data = Data() 
    done = False
    state = env.reset()
    if verbose:
        env.render()
        sleep(.005)
    data.add(state)
    total = 0
    while not done:
        a = pid(state,params)
        state,reward,done,_ = env.step(a)
        total += reward
        if verbose:
            env.render()
            sleep(.005)
        data.add(state)
    return total, data

def optimize(params, current_score, env, step):
    """ runs a step of randomized hill climbing """

    # add gaussian noise (less noise as n_steps increases)
    test_params = params + np.random.normal(0,20.0/step,size=params.shape)
    
    # test params over 5 trial avg
    scores = []
    for trial in range(5):
        score,_ = run(test_params,env)
        scores.append(score)
    avg = np.mean(scores)
    
    # update params if improved
    if avg > current_score:
        return test_params,avg
    else:
        return params,current_score
    
def main():
    # Setup environment
    env = gym.make('LunarLanderContinuous-v2')
    env._max_episode_steps = 300
   
    # Seed RNGs
    np.random.seed(0)
    env.seed(0)

    # Random Hill Climb over params
    params = np.array([0,0,0,0])
    score = -300 # bad starting score
    for steps in range(101):
        params,score = optimize(params,score,env,steps+1)
        if steps%10 == 0:
            print "Step:",steps,"Score:",score,"Params:",params

    # Get data for final run
    scores = []
    for trial in range(10):
        score, data = run(params, env, True)
        scores.append(score)
    env.close()
    print "Average Score:",np.mean(scores)
    data.graph()

if __name__ == '__main__':
    main()
