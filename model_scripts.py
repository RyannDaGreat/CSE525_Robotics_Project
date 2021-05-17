# -*- coding: utf-8 -*-
"""
Created on Thu Apr 29 12:05:18 2021

@author: Mohiiiiiib
"""

import numpy as np
import pandas as pd
import os
import json
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.autograd as autograd
import torch.optim as optim
import torch.nn.functional as F 


# State + State Prime -> Action Neural Net Model
# 3 FC Layers to an output (with Sigmoid activations)
class ssa_model(nn.Module):
    def __init__(self):
        super(ssa_model, self).__init__()
        
        self.fc1 = nn.Linear(8, 8)
        self.sig1 = nn.ReLU()
        
        self.fc2 = nn.Linear(8, 8)
        self.sig2 = nn.ReLU()
        
        self.fc3 = nn.Linear(8, 3)
        self.sig3 = nn.ReLU()
        

    def forward(self, ss):
        res1 = self.fc1(ss)
        res2 = self.sig2(res1)
        
        res3 = self.fc2(res2)
        res4 = self.sig2(res3)
        
        res5 = self.fc3(res4)
        res5 = self.sig3(res5)
        
        return res5
        
# State + Action -> State Prime Neural Net Model
# 3 FC Layers to an output (with Sigmoid activations)
class sas_model(nn.Module):
    def __init__(self):
        super(sas_model, self).__init__()
        
        self.fc1 = nn.Linear(5, 5)
        self.sig1 = nn.Sigmoid()
        
        self.fc2 = nn.Linear(5, 5)
        self.sig2 = nn.Sigmoid()
        
        self.fc3 = nn.Linear(5, 4)
        
    def forward(self, sa):
        res1 = self.fc1(sa)
        res2 = self.sig1(res1)
        
        res3 = self.fc2(res2)
        res4 = self.sig2(res3)
        
        res5 = self.fc3(res4)
        
        return res5
        
    
def convert_action(action):
    if action == 'left':
        return [1, 0, 0]
    if action == 'right':
        return [0, 0, 1]
    else:
        return [0, 1, 0]


def get_state_entry(fname):
    with open(fname) as fp:
        state_dict = json.load(fp)
        
    state_entry = []
    state_entry.extend(state_dict['robot_center_in_cm'])
    state_entry.extend(state_dict['robot_forward_vec'])
    
    return state_entry
     
   
def load_data():
    # Load the dataframe using pandas open_csv with a /t delimiter
    # Seperate into states, next_states, and actions
    csv_file = os.path.join('data', 'transitions.tsv')
    tsv_data = pd.read_csv(csv_file, sep='\t')
    
    rows = tsv_data.shape[0]
    
    actions = []
    states = []
    state_primes = []
    
    for i in range(rows):
        entry_a = tsv_data['action'][i]
        entry_s = tsv_data['s'][i]
        entry_s_prime = tsv_data['s_prime'][i]
        
        split_s = entry_s.split('/')
        split_s_prime = entry_s_prime.split('/')
        
        if int(split_s[3]) + 1 == int(split_s_prime[3]):
            state_path = os.path.join(split_s[1], split_s[2], split_s[3], 'state.json')
            state_prime_path = os.path.join(split_s_prime[1],
                                            split_s_prime[2],
                                            split_s_prime[3],
                                            'state.json')
            
            states.append(get_state_entry(state_path))
            state_primes.append(get_state_entry(state_prime_path))
            
            actions.append(convert_action(entry_a))
    
    return np.array(states), np.array(state_primes), np.array(actions)
        

def load_data_ssa():
    states, state_primes, actions = load_data()

    # Assemble together data into 
    print(states.shape)
    print(state_primes.shape)
    
    data = np.empty([5389, 8])
    data[:, :4] = states
    data[:, 4:] = state_primes
    print(data.shape)
    
    labels = actions

    return data, labels


def load_data_sas():
    states, state_primes, actions = load_data()

    # Assemble together data into 
    print(states.shape)
    print(state_primes.shape)
    
    data = np.empty([5389, 5])
    data[:, :4] = states
    data[:, 4] = actions
    print(data.shape)
    
    labels = state_primes

    return data, labels


"""
def fix_pred(pred):
    print(pred.shape)
    for i in range(pred.size):
        if pred[i] < -0.5:
            pred[i] = -1
        if pred[i] > 0.5:
            pred[i] = 1
        else:
            pred[i] = 0
    return pred
"""


# TODO: Change the label to be a one-hot encoding instead of -1, 0, 1
def train_ssa_agent(data, labels):
    train_data = data[:4500, :]
    test_data = data[4500:, :]
    train_labels = labels[:4500]
    test_labels = labels[4500:]
    
    ssa = ssa_model()
    ssa = ssa.double()
    loss_func = nn.MSELoss()
    net_opt = optim.Adam(ssa.parameters(), lr=0.0001)
    train_res = []
    test_res = []
    episodes = 300
    for n in range(episodes):
        print(n)
        randomized_inds = np.arange(train_data.shape[0])
        np.random.shuffle(randomized_inds)
        for i in randomized_inds:
            entry = train_data[i, :]
            label = train_labels[i]
            
            entry = torch.from_numpy(entry)
            entry = entry.double()
            label = torch.from_numpy(np.array(label))
            label = label.double()
            
            pred = ssa(entry)
            loss = loss_func(pred, label)
            net_opt.zero_grad()
            loss.backward()
            net_opt.step()
            
        if (n + 1) % 25 == 0:
            ssa.eval()
            
            torch_data = torch.from_numpy(train_data)
            
            pred = ssa(torch_data)
            pred = pred.detach().cpu().numpy()
            alt_pred = pred
            # pred = fix_pred(pred[:, 0])
            
            # acc = np.sum(np.equal(pred, train_labels)) / train_labels.size
            
            acc = np.sum(np.equal(np.argmax(pred, axis=1), 
                                  np.argmax(train_labels, axis=1))) / train_labels.size
            print(acc)
            train_res.append(acc)
            
            
            torch_data = torch.from_numpy(test_data)
            
            pred = ssa(torch_data)
            pred = pred.detach().cpu().numpy()
            alt_pred = pred
            # pred = fix_pred(pred[:, 0])
            
            acc = np.sum(np.equal(np.argmax(pred, axis=1), 
                                  np.argmax(test_labels, axis=1))) / test_labels.size
            print(acc)
            test_res.append(acc)
            
            ssa.train()

    return pred, ssa, train_res, test_res


def train_sas_agent(data, labels):
    sas = sas_model()
    sas = sas.double()
    loss_func = nn.MSELoss()
    net_opt = optim.Adam(sas.parameters())
    episodes = 100
    for n in range(episodes):
        print(n)
        randomized_inds = np.arange(data.shape[0])
        np.random.shuffle(randomized_inds)
        for i in randomized_inds:
            entry = data[i, :]
            label = labels[i, :]
            
            entry = torch.from_numpy(entry)
            entry = entry.double()
            label = torch.from_numpy(label)
            label = label.double()
            
            pred = sas(entry)
            loss = loss_func(pred, label)
            net_opt.zero_grad()
            loss.backward()
            net_opt.step()

    sas.eval()
    
    torch_data = torch.from_numpy(data)
    
    pred = sas(torch_data)
    pred = pred.detach().cpu().numpy()    
    acc = np.sum(np.equal(pred, labels)) / np.size(labels)
    print(acc)

    return pred, sas


data, labels = load_data_ssa()
a_pred, ssa, train_res, test_res = train_ssa_agent(data, labels)

plt.figure()
plt.plot(np.array(range(train_res.size)), train_res)

plt.figure()
plt.plot(np.array(range(test_res.size)), test_res)

# data, labels = load_data_sas()
# s_pred, sas = train_sas_agent(data, labels)