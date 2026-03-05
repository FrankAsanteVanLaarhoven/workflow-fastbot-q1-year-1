#!/usr/bin/env python3
import gymnasium as gym
import torch
from fsrl.agent import PPOLagrangianAgent
from fsrl.utils.exp_util import auto_name
from fsrl.utils.net.common import ActorProb, Critic

def train():
    # Placeholder for custom hospital environment
    env = gym.make("PointMaze_UMaze-v3") 
    
    # Configure Safe RL Agent (PPO-Lagrangian)
    agent = PPOLagrangianAgent(
        env,
        cost_limit=1.0,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )
    
    print("Starting Safe RL Training...")
    agent.learn(total_steps=100000)
    agent.save("hospital_safe_vla_model.pt")

if __name__ == "__main__":
    train()
