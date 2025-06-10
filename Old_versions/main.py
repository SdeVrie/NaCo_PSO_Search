# E2RPSO
from E2RPSO import E2RPSO
from RDPSO import RDPSO
from Exhaustive_search import exhaustive_search
from Random_search import random_search
from dPSO import run_dpso
from Continuous_grid import Grid
import csv


#Variables
SIDE = [100, 200]
agent_num = [3,6,9]
obstacles = [5, 15, 25]
agent_radius = 3
agent_half_size = 1
T = 1000
num_targets = [1,5,10]
obstacle_half_size = [1, 5]
target_half_size = 1
seeds = [683, 430, 836]
STEP_SIZE = 2

algorithms = [
    ("E2RPSO", E2RPSO),
    ("Stochastic_search", exhaustive_search),
    ("Random_search", random_search),
    ("dPSO", run_dpso),
    ("RDPSO", RDPSO)
]

# Open CSV file for writing
with open('results_table.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    # Write header row
    writer.writerow(['Algorithm', 'Targets_Found', 'Total_Distance', 'Steps', 'Side', 'Seed', 'Obstacles', 'Agents', 'Targets'])

    for side in SIDE:
        for seed in seeds:
            for ob in obstacles:
                for an in agent_num:
                    for nt in num_targets:
                        for algo_name, algo_func in algorithms:
                            grid = Grid(side, ob, nt, an, agent_half_size, agent_radius, obstacle_half_size, target_half_size, seed)
                            steps, targets_found, total_distance = algo_func(grid, T, STEP_SIZE)
                            
                            # Write data row
                            writer.writerow([algo_name, targets_found, total_distance, steps, side, seed, ob, an, nt])
                            csvfile.flush()
                            
                            print(f'{algo_name} -> targets_found: {targets_found}, total distance covered: {total_distance}, steps needed: {steps}')
                            print(f'Using variables -> Side: {side}, seed: {seed}, obstacles: {ob}, agents: {an}, targets: {nt}')
                            print("**********************************************************************************************************")
