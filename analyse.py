import pandas as pd

file_path = 'results_table.csv' 
df = pd.read_csv(file_path)

######################################################
#For obstacles
######################################################

avg_steps = df.groupby(['Algorithm', 'Obstacles'])['Steps'].agg(['mean', 'std', 'min', 'max']).reset_index()

pivot_table = avg_steps.pivot(index='Algorithm', columns='Obstacles', values='mean')

print("Average Steps per Algorithm per Number of Obstacles:")
print(pivot_table)

######################################################
#For targets
######################################################

avg_steps = df.groupby(['Algorithm', 'Targets'])['Steps'].agg(['mean', 'std', 'min', 'max']).reset_index()

pivot_table = avg_steps.pivot(index='Algorithm', columns='Targets', values='mean')

print("Average Steps per Algorithm per Number of targets:")
print(pivot_table)

######################################################
#For agents
######################################################

avg_steps = df.groupby(['Algorithm', 'Agents'])['Steps'].agg(['mean', 'std', 'min', 'max']).reset_index()

# Pivot based on mean values
pivot_table = avg_steps.pivot(index='Algorithm', columns='Agents', values='mean')

print("Average Steps per Algorithm per Number of Agents:")
print(pivot_table)


######################################################
#For obstacles
######################################################

avg_steps = df.groupby(['Algorithm', 'Obstacles'])['Total_Distance'].agg(['mean', 'std', 'min', 'max']).reset_index()

pivot_table = avg_steps.pivot(index='Algorithm', columns='Obstacles', values='mean')

print("Average Distance per Algorithm per Number of Obstacles:")
print(pivot_table)

######################################################
#For targets
######################################################

avg_steps = df.groupby(['Algorithm', 'Targets'])['Total_Distance'].agg(['mean', 'std', 'min', 'max']).reset_index()

pivot_table = avg_steps.pivot(index='Algorithm', columns='Targets', values='mean')

print("Average Distance per Algorithm per Number of targets:")
print(pivot_table)

######################################################
#For agents
######################################################

avg_steps = df.groupby(['Algorithm', 'Agents'])['Total_Distance'].agg(['mean', 'std', 'min', 'max']).reset_index()

# Pivot based on mean values
pivot_table = avg_steps.pivot(index='Algorithm', columns='Agents', values='mean')

print("Average Distance per Algorithm per Number of Agents:")
print(pivot_table)

