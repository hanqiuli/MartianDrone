import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Define the scores
scores = pd.DataFrame({
    'Option': ["Tiltwing", "Tiltrotor", "Blimp", "Hexacopter"],
    'Energy': [3, 1, 4, 3],
    'Controllability': [3, 3, 2, 4],
    'Mass': [4, 3, 3, 4],
    'Design complexity': [2, 3, 4, 2],
    'V_maxrange': [4, 4, 3, 4]
})

# Define the total scores from excel
total_excel_scores = np.array([310, 240, 325, 335]) / 4

# Initial weights as percentages (they sum to 1)
initial_weights = np.array([0.35, 0.25, 0.15, 0.15, 0.10])
perturbation_range = 0.5  # ±50%

# Number of simulations
num_simulations = 10000

# Function to perform Monte Carlo simulation
def monte_carlo_simulation(weights, scores, criteria):
    simulation_results = np.zeros((num_simulations, len(scores)))
    for i in range(num_simulations):
        # Perturb weights within ±50% of their initial values
        perturbed_weights = weights * (1 + np.random.uniform(-perturbation_range, perturbation_range, weights.shape))
        # Normalize the perturbed weights to sum to 1
        perturbed_weights /= perturbed_weights.sum()
        # Calculate the weighted scores
        weighted_scores = (np.dot(scores[criteria], perturbed_weights) * 100) / 4
        # Store the results
        simulation_results[i, :] = weighted_scores
    # Convert results to a DataFrame
    simulation_df = pd.DataFrame(simulation_results, columns=scores['Option'])
    return simulation_df

# Criteria list
criteria = ['Energy', 'Controllability', 'Mass', 'Design complexity', 'V_maxrange']

# Perform simulation for the initial case
simulation_df_initial = monte_carlo_simulation(initial_weights, scores, criteria)

# Analyze the results
summary_stats_initial = simulation_df_initial.describe().drop('count').round(2)

# Export summary statistics to CSV
summary_stats_initial.to_csv('TradeoffTools/summary_stats_initial.csv')

print("Summary statistics of the Monte Carlo simulation:")
print(summary_stats_initial)

# Set larger font size for plots
plt.rcParams.update({'font.size': 14})

# Plot the initial boxplot
plt.figure(figsize=(12, 8))
ax = simulation_df_initial.boxplot()
# Add the total scores from excel as scatter plot
for i, score in enumerate(total_excel_scores):
    plt.scatter(i + 1, score, color='red', marker='x', zorder=5, label='Actual Total Score' if i == 0 else "")
# Labeling
plt.xlabel('Options')
plt.ylabel('Total score')
plt.ylim(0, 100)
plt.legend()
plt.savefig('plots/Sensitivity/initial_boxplot.png')
plt.show()

# List of criteria to eliminate one at a time
all_simulation_dfs = {}

# Perform simulation by eliminating each criterion
for criterion in criteria:
    reduced_scores = scores.drop(columns=[criterion])
    reduced_criteria = [c for c in criteria if c != criterion]
    remaining_weights = np.delete(initial_weights, criteria.index(criterion))
    redistributed_weights = remaining_weights + initial_weights[criteria.index(criterion)] / len(remaining_weights)
    simulation_df = monte_carlo_simulation(redistributed_weights, reduced_scores, reduced_criteria)
    all_simulation_dfs[criterion] = simulation_df
    summary_stats = simulation_df.describe().drop('count').round(2)
    summary_stats.to_csv(f'TradeoffTools/summary_stats_without_{criterion}.csv')

    print(f"Summary statistics of the Monte Carlo simulation after eliminating {criterion}:")
    print(summary_stats)

# Plot the boxplots for each configuration
plt.figure(figsize=(20, 12))

plt.subplot(2, 3, 1)
simulation_df_initial.boxplot()
plt.xlabel('Options')
plt.ylabel('Total score')
plt.title('Initial Configuration')
plt.ylim(0, 100)
for i, score in enumerate(total_excel_scores):
    plt.scatter(i + 1, score, color='red', marker='x', zorder=5)

for idx, (criterion, df) in enumerate(all_simulation_dfs.items()):
    plt.subplot(2, 3, idx + 2)
    df.boxplot()
    plt.xlabel('Options')
    plt.ylabel('Total score')
    plt.title(f'After eliminating {criterion}')
    plt.ylim(0, 100)

plt.tight_layout()
plt.savefig('plots/Sensitivity/boxplots_all_configurations.png')
plt.show()

# Plot the boxplots excluding initial configuration, V_maxrange, and Mass
plt.figure(figsize=(20, 6))

# Exclude initial, V_maxrange, and Mass
excluded_criteria = ['V_maxrange', 'Mass']

remaining_criteria = [criterion for criterion in criteria if criterion not in excluded_criteria]

for idx, criterion in enumerate(remaining_criteria):
    simulation_df = all_simulation_dfs[criterion]
    plt.subplot(1, 3, idx + 1)
    simulation_df.boxplot()
    plt.xlabel('Options')
    plt.ylabel('Total score')
    plt.title(f'After eliminating {criterion}')
    plt.ylim(0, 100)


plt.tight_layout()
plt.savefig('plots/Sensitivity/boxplots_excluding_initial_vmax_mass.png')
plt.show()
