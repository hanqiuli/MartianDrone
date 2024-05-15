import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Define the scores
scores = pd.DataFrame({
    'Option': ["Tiltwing", "Tiltrotor", "Blimp", "Hexacopter"],
    'Power': [3, 2, 4, 3],
    'Controllability': [3, 3, 2, 4],
    'Mass': [4, 3, 3, 4],
    'Design complexity': [2, 3, 4, 3],
    'V_maxrange': [4, 4, 3, 4]
})

# Initial weights as percentages (they sum to 1)
initial_weights = np.array([0.35, 0.25, 0.15, 0.15, 0.10])
perturbation_range = 0.3  # ±30%

# Number of simulations
num_simulations = 10000

# Initialize an array to store the simulation results
simulation_results = np.zeros((num_simulations, len(scores)))

# Perform the Monte Carlo simulation
for i in range(num_simulations):
    # Perturb weights within ±30% of their initial values
    perturbed_weights = initial_weights * (1 + np.random.uniform(-perturbation_range, perturbation_range, initial_weights.shape))
    
    # Normalize the perturbed weights to sum to 1
    perturbed_weights /= perturbed_weights.sum()
    
    # Calculate the weighted scores
    scores['Random_Weighted_Score'] = (
        scores['Power'] * perturbed_weights[0] +
        scores['Controllability'] * perturbed_weights[1] +
        scores['Mass'] * perturbed_weights[2] +
        scores['Design complexity'] * perturbed_weights[3] +
        scores['V_maxrange'] * perturbed_weights[4]
    )
    
    # Scale the scores so that they sum to 100
    total_score = scores['Random_Weighted_Score'].sum()
    scores['Scaled_Score'] = (scores['Random_Weighted_Score'] / total_score) * 100
    
    # Store the results
    simulation_results[i, :] = scores['Scaled_Score']

# Convert results to a DataFrame
simulation_df = pd.DataFrame(simulation_results, columns=scores['Option'])

# Analyze the results
summary_stats = simulation_df.describe()

print("Summary statistics of the Monte Carlo simulation:")
print(summary_stats)

# Plot the distribution of scores for each option as box plot
plt.figure(figsize=(12, 8))
simulation_df.boxplot()
plt.xlabel('Options')
plt.ylabel('Weighted Score')
plt.title('Box Plot of Weighted Scores from Monte Carlo Simulation')
plt.show()

# Optionally, plot the rankings distribution as box plot
rankings = simulation_df.rank(axis=1, ascending=False)

print("\nSummary statistics of the rankings from the Monte Carlo simulation:")
rank_summary = rankings.describe()
print(rank_summary)

# Plot the ranking distribution for each option as box plot
plt.figure(figsize=(12, 8))
rankings.boxplot()
plt.xlabel('Options')
plt.ylabel('Rank')
plt.title('Box Plot of Rankings from Monte Carlo Simulation')
plt.gca().invert_yaxis()  # Invert y-axis to have rank 1 at the top
plt.show()