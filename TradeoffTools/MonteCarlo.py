import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Define the scores
scores = pd.DataFrame({
    'Option': ["Tiltwing", "Tiltrotor", "Blimp", "Hexacopter"],
    'Power': [3, 1, 4, 3],
    'Controllability': [3, 3, 2, 4],
    'Mass': [4, 3, 3, 4],
    'Design complexity': [2, 3, 4, 3],
    'V_maxrange': [4, 4, 3, 4]
})

# Initial weights as percentages (they sum to 1)
initial_weights = np.array([0.35, 0.25, 0.15, 0.15, 0.10])
perturbation_range = 0.4  # ±40%

# Number of simulations
num_simulations = 10000

# Initialize an array to store the simulation results
simulation_results = np.zeros((num_simulations, len(scores)))

# Perform the Monte Carlo simulation
for i in range(num_simulations):
    # Perturb weights within ±40% of their initial values
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

# now, we also see the scores eliminating V_maxrange

# Define the scores
scores = pd.DataFrame({
    'Option': ["Tiltwing", "Tiltrotor", "Blimp", "Hexacopter"],
    'Power': [3, 1, 4, 3],
    'Controllability': [3, 3, 2, 4],
    'Mass': [4, 3, 3, 4],
    'Design complexity': [2, 3, 4, 3]
})

# Initial weights as percentages (they sum to 1)

initial_weights = np.array([0.35, 0.25, 0.20, 0.20])

# Now we just plot the results without simulating again
# Calculate the weighted scores
scores['Weighted_Score'] = (
    scores['Power'] * initial_weights[0] +
    scores['Controllability'] * initial_weights[1] +
    scores['Mass'] * initial_weights[2] +
    scores['Design complexity'] * initial_weights[3]
)

# Scale the scores so that they sum to 100
total_score = scores['Weighted_Score'].sum()
scores['Scaled_Score'] = (scores['Weighted_Score'] / total_score) * 100

# Plot the distribution of scores for each option as a bar chart

plt.figure(figsize=(12, 8))
plt.bar(scores['Option'], scores['Scaled_Score'])
plt.xlabel('Options')
plt.ylabel('Weighted Score')
plt.title('Weighted Scores of Different Drone Options, after eliminating V_maxrange')
plt.show()