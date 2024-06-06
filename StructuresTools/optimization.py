import pandas as pd
import os


filepath = 'configurations.csv'
# Load the data from the CSV file with pandas
data = pd.read_csv(filepath)

#sort the data by the score column
sorted_data = data.sort_values('composite_score', ascending=True)


# Print the top 5 configurations
print(sorted_data.head(50))





