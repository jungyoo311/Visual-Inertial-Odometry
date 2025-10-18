import pandas as pd
import matplotlib.pyplot as plt

# Read the data from the CSV file
try:
    df = pd.read_csv("feature_stats.csv")
except FileNotFoundError:
    print("Error: feature_stats.csv not found. Did you run the C++ program?")
    exit()

# Create the plot
plt.figure(figsize=(12, 7))

plt.plot(df['Frame'], df['Green'], label='Tracked Features (Green)', color='green', marker='.')
plt.plot(df['Frame'], df['Blue'], label='New Features (Blue)', color='blue', marker='.')
plt.plot(df['Frame'], df['Red'], label='Lost Features (Red)', color='red', marker='.')

# Add titles and labels for clarity
plt.title('Feature Tracking Statistics Over Time')
plt.xlabel('Frame Number')
plt.ylabel('Number of Features')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Save the plot to a file and show it
plt.savefig('feature_trends.png')
plt.show()

print("Plot saved to feature_trends.png")