import pandas as pd


# Load the CSV file into a DataFrame
df = pd.read_csv(r'raw_data_2024_10\address.csv')
print(df.shape[0])
