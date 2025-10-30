import pandas as pd

market_trans_df = pd.read_csv(r'..\Processed data\market_transactions_commondebt_cleaned.csv')


# Load the Address_features dataset
address_features = pd.read_csv(r'..\Processed data\Address_apartment_cleaned.csv')

# Merge the datasets on 'address_id' and 'id'
df = market_trans_df.merge(address_features, left_on='address_id', right_on='id', how='left')

# Identify rows with NaN values after the merge
nan_rows = df[df.isna().any(axis=1)]
if not nan_rows.empty:
    print("Rows with NaN values after merge:")
    print(nan_rows)


# Option 2: Drop rows with NaN values (uncomment the line below if preferred)
df = df.dropna()

# Drop the 'id' column from the merged DataFrame
df = df.drop(columns=['id'])

# Print the first few rows of the merged DataFrame to verify
print(df.head())
# Print basic information about the dataset
print("\nDataset Info:")
print(df.info())

# Display summary statistics
print("\nSummary Statistics:")
print(df.describe())

# Check for any remaining missing values
print("\nMissing Values Count:")
print(df.isnull().sum())

# Display column names and data types
print("\nColumn Names and Data Types:")
for col in df.columns:
    print(f"{col}: {df[col].dtype}")

# Print dataset dimensions
print(f"\nDataset Shape: {df.shape} (rows x columns)")

# Turn bool to int
df = df * 1

df.to_csv(r'..\Processed data\combined_apartment_data.csv', index=False, sep=";", decimal=",")



