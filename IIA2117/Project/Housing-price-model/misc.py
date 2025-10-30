import pandas as pd

df = pd.read_csv(r'C:\Users\Bjorn\OneDrive - ZEM AS\Studier\Multivariate Data-Analysis\DA-3\Cleanup V1\combined_data.csv')

# Get the total number of rows and columns
print(f"Total rows: {df.shape[0]}, Total columns: {df.shape[1]}")

# Display the first few rows of the dataframe
print("First few rows of the dataframe:")
print(df.head())

# Check for missing values in the dataset
missing_values = df.isnull().sum()
print("Missing values in each column:")
print(missing_values)

# Check the data types of each column
print("Data types of each column:")
print(df.dtypes)

# Check for duplicate rows
duplicate_rows = df.duplicated().sum()
# Remove duplicate rows
df = df.drop_duplicates()
print(f"Number of duplicate rows removed: {duplicate_rows}")

# Check if the dataset is ready for regression analysis
if missing_values.sum() > 0:
    print("The dataset contains missing values. Consider handling them before analysis.")
if duplicate_rows > 0:
    print("The dataset contains duplicate rows. Consider removing them.")

# Get an overview of unique occurrences of 'bygningstypekode' and their counts
bygningstypekode_counts = df['bygningstypekode'].value_counts()
print(bygningstypekode_counts)


df.to_csv(r'C:\Users\Bjorn\OneDrive - ZEM AS\Studier\Multivariate Data-Analysis\DA-3\Cleanup V1\finished_dataset_oslo.csv', index=False)
