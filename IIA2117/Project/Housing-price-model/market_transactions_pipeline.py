import pandas as pd


# Load the CSV file into a DataFrame
df = pd.read_csv(r'..\raw_data_2024_10\market_transactions.csv')


# Filter the DataFrame to include only rows where 'official_date' is after 2019
df['official_date'] = pd.to_datetime(df['official_date'], errors='coerce')

# Remove rows with invalid data in 'official_date', 'address_id', or 'official_price'
df = df.dropna(subset=['official_date', 'address_id', 'official_price'])

#Remove any housing sold before 2019
df = df[df['official_date'] > '2022-12-31']

"""
# Add also shared_cost_monthly and municipal_charges_yearly
df = df.dropna(subset=['shared_cost_monthly', 'municipal_charges_yearly'])
df = df[['shared_cost_monthly', 'municipal_charges_yearly', 'official_date', 'address_id', 'official_price']]
"""

# Add common debt to official price
df.official_price = df.official_price + df.common_debt

# Keep only the specified columns
df = df[['official_date', 'address_id', 'official_price']]

# Rename 'official_date' to 'sold_date'
df = df.rename(columns={'official_date': 'sold_date'})


# Add month sold and season sold
def date_to_month(date):
    title = "sold_"
    months = [None, "january", "february", "march", "april", "may", "june", \
              "july", "august", "september", "october", "november", "december"]
    return title + months[date.month]

def date_to_season(date):
    title = "sold_"
    seasons = ["winter", "spring", "summer", "autumn"]
    #print(date.month, seasons[(date.month % 12) // 3])
    return title + seasons[(date.month % 12) // 3]

df["sold_month"] = df['sold_date'].apply(date_to_month)
df["sold_season"] = df['sold_date'].apply(date_to_season)
# One-hot encoding
month_encoded = pd.get_dummies(df['sold_month'], drop_first=False)
season_encoded = pd.get_dummies(df['sold_season'], drop_first=False)
df = pd.concat([df.drop(columns=['sold_month']), month_encoded], axis=1)
df = pd.concat([df.drop(columns=['sold_season']), season_encoded], axis=1)

# Check for NaN values in the dataset
if df.isnull().values.any():
    print("There are NaN values in the dataset.")
    print(df.isnull().sum())
else:
    print("No NaN values found in the dataset.")

#save the modified DataFrame back to a CSV file
df.to_csv(r'..\Processed data\market_transactions_commondebt_cleaned.csv', index=False)