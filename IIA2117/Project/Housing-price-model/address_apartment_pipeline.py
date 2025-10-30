import pandas as pd


# Load the CSV file into a DataFrame
df = pd.read_csv(r'..\raw_data_2024_10\address.csv')

# print(df['poststed'].unique())
# print(df['bygningstypekode'].unique())
# print(df['naringsgruppekode'].unique())
# print(df['bruksenhettypekode'].unique())


df = df[
(df['poststed'].str.lower() == 'oslo') & 
#Bygningstypekoder: 111 = Enebolig, 112 = enebolig m/sokkelleil., 121 = Tomannsbolig, vertikaldelt, 122 = Tomannsbolig, horisontaldelt
# 131 = Rekkehus, 136 = Andre småhus m/3 boliger el fl, 141 = Stort frittliggende boligbygg pÃ¥ 2 etg, 142 =Stort frittliggende boligbygg på 3 og 4 etg.,
# 143 = Stort frittliggende boligbygg på 5 etg. el. mer,146 = Store sammenb. boligbygg på 5 etg. el. mer
(df['bygningstypekode'].notnull() & df['bygningstypekode'].isin([111, 112, 121, 122, 131, 136, 141, 142, 143, 146])) &
(df['naringsgruppekode'] == 'X') & #X is for private property
(df['bruksenhettypekode'] == 'B') #only allow approved units
]

# Drop rows where 'bruksaerealbruksenhet', 'heating_score', 'energy_score', are not valid
df = df.dropna(subset=['bruksaerealbruksenhet', 'heating_score', 'energy_score'])

# Drop rows where "unit_type" is not apartment
df = df[df.unit_type == "apartment"]


# List of columns to remove
columns_to_remove = [
'festenr', 'prom', 'matrikkel', 'bygningstype', 'internideiendom', 'bolignr',
'naringsgruppe', 'eiendomareal', 'postnr', 'internidadresse', 'gardsnr',
'kommunenr', 'bruksnr', 'antallboenheter', 'bruksarealbolig',
'bruksarealannet', 'bruksarealtotalt', 'seksjonsnr', 'internidborettsandel',
'organisasjonsnr', 'borettslagetsnavn', 'borettsandel', 'eiendomadressegatenr',
'eiendomadressegatenavn', 'eiendomadressehusnr', 'eiendomadressebokstav',
'bruksenhettype', 'coordinates', 'internidbygning', 'bygningsnr',
'bruttoarealbolig', 'bruttoarealannet', 'bruttoarealtotalt', 'tattibrukdato',
'internidbruksenhet', 'is_approved', 'is_active', 'updated_date', 'slug',
'grunnkretsnr', 'grunnkrets', 'unit_type_group', 'source', 'parkering',
'ad_category', 'owner', 'registrertlandbruksregisteret',
'registrertkulturminnebygning', 'registrertgrunnforurensning',
'registrertkulturminneeiendom', 'registrertgrunnerverv', 'registrertbodriveplikt',
'county_id', 'soverom', 'id_matrikkelen_adresse', 'id_matrikkelen_bruksenhet',
'id_matrikkelen_bygning', 'id_matrikkelen_eiendom', 'coordinates_geom',
'coord_x', 'coord_y', 'matrikkel2019', 'matrikkel2023', 'owner_share',
'coordinates', 'private_ad_url', 'balcony_size', 
'bydel', 'poststed', 'naringsgruppekode','bruksenhettypekode', 'unit_type'
]

# Remove the specified columns
df = df.drop(columns=columns_to_remove)



# Get an overview of unique occurrences of 'bygningstypekode' and their counts
bygningstypekode_counts = df['bygningstypekode'].value_counts()
print(bygningstypekode_counts)


# Change values of 'harheis' to 0 and 1
df['harheis'] = df['harheis'].map({'N': 0, 'J': 1})

# Normalize 'balkong' values to lowercase and assign 0 or 1 based on the condition
df['balkong'] = df['balkong'].str.lower().apply(lambda x: 1 if x == 't' else 0)


# Map 'energy_score' from letter grades to integers
energy_score_mapping = {'G': 1, 'F': 2, 'E': 3, 'D': 4, 'C': 5, 'B': 6, 'A': 7}

# Filter valid energy scores and map them to integers
df = df[df['energy_score'].isin(energy_score_mapping.keys())]
df['energy_score'] = df['energy_score'].map(energy_score_mapping)
# Cast 'heating_score' as integer
df['heating_score'] = df['heating_score'].astype('Int64')

# One-hot encode 'bygningstypekode' with the specified labels
bygningstypekode_labels = {
    111: 'Enebolig',
    112: 'Enebolig_m_sokkelleil',
    121: 'Tomannsbolig_vertikaldelt',
    122: 'Tomannsbolig_horisontaldelt',
    131: 'Rekkehus',
    136: 'Andre_smahus',
    141: 'Stort_frittliggende_2etg',
    142: 'Stort_frittliggende_3_4etg',
    143: 'Stort_frittliggende_5etg_eller_mer',
    146: 'Store_sammenb_5etg_eller_mer'
}

# Map 'bygningstypekode' to the corresponding labels
df['bygningstypekode_label'] = df['bygningstypekode'].map(bygningstypekode_labels)

# One-hot encode the labels
bygningstypekode_encoded = pd.get_dummies(df['bygningstypekode_label'], prefix='bygningstypekode', drop_first=True)

# Concatenate the one-hot encoded columns with the original DataFrame
df = pd.concat([df.drop(columns=['bygningstypekode', 'bygningstypekode_label']), bygningstypekode_encoded], axis=1)

# Map 'bydelsnr' to the corresponding labels
bydelsnr_labels = {
    0: 'Ukjent',
    1: 'Gamle_Oslo',
    2: 'Grünerløkka',
    3: 'Sagene',
    4: 'St_Hanshaugen',
    5: 'Frogner',
    6: 'Ullern',
    7: 'Vestre_Aker',
    8: 'Nordre_Aker',
    9: 'Bjerke',
    10: 'Grorud',
    11: 'Stovner',
    12: 'Alna',
    13: 'Østensjø',
    14: 'Nordstrand',
    15: 'Søndre_Nordstrand'
}

# Optionally remove rows with 'Ukjent' if they are not useful
#df = df[df['bydelsnr'] != 0]

# Map 'bydelsnr' to the corresponding labels
df['bydelsnr_label'] = df['bydelsnr'].map(bydelsnr_labels)

# One-hot encode the labels
bydelsnr_encoded = pd.get_dummies(df['bydelsnr_label'], prefix='bydelsnr', drop_first=True)

# Concatenate the one-hot encoded columns with the original DataFrame
df = pd.concat([df.drop(columns=['bydelsnr', 'bydelsnr_label']), bydelsnr_encoded], axis=1)


# Building date feature engineering
# Step 1: Convert 'etablertdato' to datetime format
df['etablertdato'] = pd.to_datetime(df['etablertdato'], errors='coerce')


# Drop rows where 'etablertdato' is missing
df = df.dropna(subset=['etablertdato'])

# Step 2: Extract only the year from the parsed date
df['build_year'] = df['etablertdato'].dt.year.astype('Int64')


def year_to_epoc(build_year):
    if build_year < 1930:
        return "1930_pre"
    elif build_year < 1950:
        return "1930_1950"
    elif build_year < 1970:
        return "1950_1970"
    elif build_year < 1990:
        return "1970_1990"
    elif build_year < 2010:
        return "1990_2010"
    else:
        return "post_2010"
    
df['epoch'] = df['build_year'].apply(year_to_epoc)


# One-hot encoding
epoch_encoded = pd.get_dummies(df['epoch'], drop_first=True)


df = pd.concat([df.drop(columns=['etablertdato','epoch']), epoch_encoded], axis=1)
print(df.columns)
print(df.sample(5))

# Check for NaN values in the dataset
if df.isnull().values.any():
    print("There are NaN values in the dataset.")
    print(df.isnull().sum())
else:
    print("No NaN values found in the dataset.")



# Check the shape of the DataFrame
print(f"Dataset shape: {df.shape}")

# Check for duplicate rows
duplicate_rows = df.duplicated().sum()
print(f"Number of duplicate rows: {duplicate_rows}")

# Check for NaN values in each column
nan_counts = df.isnull().sum()
print("NaN values per column:")
print(nan_counts[nan_counts > 0])

# Check the data types of each column
print("Data types of columns:")
print(df.dtypes)

# Get basic statistics for numerical columns
print("Basic statistics for numerical columns:")
print(df.describe())

# Get basic statistics for categorical columns
categorical_columns = df.select_dtypes(include=['object', 'category']).columns
print("Basic statistics for categorical columns:")
if not categorical_columns.empty:
    print(df[categorical_columns].describe())
else:
    print("No categorical columns found in the DataFrame.")

# Check for outliers in numerical columns using IQR
numerical_columns = df.select_dtypes(include=['number']).columns
for col in numerical_columns:
    Q1 = df[col].quantile(0.25)
    Q3 = df[col].quantile(0.75)
    IQR = Q3 - Q1
    outliers = ((df[col] < (Q1 - 1.5 * IQR)) | (df[col] > (Q3 + 1.5 * IQR))).sum()
    print(f"Number of outliers in {col}: {outliers}")

# Check the distribution of target variables (if applicable)
print("Distribution of target variables (if any):")
if 'target_column' in df.columns:  # Replace 'target_column' with the actual target column name
    print(df['target_column'].value_counts())
else:
    print("No target column specified.")


# Save the modified DataFrame back to a CSV file
df.to_csv(r'..\Processed data\Address_apartment_cleaned.csv', index=False)

