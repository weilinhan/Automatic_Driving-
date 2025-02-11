import pandas as pd
# to see all rows
pd.set_option('display.max_rows', None)
# read
df = pd.read_csv('results/speedCapturing.csv')
# show
print(df)
