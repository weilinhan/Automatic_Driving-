import pandas as pd
import plotly.express as px

# read
df = pd.read_csv('results/speedCapturing.csv')
# axis
fig = px.line(df,
              x='time in ms',
              y=['measured speed in cm/s',
                 'requested speed in cm/s',
                 'requested PWM in 0.1%'],
              title='speed measurement')
# plot
fig.show()
# safe
fig.write_html('results/speed_measurement.html')
