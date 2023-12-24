import pandas as pd

# Function to read CSV data
def csv_to_datadict(file_path: str, fps, time_key='time [s]'):
    df = pd.read_csv(file_path)
    datadict = {}
    for key in df:
        datadict[key] = df[key].to_numpy()

    # Subsample data
    dt = datadict[time_key][1] - datadict[time_key][0]
    samples = int((1 / fps) / dt)
    for key in datadict:
        datadict[key] = datadict[key][::samples]
    return datadict