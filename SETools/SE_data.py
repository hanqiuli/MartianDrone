import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

data_path = os.path.join('SETools/data', 'configurations_data.csv')


class SE_data:
    '''Class to store  and retrieve the design data for all the subsystems'''
    def __init__(self, data_path:str = data_path):
        self.data_path = data_path
        self.data = pd.read_csv(data_path)
        self.data = self.data.set_index('Subsystem')
        self.data = self.data.drop(columns=['Unnamed: 0'])
        