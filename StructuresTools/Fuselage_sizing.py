import sys
sys.path.append("./SETools")

import numpy
import pandas
import os

from SE_data import SEData, data_path, backup_path  

data_path = os.path.join('SETools/data', 'configuration_data.csv')
backup_path = os.path.join('SETools/data', 'backup')

data = SEData(data_path, backup_path)
print(data.)