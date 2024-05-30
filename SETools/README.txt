
To import
from SETools import SE_Data

# Create an instance of the SEData class
se_data = SEData(data_path="/path/to/design_configuration.txt", backup_path="/path/to/backup_folder.txt")

# Retrieve the most recent design configuration
design_config = se_data.get_all_data()

# Use the retrieved design configuration in your subsystem sizing calculations
subsystem_size = calculate_subsystem_size(design_config) #example

# Store properties in the configuration file
se_data.add_subsystem_property("EPS","Power", 1200)

#An entire subsystem can be added at once if a dictionary with all properties and values exists
EPS_dict = {'Mass' : 15, 'Power': 1000, 'efficiency': 0.6}
se_data.add_subsystem("EPS", EPS_dict)

#Individual values can be updated within the subsystem
se_data.update_subsystem_property(self, "EPS", "Power", 1000)

# It is possible to create a backup:
se_data.create_backup()

# It is possible to empty the configuration file completely or for an individual subsystem. A backup will be automatically created in that case

se_data.clear_values() #clears all property entries for all subsystems, subsystem and property header are conserved

design_config.clear_values("EPS") # clears all property entries for the selected subsystem

##NOTE: All functions are case insensitive. Everything is automatically converted to uppercase.

    