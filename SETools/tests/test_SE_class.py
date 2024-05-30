import sys
sys.path.append("./SETools")

import pytest
import os
import pandas as pd
import numpy as np
from datetime import datetime
from SE_data import SEData, test_data_path, test_backup_path

# Helper function to clean up created files during tests
def cleanup_files():
    if os.path.exists(test_data_path):
        os.remove(test_data_path)
    if os.path.exists(test_backup_path):
        for file in os.listdir(test_backup_path):
            os.remove(os.path.join(test_backup_path, file))

@pytest.fixture(scope="function")
def setup_teardown():
    # Setup: clean up before each test
    cleanup_files()
    yield
    # Teardown: clean up after each test
    cleanup_files()

def test_add_subsystem(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100, "power": 200})
    result = data.get_subsystem("ALT")
    assert result["Property"].tolist() == ["MASS", "POWER"]
    assert result["ALT"].tolist() == [100, 200]

def test_add_subsystem_property(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100})
    data.add_subsystem_property("ALT", "power", 200)
    result = data.get_subsystem("ALT")
    assert result["Property"].tolist() == ["MASS", "POWER"]
    assert result["ALT"].tolist() == [100, 200]

def test_clear_values_single_subsystem(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100, "power": 200})
    data.clear_values("ALT")
    result = data.get_subsystem("ALT")
    assert result.empty

def test_clear_values_all_subsystems(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100, "power": 200})
    data.add_subsystem("EPS", {"mass": 50, "power": 150})
    data.clear_values()
    result_alt = data.get_subsystem("ALT")
    result_eps = data.get_subsystem("EPS")
    assert result_alt.empty
    assert result_eps.empty

def test_update_subsystem_property(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100})
    data.update_subsystem_property("ALT", "mass", 150)
    result = data.get_subsystem("ALT")
    assert result["Property"].tolist() == ["MASS"]
    assert result["ALT"].tolist() == [150]

def test_create_backup(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100, "power": 200})
    data.create_backup()
    backup_files = os.listdir(test_backup_path)
    assert len(backup_files) == 1
    assert backup_files[0].startswith("configuration_data_")

def test_remove_subsystem(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100, "power": 200})
    data.remove_subsystem("ALT")
    with pytest.raises(ValueError):
        data.get_subsystem("ALT")

def test_remove_subsystem_property(setup_teardown):
    data = SEData(test_data_path, test_backup_path)
    data.add_subsystem("ALT", {"mass": 100, "power": 200})
    data.remove_subsystem_property("ALT", "power")
    result = data.get_subsystem("ALT")
    assert result["Property"].tolist() == ["MASS"]
    assert result["ALT"].tolist() == [100]

