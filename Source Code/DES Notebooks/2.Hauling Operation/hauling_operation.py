import datetime as dt
import time
import simpy
import openclsim.core as core
import openclsim.model as model
import openclsim.plot as plot
import opentnsim

import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
import folium


def run(total_waste:int, file):
    '''
    def run(total_waste: int, file: str) -> dict:
    """
    Run a simulation based on input parameters and data from a CSV file.

    Parameters:
        total_waste (int): Total amount of waste to be processed.
        file (str): Path to the CSV file containing data for truck configuration.

    Returns:
        dict: A dictionary containing simulation results, including information about trucks,
              activities performed by trucks, and locations.

    This function reads truck configuration data from a CSV file, simulates truck movements,
    loading, unloading, and waste processing activities, and returns the results of the simulation.
    The simulation environment is set up using SimPy. Truck movements and activities are modeled
    based on the provided data, including locations, payload capacity, speed, and processing rates.

    Note:
        The CSV file must contain columns for 'ID', 'Type', 'Payload (Ton)', 'Minimum Speed (Mile/Hour)',
        'Maximum Speed (Mile/Hour)', 'Unloading Time (Minute)', 'Param 1', and 'Param 2'.
        The 'Param 1' and 'Param 2' columns specify parameters for the distribution of unloading times.
        The function dynamically creates truck and location objects based on the input data,
        simulates activities, and returns the simulation results.
    """


    
    
    '''
    # Read the CSV file
    file_path = file
    df = pd.read_csv(file_path)

    # Create an empty list to store information about each truck
    truck_list = []

    # Initialize a counter for overall truck numbering
    overall_truck_counter = 1

    # Iterate through rows and repeat based on the number of trucks
    for index, row in df.iterrows():
        # Convert 'Number of trucks' to integer
        num_trucks = int(row['Number of Trucks'])

        for truck_num in range(num_trucks):
            # Create a dictionary for each truck
            truck_info = {
                'ID': row['ID'],
                'Name of the trucks': f'truck{overall_truck_counter}',
                'Type': row['Type'],
                'Payload (Ton)': row['Payload (Ton)'],
                'Minimum Speed (Mile/Hour)': row['Minimum Speed (Mile/Hour)'],
                'Maximum Speed (Mile/Hour)': row['Maximum Speed (Mile/Hour)'],
                'Unloading Time (Minute)': row['Unloading Time (Minute)'],
                'Param 1': row['Param 1'],
                'Param 2': row['Param 2']
            }
            # Increment the overall truck counter
            overall_truck_counter += 1

            # Append the truck information to the list
            truck_list.append(truck_info)

    # Create a new data frame from the list
    df_result = pd.DataFrame(truck_list)

    # Calculate the total number of trucks as an integer
    total_trucks = int(df['Number of Trucks'].sum())

    # create a Site object based on desired mixin classes
    Site = type('Site',
                (core.Identifiable,
                 core.Log,
                 core.Locatable,
                 core.HasMultiContainer,
                 core.HasResource,), {})

    # create a TransportProcessingResource object based on desired mixin classes
    TransportProcessingResource = type(
        "TransportProcessingResource",
        (
            core.MultiContainerDependentMovable,
            core.HasResource,
            core.Processor,
            core.Identifiable,
            core.Log,
            opentnsim.core.ExtraMetadata
        ),
        {},
    )
    
    simulation_start = 0
    my_env = simpy.Environment(initial_time=simulation_start)
    registry = {}

    # prepare input data for source
    location_source01 = shapely.geometry.Point(4.062705498083785, 51.95909715255291)
    data_source01 = {"env": my_env,
                   "name": "source01",
                   "geometry": location_source01,
                   "initials": [
                       {"id": "waste", "level": total_waste, "capacity": total_waste},
                   ],
                   "nr_resources": 1
                   }
    source01 = Site(**data_source01)

    # prepare input data for dump
    location_dump01 = shapely.geometry.Point(4.1795229742823, 52.11590591837503)
    data_dump01 = {"env": my_env,
                 "name": "dump01",
                 "geometry": location_dump01,
                 "initials": [
                     {"id": "waste", "level": 0, "capacity": total_waste},
                 ],
                 }
    dump01 = Site(**data_dump01)

    # create truck objects
    trucks = {}

    for i in range(1, total_trucks + 1):
        # Extract information from the truck list
        truck_info = df_result[df_result['Name of the trucks'] == f'truck{i}'].iloc[0]

        # Extract unloading time distribution parameters
        mean_unloading_time = truck_info['Param 1']
        std_dev_unloading_time = truck_info['Param 2']
        # Generate random unloading time from the specified distribution
        unloading_rate = np.random.normal(loc=mean_unloading_time, scale=std_dev_unloading_time)

        # Create an instance of TransportProcessingResource
        trucks[f"truck{i}"] = TransportProcessingResource(
            env=my_env,
            name=f"truck{i}",
            geometry=location_dump01,
            initials=[
                {"id": "waste", "level": 0, "capacity": truck_info['Payload (Ton)']},
            ],
            loading_rate=1,
            unloading_rate=unloading_rate,
            compute_v=lambda x: 10 + 2 * x,
        )

   # Create activity objects
    activities_d_truck = {}

    for i in range(1, total_trucks + 1):
        requested_resources = {}

        d_subprocesses = [
            model.MoveActivity(
                env=my_env,
                name=f"driving_empty:" + trucks[f"truck{i}"].name,
                registry=registry,
                mover=trucks[f"truck{i}"],
                destination=source01,
                duration=20,
            ),
            model.ShiftAmountActivity(
                env=my_env,
                name=f"loading:" + trucks[f"truck{i}"].name,
                registry=registry,
                processor=trucks[f"truck{i}"],
                origin=source01,
                destination=trucks[f"truck{i}"],
                amount=20,
                duration=50,
                id_="waste",
                requested_resources=requested_resources,
            ),
            model.MoveActivity(
                env=my_env,
                name=f"driving_full:" + trucks[f"truck{i}"].name,
                registry=registry,
                mover=trucks[f"truck{i}"],
                destination=dump01,
                duration=15,
            ),
            model.ShiftAmountActivity(
                env=my_env,
                name=f"unloading:" + trucks[f"truck{i}"].name,
                registry=registry,
                processor=trucks[f"truck{i}"],
                origin=trucks[f"truck{i}"],
                destination=dump01,
                amount=20,
                duration=25,
                id_="waste",
                requested_resources=requested_resources,
            )
        ]
        activities_d_truck[f"activities_d_truck{i}"] = model.WhileActivity(
            env=my_env,
            name=f"while_sequential_activity_d_truck{i}",
            registry=registry,
            sub_processes=[model.SequentialActivity(
                env=my_env,
                name=f"sequential_activity_d_truck{i}",
                registry=registry,
                sub_processes=d_subprocesses
            )],
            condition_event=[{"type": "container",
                              "concept": dump01,
                              "state": "full",
                              "id_": "default_reservations"}
                             ]
        )

    model.register_processes([*activities_d_truck.values()])
    my_env.run()

    return {
        "trucks": trucks,
        "activities_d_truck": activities_d_truck,
        "source01": source01,
        "dump01": dump01,
    }
