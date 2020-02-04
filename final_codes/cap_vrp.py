"""Capacited Vehicles Routing Problem (CVRP)."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model(bus_cap = 32):
    """Stores the data for the problem."""
    data = {}
    data['nodes'] = ['bosch bidadi, Banglore', 'Jayanagar, Banglore', 'Uttarahalli road Kengeri, Banglore', 'Mantri Apartment, Banglore', 'Kadarenahalli, Banglore', 'Devegowda Petrol Bunk, Banglore', 'Channasandra RNSIT, Banglore', 'Hoskeralli, Banglore', 'Bata Show Room, Banglore', 'Hosakarehalli, Banglore', 'Kodipalya (Uttarahalli main Road), Banglore', 'Kanthi Sweets RR Nagar, Banglore', 'Katherguppe Circle, Banglore', 'Chowdeshwari Talkies, Banglore', 'HosaKerehalli, Banglore', 'Kathreguppe, Banglore', 'Kathreguppe, Banglore', 'Rajarajeshwarinagar Double Road, Banglore', 'PESIT Collage, Banglore', 'Rajarajeshwari temple, Banglore', 'Kamakya Theatre, Banglore', 'Ittamadu, Banglore', 'Jantha Bazar, Banglore', 'Kathreguppe, Banglore']
    data['distance_matrix'] =[[0, 36401, 18438, 50945, 34845, 33266, 22478, 31947, 50466, 31947, 23184, 23862, 32669, 34789, 31947, 33265, 33265, 33089, 41453, 23045, 32968, 32659, 31856, 33265], [34770, 0, 14581, 8732, 4004, 3194, 9259, 6978, 13888, 6978, 8538, 9818, 4325, 3948, 6978, 3111, 3111, 5612, 14024, 11076, 3627, 5156, 6004, 3111], [17635, 14142, 0, 35069, 9832, 11007, 4041, 8612, 34590, 8612, 4746, 5424, 10411, 9974, 8612, 11007, 11007, 15357, 25577, 5313, 10709, 8385, 14124, 11007], [48625, 8638, 35111, 0, 10555, 11309, 16132, 15100, 6325, 15100, 15411, 16691, 12447, 10478, 15100, 11276, 11276, 9536, 9205, 18695, 12094, 13278, 12413, 11276], [33104, 3430, 9843, 10697, 0, 1521, 5802, 5313, 15853, 5313, 5081, 6361, 2660, 291, 5313, 3186, 3186, 8911, 15989, 8907, 2307, 3491, 9292, 3186], [31711, 3085, 10242, 11450, 1592, 0, 6201, 3920, 16606, 3920, 5480, 6760, 1267, 1536, 3920, 2025, 2025, 7898, 16742, 7515, 914, 2098, 8279, 2025], [21692, 9144, 4056, 16412, 5806, 5939, 0, 4586, 21568, 4586, 721, 1590, 6381, 5948, 4586, 6619, 6619, 13836, 27230, 6002, 6027, 4359, 13730, 6619], [30427, 6773, 8627, 15075, 5217, 3638, 4586, 0, 20231, 0, 3865, 4497, 3607, 5161, 0, 3700, 3700, 10880, 31413, 6231, 2928, 1440, 10167, 3700], [47916, 13263, 34402, 4803, 15180, 15933, 36206, 19725, 0, 19725, 20036, 37590, 17072, 15102, 19725, 15901, 15901, 13646, 8496, 23320, 16719, 17903, 16523, 15901], [30427, 6773, 8627, 15075, 5217, 3638, 4586, 0, 20231, 0, 3865, 4497, 3607, 5161, 0, 3700, 3700, 10880, 31413, 6231, 2928, 1440, 10167, 3700], [22398, 8423, 4762, 15691, 5085, 5218, 721, 3865, 20847, 3865, 0, 1848, 5660, 5227, 3865, 5898, 5898, 13116, 27196, 6708, 5306, 3638, 14435, 5898], [22606, 9645, 5562, 16913, 6307, 6439, 1714, 4288, 22069, 4288, 1973, 0, 5942, 6449, 4288, 6538, 6538, 13772, 28735, 4812, 6528, 4860, 12539, 6538], [31218, 5263, 9124, 13565, 2508, 2128, 5083, 2164, 18721, 2164, 4362, 5642, 0, 2719, 2164, 2295, 2295, 9383, 32203, 7021, 1329, 714, 9764, 2295], [32596, 3374, 9985, 10642, 291, 1013, 5944, 4805, 15798, 4805, 5223, 6503, 2152, 0, 4805, 2678, 2678, 8855, 15933, 8399, 1799, 2983, 9236, 2678], [30427, 6773, 8627, 15075, 5217, 3638, 4586, 0, 20231, 0, 3865, 4497, 3607, 5161, 0, 3700, 3700, 10880, 31413, 6231, 2928, 1440, 10167, 3700], [31686, 3117, 10994, 11375, 3227, 1648, 6644, 3894, 16531, 3894, 5923, 6438, 1241, 3171, 3894, 0, 0, 7515, 32671, 7489, 888, 2072, 7896, 0], [31686, 3117, 10994, 11375, 3227, 1648, 6644, 3894, 16531, 3894, 5923, 6438, 1241, 3171, 3894, 0, 0, 7515, 32671, 7489, 888, 2072, 7896, 0], [36296, 4268, 14847, 10168, 7972, 6751, 12334, 9216, 13861, 9216, 11613, 13128, 6931, 7916, 9216, 6208, 6208, 0, 16428, 11342, 6952, 7762, 4473, 6208], [40062, 18312, 26548, 14700, 19869, 34178, 28353, 32859, 11297, 32859, 28307, 29737, 33581, 19792, 32859, 34177, 34177, 19239, 0, 33828, 33880, 33571, 22116, 34177], [20467, 10670, 3505, 18972, 9114, 7535, 5067, 5600, 24128, 5600, 5772, 3734, 6939, 9058, 5600, 7535, 7535, 11885, 29742, 0, 7237, 6928, 10652, 7535], [30886, 3936, 10195, 12238, 2380, 801, 5789, 3095, 17394, 3095, 5069, 5638, 442, 2325, 3095, 968, 968, 8749, 31872, 6690, 0, 1273, 9130, 968], [30858, 5016, 8400, 13318, 3460, 1881, 4359, 1440, 18474, 1440, 3638, 4918, 1674, 3404, 1440, 1943, 1943, 8960, 31844, 6661, 1478, 0, 9341, 1943], [32483, 7335, 15521, 13323, 11082, 9861, 15134, 12485, 17016, 12485, 15840, 13802, 10041, 11026, 12485, 9318, 9318, 4496, 19583, 12016, 10062, 10872, 0, 9318], [31686, 3117, 10994, 11375, 3227, 1648, 6644, 3894, 16531, 3894, 5923, 6438, 1241, 3171, 3894, 0, 0, 7515, 32671, 7489, 888, 2072, 7896, 0]]
    data['demands'] = [0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8,4,6,9,1,3,4,5]
    data['num_vehicles'] = int(sum(data['demands'])/(0.85*bus_cap))+1
    data['vehicle_capacities'] = [bus_cap]*data['num_vehicles']
    data['depot'] = 0
    return data

def gen_map_link(link, place):
    x = place.replace(',',' ').split()
    c = x[0]
    for i in x[1:]:
        c = c + '+' + i
    # print(link)
    return link + c + '/'


def print_solution(data, manager, routing, assignment):
    """Prints assignment on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        link_to_maps = 'https://www.google.com/maps/dir/'
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0}: Load({1}) -> '.format(data['nodes'][node_index], route_load)
            link_to_maps = gen_map_link(link_to_maps, data['nodes'][node_index])
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0}: Load({1})\n'.format(data['nodes'][manager.IndexToNode(index)],
                                                 route_load)
        link_to_maps = gen_map_link(link_to_maps, data['nodes'][manager.IndexToNode(index)])
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print('link to maps: ', link_to_maps)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    mode = int(input("press 1 to run with default values, 2 to run with custom values \n enter your choice: "))
    if(mode == 1):
        bus_cap = 32
        max_dist_veh = 100
        print("Running with default values.\nbus capacity = 32, max vehicle distance = 100km")
    if(mode == 2):
        bus_cap = int(input("Enter the capacity of each bus: "))
        max_dist_veh = int(input("enter the max vehicle distance a vehicle can travel: "))
    else:
        bus_cap = 32
        max_dist_veh = 100
        print("incorrect option choosed, running with default values.\nbus capacity = 32, max vehicle distance = 100km")
    data = create_data_model(bus_cap)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    #Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_dist_veh * 1000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(0)
        

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 10
    search_parameters.log_search = True

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    # assignment = routing2.SolveFromAssignmentWithParameters(assignment, search_parameters)

    # Print solution on console.
    if assignment:
        print_solution(data, manager, routing, assignment)
    else:
        print('NO SOLUTION')

if __name__ == '__main__':
    main()