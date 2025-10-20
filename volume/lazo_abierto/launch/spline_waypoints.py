import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from scipy.interpolate import CubicSpline

######## Funciones auxiliares ########
def create_data_model(x, y):
    """Stores the data for the problem."""
    data = {}
    data["locations"] = list(zip(x, y))
    data["distance_matrix"] = np.zeros((len(data["locations"]), len(data["locations"])))
    for i in range(len(data["locations"])):
        for j in range(len(data["locations"])):
            data["distance_matrix"][i][j] = np.linalg.norm(np.array(data["locations"][i]) - np.array(data["locations"][j]))
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data

def distance_callback(from_index, to_index, manager, data):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return int(data["distance_matrix"][from_node][to_node])

def get_routes(solution, routing, manager, data, alpha_time):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    time = [0]
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        location_index = manager.IndexToNode(index)
        route = [data["locations"][location_index]]
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            location_index = manager.IndexToNode(index)
            route.append(data["locations"][location_index])
            time.append(time[-1] + alpha_time * data['distance_matrix'][previous_index][location_index])
        routes.append(route)
    return np.array(routes[0]), np.array(time)

def generate_random_waypoints(range, samples, alpha_time, seed=28):
    """Genera waypoints aleatorios y calcula los spline waypoints que los recorre en menor tiempo.
    
    Parameters
    ----------
    range : float
        Rango en el que se generan los waypoints aleatorios.
    samples : int
        Numero de waypoints aleatorios a generar.
    alpha_time : float
        Constante que multiplica a la distancia para calcular el tiempo.
    
    Returns
    -------
    list
        Lista de waypoints en formato [t, x, y, theta] para ser usados en el launch.
    """
    np.random.seed(seed)
    xs = np.concatenate([[1], np.random.uniform(-range, range, samples)])
    ys = np.concatenate([[0], np.random.uniform(-range, range, samples)])
    
    data = create_data_model(xs, ys)
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Funcion objetivo
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromMilliseconds(1000)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    data['locations_sorted'], data['time_sorted'] = get_routes(solution, routing, manager, data, alpha_time)

    # Create separate splines for x and y coordinates
    cs_x = CubicSpline(data['time_sorted'], data['locations_sorted'][:, 0], bc_type='natural') # f(x(t))
    cs_y = CubicSpline(data['time_sorted'], data['locations_sorted'][:, 1], bc_type='natural') # f(y(t))
    dxdt = cs_x.derivative()(data['time_sorted'])
    dydt = cs_y.derivative()(data['time_sorted'])
    angles = np.arctan2(dydt, dxdt)

    data['spline_waypoints'] = np.concatenate((data['time_sorted'][:, None] + alpha_time, data['locations_sorted'], angles[:, None]), axis=1)
    data['spline_waypoints'] = np.concatenate((np.array([[0., 0., 0., 0.]]), data['spline_waypoints']))
    data['spline_waypoints'] = data['spline_waypoints'].reshape(-1).round(8).tolist()

    return data['spline_waypoints']

if __name__ == "__main__":
    spline_waypoints = generate_random_waypoints(range=5, samples=6, alpha_time=1)
    print(np.array(spline_waypoints).reshape(-1, 4))