#include "AStarPlanner.h"
#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#define COST_BETWEEN_CELLS 1

robmovil_planning::AStarPlanner::AStarPlanner()
: GridPlanner()
{ }

std::vector<robmovil_planning::AStarPlanner::Cell> robmovil_planning::AStarPlanner::neighbors(const Cell& c)
{
  /* COMPLETAR: Calcular un vector de vecinos (distintos de c).
   * IMPORTANTE: Tener en cuenta los limites de la grilla (utilizar grid_->info.width y grid_->info.heigh)
   *             y aquellas celdas ocupadas */
   
  uint i = c.i;
  uint j = c.j;
  std::vector<std::pair<int, int>> posiblesVecinos = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1},   // cardinales (N, S, E, O)
      {-1, -1}, {-1, 1}, {1, -1}, {1, 1}  //diagonales (NE, SE, SW, NW)
    };
  std::vector<Cell> neighbors;
  for (const auto& posiblesVecino : posiblesVecinos)
  {
    int ni = i+posiblesVecino.first;
    int nj = j+posiblesVecino.second;
    double x;
    double y;

    if(!getCenterOfCell (ni , nj , x , y)){ //devuelve !false si esta fuera de rango 
      continue;
    }

    if(isANeighborCellOccupy(ni , nj)){ //devuelve true si esta ocupada 
      continue;
    }
    Cell neighbor;
    neighbor.i = ni;
    neighbor.j = nj;
    neighbors.push_back(neighbor);
  }

  return neighbors;
}

double robmovil_planning::AStarPlanner::heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
{  
  /* COMPLETAR: Funcion de heuristica de costo */

  double dx = std::abs(static_cast<double>(current.i) - static_cast<double>(goal.i));
  double dy = std::abs(static_cast<double>(current.j) - static_cast<double>(goal.j));
  
  double heuristic = (dx + dy) * COST_BETWEEN_CELLS;

  return heuristic;
}

bool robmovil_planning::AStarPlanner::do_planning(robmovil_msgs::msg::Trajectory& result_trajectory)
{
  uint start_i, start_j;
  uint goal_i, goal_j;
  
  getCellOfPosition(starting_pose_.pose.position.x, starting_pose_.pose.position.y, start_i, start_j);
  getCellOfPosition(goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_i, goal_j);
  
  /* Celdas de inicio y destino */
  Cell start = Cell(start_i, start_j);
  Cell goal = Cell(goal_i, goal_j);
  
  /* Contenedores auxiliares recomendados para completar el algoritmo */
  std::priority_queue<CellWithPriority, std::vector<CellWithPriority>, PriorityCompare> frontier;
  std::map<Cell, Cell> came_from;
  std::map<Cell, double> cost_so_far;
  std::map<Cell, bool> closed_set;
  
  bool path_found = false;
  
  /* Inicializacion de los contenedores (start comienza con costo 0) */
  frontier.push(CellWithPriority(start, 0));
  cost_so_far[start] = 0;
  
  /* COMPLETAR: Utilizar los contenedores auxiliares para la implementacion del algoritmo A*
   * NOTA: Pueden utilizar las funciones neighbors(const Cell& c) y heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
   *       para la resolucion */
  
  while (!frontier.empty()) {

    Cell current = frontier.top(); // current = vertex in OPEN with min f[]
    frontier.pop(); // remove current from OPEN

    // Line 12: if current == goal then return
    if (current == goal) {
      path_found = true;
      break;
    }

    // Line 16: agregar current a CLOSED
    closed_set[current] = true;
    
    // Line 17: recorrer los vecinos de current
    for (const Cell& neighbor : neighbors(current)) {
      
      // Line 18: if neighbor in CLOSED then skip iteration
      if (closed_set[neighbor]) {
        continue;
      }

      // Line 21: cost = g[current] + movement_cost(current, neighbor)
      double cost = cost_so_far[current] + COST_BETWEEN_CELLS;

      // Line 22: neighbor not OPEN
      if (cost_so_far.find(neighbor) == cost_so_far.end()) {
        // add neighbor to OPEN
        frontier.push(CellWithPriority(neighbor, cost + heuristic_cost(start, goal, neighbor)));
      } else if (cost >= cost_so_far[neighbor]) {
        continue; // Este no es un mejor camino
      }
        
        // Line 27: prev[neighbor] = current
        came_from[neighbor] = current;
        
        // Line 28: g[neighbor] = cost
        cost_so_far[neighbor] = cost;
      }
    }

  if(not path_found)
    return false;
  
  /* Construccion y notificacion de la trajectoria.
   * NOTA: Se espera que came_from sea un diccionario representando un grafo de forma que:
   *       goal -> intermedio2 -> intermedio1 -> start */
  notifyTrajectory(result_trajectory, start, goal, came_from);

  return true;
}

void robmovil_planning::AStarPlanner::notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory, const Cell& start, const Cell& goal, 
                                                       std::map<Cell, Cell>& came_from)
{
  std::vector<Cell> path;
  Cell current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    //ROS_INFO_STREAM("Path " << it->i << ", " << it->j);
    RCLCPP_INFO(this->get_logger(), "Path: %u, %u", it->i, it->j);
    
    double cell_x, cell_y;
    
    getCenterOfCell(it->i, it->j, cell_x, cell_y);
    
    // Se crean los waypoints de la trajectoria
    robmovil_msgs::msg::TrajectoryPoint point_msg;

    point_msg.transform.translation.x = cell_x;
    point_msg.transform.translation.y = cell_y;
    point_msg.transform.translation.z = 0;
    
    if(it != path.rend()-1){
      double delta_x, delta_y;
      getCenterOfCell((it+1)->i, (it+1)->j, delta_x, delta_y);
      delta_x = delta_x - cell_x;
      delta_y = delta_y - cell_y;
      tf2::Quaternion delta_q;
      delta_q.setRPY(0, 0, angles::normalize_angle(atan2(delta_y, delta_x)));
      point_msg.transform.rotation = tf2::toMsg(delta_q);
      //point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(delta_y, delta_x)) );
    } else {
      tf2::Quaternion delta_q;
      delta_q.setRPY(0, 0, angles::normalize_angle(atan2(cell_y, cell_x)));
      point_msg.transform.rotation = tf2::toMsg(delta_q);
      //point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(cell_y, cell_x)) );
    }
    
    result_trajectory.points.push_back( point_msg );
  }
}
