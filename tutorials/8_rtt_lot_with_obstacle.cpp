//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include "fields2cover.h"
#include <iostream>

int main() {
 
  //Field Definition
  F2CLinearRing outter_ring{
    F2CPoint(0, 0), F2CPoint(40, 0),F2CPoint(40,24), F2CPoint(0, 24), 
    F2CPoint(0, 0)};
  F2CLinearRing obstacle_1{
    F2CPoint(25,15), F2CPoint(30,15), F2CPoint(30,17), F2CPoint(25,17), 
    F2CPoint(25,15)};
  F2CLinearRing obstacle_2{
    F2CPoint(10,7), F2CPoint(17,9), F2CPoint(11,14), F2CPoint(7,12), 
    F2CPoint(10,7)};
  F2CCell cell;
  cell.addRing(outter_ring);
  cell.addRing(obstacle_1);
  cell.addRing(obstacle_2);
  F2CCells cells;
  cells.addGeometry(cell);

  //Robot Definition   
  F2CRobot robot (0.8, 2.5); //(robot width, operating width)
  robot.setMinRadius(0.3);  // m
  robot.linear_curv_change = 5;  // 1/m^2

  //Headland Generator
  f2c::hg::ConstHL const_hl; //constant headlands
  F2CCells mid_hl = const_hl.generateHeadlands(cells, 1.5 * robot.robot_width); //1/2 headlands
  F2CCells no_hl = const_hl.generateHeadlands(cells, 2 * robot.robot_width);//headlands shape

  //Swath Generator
  f2c::obj::SwathLength obj;//bf target criteria
  f2c::sg::BruteForce swath_gen;
  F2CSwaths swaths = swath_gen.generateSwaths(M_PI_4, robot.op_width, no_hl.getGeometry(0));
  
  //Route Planner
  f2c::rp::SnakeOrder route;
  swaths = route.genSortedSwaths(swaths);

  //Path Planner
  f2c::pp::PathPlanning path_planner;
  f2c::pp::DubinsCurves path;
  F2CPath path_dubins = path_planner.searchBestPath(robot, swaths, path);

  //Outputs
  std::cout << "####### Tutorial 8 rtt lot ######" << std::endl;
  f2c::Visualizer::figure(81);
  f2c::Visualizer::plot(cells);
  f2c::Visualizer::plot(no_hl);
  f2c::Visualizer::plot(path_dubins);
  f2c::Visualizer::plot(swaths);
  f2c::Visualizer::save("wide_dubins_snake");

  return 0;
}




