/*
 * station.h
 *
 * Purpose: Station
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#pragma once
#include "simplegrid.h"


class Station : public SimpleGrid {
private:
  int startSt;

  Paths stations;
  void init();
  void setStartGoal();

public:
  Station(std::string filename);
  Station(std::string filename, std::mt19937* _MT);
  ~Station();

  Paths getRandomStartGoal(int num);
  Node* getNewGoal(Node* v);
  std::string logStr();
};
