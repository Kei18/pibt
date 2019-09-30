#pragma once
#include "simplegrid.h"

/*
 * graph representation
 *
 * p : pickup loc.
 * d : devlivery loc.
 * e : end point
 * s : pickup and delivery loc.
 * a : all
 * @ : object
 * T : object
 */

class PD : public SimpleGrid {
private:
  void init();

protected:
  Nodes pickup;
  Nodes delivery;
  Nodes endpoints;

  void setStartGoal();

public:
  PD(std::string filename);
  PD(std::string filename, std::mt19937* _MT);
  ~PD();

  Nodes getPickup() { return pickup; }
  Nodes getDelivery() { return delivery; }
  Nodes getEndpoints() { return endpoints; }
  Nodes getAllSpecialPoints();
  Paths getRandomStartGoal(int num);

  std::string logStr();
};
