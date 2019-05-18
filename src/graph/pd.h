#pragma once
#include <vector>
#include "grid.h"


// map for Pickup and Delivery
class PD : public Grid {
private:
  void init();

protected:
  std::string filename;
  std::vector<Node*> pickup;     // pickup locatoins
  std::vector<Node*> delivery;   // delivery locations
  std::vector<Node*> endpoints;  // endpoints (for TP algorithm)

public:
  PD(std::string filename);
  PD(std::string filename, std::mt19937* _MT);
  ~PD();

  std::vector<Node*> getPickup() { return pickup; }
  std::vector<Node*> getDelivery() { return delivery; }
  std::vector<Node*> getEndpoints() { return endpoints; }
  std::vector<std::vector<Node*>> getStartGoal(int num);
  std::vector<Node*> getAllSpecialPoints();

  std::string logStr();
};
