/*
 * pd.cpp
 *
 * Purpose: Field of Pick & Delivery
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include <fstream>
#include <regex>
#include "pd.h"
#include "../util/util.h"

PD::PD(std::string _filename) : SimpleGrid(_filename) {
  init();
}

PD::PD(std::string _filename, std::mt19937* _MT) : SimpleGrid(_filename, _MT) {
  init();
}

PD::~PD() {}

void PD::init() {
  std::string file_pd = filename + ".pd";  // pickup and delivery
  std::ifstream file(file_pd);
  if (!file) {
    std::cout << "error@PD::init, file "
              << file_pd
              << " does not exist" << "\n";
    std::exit(1);
  }

  std::string line;
  std::smatch results;
  int w = getW();
  int h = getH();
  int j = 0;  // height
  int id = 0;
  std::string s;
  Node* v;

  std::regex r_obj   = std::regex(R"([@T])");
  std::regex r_pickup = std::regex(R"([psa])");  // pickup loc.
  std::regex r_deliv  = std::regex(R"([dsa])");  // delivery loc.
  std::regex r_end    = std::regex(R"([ea])");  // end loc.

  while (getline(file, line)) {
    // error check, width
    if (line.size() != w) {
      std::cout << "error@PD::init, "
                << "width is invalid, should be " << w <<  "\n";
      std::exit(1);
    }

    for (int i = 0; i < w; ++i) {
      s = line[i];
      id = j * w + i;
      if (std::regex_match(s, results, r_obj)) continue;

      if (!existNode(id)) {
        std::cout << "error@PD::init, "
                  << "corresponding node does not exist, " << id << "\n";
        std::exit(1);
      }
      v = getNode(id);  // target node

      if (std::regex_match(s, results, r_pickup)) {
        pickup.push_back(v);
      }
      if (std::regex_match(s, results, r_pickup)) {
        delivery.push_back(v);
      }
      if (std::regex_match(s, results, r_end)) {
        endpoints.push_back(v);
      }
    }
    ++j;
  }
  file.close();

  // height check
  if (j != h) {
    std::cout << "error@PD::init, "
              << "height is invalid, shoudl be " << h <<  "\n";
    std::exit(1);
  }

  if (pickup.empty() || delivery.empty() || endpoints.empty()) {
    std::cout << "error@TP::init, file "
              << file_pd << ".pd is not for MAPD, "
              << "there is no pickup/delivery/end location" << "\n";
    std::exit(1);
  }

  setStartGoal();
}

void PD::setStartGoal() {
  // initialization
  starts.clear();
  goals.clear();

  // update starts
  starts = endpoints;
}

Paths PD::getRandomStartGoal(int num) {
  if (num > nodes.size()) {
    std::cout << "error@PD::getStartGoal, over node size, "
              << num << "\n";
    std::exit(1);
  }

  Paths points;
  Nodes ends(starts.size());
  std::copy(starts.begin(), starts.end(), ends.begin());
  std::shuffle(ends.begin(), ends.end(), *MT);
  for (int i = 0; i < num; ++i) points.push_back({ ends[i] });

  return points;
}

Nodes PD::getAllSpecialPoints() {
  Nodes points;
  for (auto v : pickup) {
    auto itr = std::find_if(points.begin(), points.end(),
                            [v](Node* u) { return v->getId() == u->getId(); });
    if (itr == points.end()) points.push_back(v);
  }
  for (auto v : delivery) {
    auto itr = std::find_if(points.begin(), points.end(),
                            [v](Node* u) { return v->getId() == u->getId(); });
    if (itr == points.end()) points.push_back(v);
  }
  for (auto v : endpoints) {
    auto itr = std::find_if(points.begin(), points.end(),
                            [v](Node* u) { return v->getId() == u->getId(); });
    if (itr == points.end()) points.push_back(v);
  }
  return points;
}

std::string PD::logStr() {
  std::string str = SimpleGrid::logStr();
  str += "[graph] pickup:";
  for (auto v : pickup) str += std::to_string(v->getId()) + ",";
  str += "\n[graph] delivery:";
  for (auto v : delivery) str += std::to_string(v->getId()) + ",";
  str += "\n[graph] endpoints:";
  for (auto v : endpoints) str += std::to_string(v->getId()) + ",";
  str += "\n";
  return str;
}
