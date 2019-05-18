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

PD::PD(std::string _filename, std::mt19937* _MT)
  : Grid(_MT), filename(_filename)
{
  init();
}

PD::PD(std::string _filename) : filename(_filename) {
  init();
}

void PD::init() {
  std::ifstream file(filename);
  if (!file) {
    std::cout << "error@PD::init, file "
              << filename
              << " does not exist" << "\n";
    std::exit(1);
  }

  std::string line;
  std::smatch results;
  int w  = 0;
  int h = 0;
  bool mapStart = false;

  std::regex r_height = std::regex(R"(height\s(\d+))");
  std::regex r_width = std::regex(R"(width\s(\d+))");
  std::regex r_map = std::regex(R"(map)");

  int j = 0;
  int index = 0;
  nodes.clear();
  objs.clear();

  // check
  bool validP = false;
  bool validD = false;

  while (getline(file, line)) {
    if (mapStart) {
      for (int i = 0; i < w; ++i) {
        index = j * w + i;
        if (line[i] == 'T') {  // objs
          objs.push_back(index);
        } else {
          Node* v = new Node(index);
          v->setPos(i, j);
          nodes.push_back(v);

          if (line[i] == 'P') {  // pickup loc.
            pickup.push_back(v);
            validP = true;
          } else if (line[i] == 'D') {  // delivery loc.
            delivery.push_back(v);
            validD = true;
          } else if (line[i] == 'E') {  // endpoint loc.
            endpoints.push_back(v);
          } else if (line[i] == 'S') {  // pickup and delivery loc.
            pickup.push_back(v);
            delivery.push_back(v);
            validP = true;
            validD = true;
          } else if (line[i] == 'A') {  // pickup and delivery and endpoint loc.
            endpoints.push_back(v);
            pickup.push_back(v);
            delivery.push_back(v);
            validP = true;
            validD = true;
          }
        }
      }
      ++j;
    }

    if (std::regex_match(line, results, r_height)) {
      h = std::stoi(results[1].str());
    }

    if (std::regex_match(line, results, r_width)) {
      w = std::stoi(results[1].str());
    }

    if (std::regex_match(line, results, r_map)) {
      mapStart = true;
      setSize(w, h);
    }
  }
  file.close();

  if (!validD || !validP) {
    std::cout << "error@TP::init, file "
              << filename
              << " is not for MAPD" << "\n";
    std::exit(1);
  }

  initNodes();
}

std::vector<std::vector<Node*>> PD::getStartGoal(int num) {
  if (num > nodes.size()) {
    std::cout << "error@PD::getStartGoal,over node size, "
              << num << "\n";
    std::exit(1);
  }

  std::vector<std::vector<Node*>> points;
  std::vector<Node*> ends(endpoints.size());
  std::copy(endpoints.begin(), endpoints.end(), ends.begin());
  std::shuffle(ends.begin(), ends.end(), *MT);
  for (int i = 0; i < num; ++i) points.push_back({ ends[i] });

  return points;
}

std::vector<Node*> PD::getAllSpecialPoints() {
  std::vector<Node*> points;
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

PD::~PD() {}

std::string PD::logStr() {
  std::string str;
  str += "[graph] file:" + filename + "\n";
  str += Graph::logStr();
  str += "\n";
  str += "[graph] pickup:";
  for (auto v : pickup) str += std::to_string(v->getId()) + ",";
  str += "\n[graph] delivery:";
  for (auto v : delivery) str += std::to_string(v->getId()) + ",";
  str += "\n[graph] endpoints:";
  for (auto v : endpoints) str += std::to_string(v->getId()) + ",";
  return str;
}
