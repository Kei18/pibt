/*
 * station.cpp
 *
 * Purpose: Station
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include <fstream>
#include <regex>
#include "../util/util.h"
#include "station.h"

Station::Station(std::string _filename)
  : SimpleGrid(_filename)
{
  init();
}

Station::Station(std::string _filename, std::mt19937* _MT)
  : SimpleGrid(_filename, _MT)
{
  init();
}

Station::~Station() {}


void Station::init() {
  // initialize
  for (int i = 0; i < 10; ++i) stations.push_back({});

  std::string file_st = filename + ".st";  // station
  std::ifstream file(file_st);
  if (!file) {
    std::cout << "error@Station::init, file "
              << file_st
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

  std::regex r_obj = std::regex(R"([@T])");
  std::regex r_station = std::regex(R"([0-9])");

  while (getline(file, line)) {
    // error check, width
    if (line.size() != w) {
      std::cout << "error@Station::init, "
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

      if (std::regex_match(s, results, r_station)) {
        stations[line[i] - '0'].push_back(v);
      }
    }
    ++j;
  }
  file.close();

  // height check
  if (j != h) {
    std::cout << "error@Station::init, "
              << "height is invalid, shoudl be " << h <<  "\n";
    std::exit(1);
  }

  // formatting
  auto itr = stations.begin();
  while (itr != stations.end()) {
    if (itr->empty()) {
      stations.erase(itr);
      itr = stations.begin();
    } else {
      ++itr;
    }
  }

  setStartGoal();
  startSt = 0;
}

void Station::setStartGoal() {
  // initialization
  starts.clear();
  goals.clear();

  // update starts
  for (auto st : stations) {
    for (auto v : st) {
      starts.push_back(v);
      goals.push_back(v);
    }
  }
}

Paths Station::getRandomStartGoal(int num) {
  int stNum = stations.size();
  if (std::any_of(stations.begin(), stations.end(),
                  [num, stNum] (Nodes st)
                  { return st.size() * stNum < num; })) {
    std::cout << "error@Station::getStartGoal, over node size" << "\n";
    std::exit(1);
  }

  // preparing shuffled one
  Paths tmpS, tmpG;
  for (auto st : stations) {
    Nodes areaS(st.size()), areaG(st.size());
    std::copy(st.begin(), st.end(), areaS.begin());
    std::copy(st.begin(), st.end(), areaG.begin());
    std::shuffle(areaS.begin(), areaS.end(), *MT);
    std::shuffle(areaG.begin(), areaG.end(), *MT);
    tmpS.push_back(areaS);
    tmpG.push_back(areaG);
  }

  Paths points;
  std::vector<std::vector<int>> indexes;
  std::vector<int> cands;  // choose goal station
  for (int i = 0; i < stNum; ++i) {
    cands.push_back(i);
    indexes.push_back({ 0, 0 });
  }
  int goalSt;

  for (int i = 0; i < num; ++i) {
    // choose start st
    do {
      ++startSt;
      startSt %= stNum;
    } while (indexes[startSt][0] >= stations[startSt].size());

    // choose goal st
    do {
      goalSt = randomChoose(cands, MT);
    } while (goalSt == startSt || indexes[goalSt][1] >= stations[goalSt].size());

    points.push_back({ tmpS[startSt][indexes[startSt][0]],
                       tmpG[goalSt][indexes[goalSt][1]]});
    indexes[startSt][0] += 1;
    indexes[goalSt][1]  += 1;
  }

  return points;
}

Node* Station::getNewGoal(Node* v) {
  int vIn = -1;
  std::vector<int> cands;
  for (int i = 0; i < stations.size(); ++i) {
    cands.push_back(i);
    if (inArray(v, stations[i])) vIn = i;
  }

  int newSt;
  do {
    newSt = randomChoose(cands, MT);
  } while (newSt == vIn);

  return randomChoose(stations[newSt], MT);
}

std::string Station::logStr() {
  std::string str = SimpleGrid::logStr();
  for (int i = 0; i < stations.size(); ++i) {
    str += "[graph] St." + std::to_string(i) + ":";
    for (auto v : stations[i]) str += std::to_string(v->getId()) + ",";
    str += "\n";
  }
  return str;
}
