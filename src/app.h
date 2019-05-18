#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "util/param.h"
#include "util/util.h"
#include "graph/grid.h"
#include "graph/dao.h"
#include "graph/pd.h"

#include "agent/agent.h"
#include "task/task.h"
#include "problem/mapf.h"
#include "problem/mapd.h"
#include "problem/imapf.h"

#include "solver/pibt.h"
#include "solver/cbs.h"
#include "solver/ecbs.h"
#include "solver/whca.h"
#include "solver/hca.h"
#include "solver/tp.h"
#include "solver/pps.h"

static void setCurrentTime(std::string &str);  // for log filename


Problem* run(int argc, char *argv[])
{

  std::string configfile;

  /************************
   * set args
   ************************/
  int opt = 0;
  std::string arg;
  opterr = 0;  // ignore getopt error
  while ((opt = getopt(argc, argv, "p:")) != -1) {
    arg = std::string(optarg);
    switch (opt) {
    case 'p':  // config
      configfile = arg;
      break;
    default:
      break;
    }
  }

  /************************
   * default setting
   ************************/
  Param::EnvConfig* envConfig = new Param::EnvConfig
    {
     Param::P_MAPF,    // problem type
     Param::S_PIBT,    // solver type
     "./map/5x5.map",  // field type
     3,                // agent number
     10,               // task number
     0.1,              // task frequency
     0,                // seed
     false,            // save log
     true,             // print log
     false,            // print time
    };
  Param::SolverConfig* solverConfig = new Param::SolverConfig
    {
     false,  // WarshallFloyd
     true,   // CBS, ECBS, independet detection
     5,      // WHCA*, window size
     1.5     // ECBS, suboptimal param
    };
  Param::VisualConfig* visualConfig = new Param::VisualConfig
    {
     false,                  // showicon
     "./material/sheep.png"  // icon
    };

  /************************
   * load setting
   ************************/
  setParams(configfile, envConfig, solverConfig, visualConfig);

  /************************
   * set seed
   ************************/
  // seed for problem and graph
  std::mt19937* MT_PG = new std::mt19937(envConfig->seed);
  // seed for solver
  std::mt19937* MT_S = new std::mt19937(envConfig->seed);

  /************************
   * graph definition
   ************************/
  Graph* G = nullptr;
  if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_MAPF) {
    G = new DAO(envConfig->field, MT_PG);
  } else if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_MAPD) {
    G = new PD(envConfig->field, MT_PG);
  } else if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_IMAPF) {
    G = new DAO(envConfig->field, MT_PG);
  } else {
    std::cout << "error@run, problem is not defined" << "\n";
    std::exit(1);
  }

  /************************
   * agent definition
   ************************/
  std::vector<Agent*> A;
  auto points = G->getStartGoal(envConfig->agentnum);
  for (int i = 0; i < envConfig->agentnum; ++i) {
    Agent* a = new Agent(points[i][0]);
    A.push_back(a);
  }

  /************************
   * problem definition
   ************************/
  Problem* P = nullptr;
  if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_MAPF) {
    std::vector<Task*> T;
    for (int i = 0; i < envConfig->agentnum; ++i) {
      Task* tau = new Task(points[i][1]);
      T.push_back(tau);
    }
    P = new MAPF(G, A, T, MT_PG);
  } else if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_MAPD) {
    P = new MAPD(G, A, G->getPickup(), G->getDelivery(),
                 envConfig->tasknum, envConfig->taskfrequency, MT_PG);
  } else if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_IMAPF) {
    P = new IMAPF(G, A, envConfig->tasknum, MT_PG);
  } else {
    std::cout << "error@run, problem is not defined" << "\n";
    std::exit(1);
  }

  /************************
   * solver definition
   ************************/
  Solver* solver = nullptr;
  switch (envConfig->STYPE) {
  case Param::SOLVER_TYPE::S_CBS:
    if (envConfig->PTYPE != Param::PROBLEM_TYPE::P_MAPF) {
      std::cout << "error@run, CBS cannot solve except MAPF" << "\n";
      std::exit(1);
    }
    solver = new CBS(P, solverConfig->ID);
    break;
  case Param::SOLVER_TYPE::S_ECBS:
    if (envConfig->PTYPE != Param::PROBLEM_TYPE::P_MAPF) {
      std::cout << "error@run, ECBS cannot solve except MAPF" << "\n";
      std::exit(1);
    }
    solver = new ECBS(P, solverConfig->suboptimal, solverConfig->ID);
    break;
  case Param::SOLVER_TYPE::S_WHCA:
    if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_MAPD) {
      std::cout << "error@run, WHCA cannot solve MAPD" << "\n";
      std::exit(1);
    }
    solver = new WHCA(P, solverConfig->window);
    break;
  case Param::SOLVER_TYPE::S_HCA:
    if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_MAPD) {
      std::cout << "error@run, HCA cannot solve MAPD" << "\n";
      std::exit(1);
    }
    solver = new HCA(P);
    break;
  case Param::SOLVER_TYPE::S_PPS:
    if (envConfig->PTYPE == Param::PROBLEM_TYPE::P_MAPD) {
      std::cout << "error@run, PPS cannot solve MAPD" << "\n";
      std::exit(1);
    }
    solver = new PPS(P);
    break;
  case Param::SOLVER_TYPE::S_PIBT:
    solver = new PIBT(P, MT_S);
    break;
  case Param::SOLVER_TYPE::S_TP:
    if (envConfig->PTYPE != Param::PROBLEM_TYPE::P_MAPD) {
      std::cout << "error@run, TP cannot solve except MAPD" << "\n";
      std::exit(1);
    }
    solver = new TP(P, G->getAllSpecialPoints());
    break;
  default:
    std::cout << "error@run, solver is not defined" << "\n";
    std::exit(1);
    break;
  }

  // precomputing distances
  if (solverConfig->WarshallFloyd) {
    solver->WarshallFloyd();
  }

  /************************
   * start solving
   ************************/
  std::cout << "start solving" << "\n\n";
  solver->solve();

  /************************
   * summarize results
   ************************/
  std::string result = "[setting] seed:" + std::to_string(envConfig->seed) + "\n";
  result += solver->logStr();
  if (envConfig->log) {
    std::string outfile;
    std::ofstream log;
    setCurrentTime(outfile);
    outfile = "./log/" + outfile + ".txt";
    log.open(outfile, std::ios::out);
    if (!log) {
      std::cout << "error@run, cannot open log file "
                << outfile << "\n";
      std::exit(1);
    }
    log << result;
    log.close();
  }

  if (envConfig->printlog) {
    std::cout << result << "\n";
  } else if (envConfig->printtime) {
    std::cout << "[solver] elapsed:" << solver->getElapsed() << "\n\n";
  }

  // for visualization
  P->visual_showicon = visualConfig->showicon;
  P->visual_icon = visualConfig->icon;

  return P;
}

// for log file name
static void setCurrentTime(std::string &str) {
  char buff[27] = "";
  time_t now = time(NULL);
  timeval curTime;
  gettimeofday(&curTime, NULL);
  struct tm* pnow = localtime(&now);
  sprintf(buff, "%04d-%02d-%02d-%02d-%02d-%02d-%06d",
          pnow->tm_year + 1900, pnow->tm_mon + 1,
          pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec,
          curTime.tv_usec);
  str = std::string(buff);
}
