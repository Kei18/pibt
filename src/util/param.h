#pragma once
#include <iostream>
#include <regex>


namespace Param {
  // def. of problem types
  enum PROBLEM_TYPE { P_MAPF,
                      P_MAPD,
                      P_IMAPF,
                      P_MAPF_STATION,
                      P_IMAPF_STATION };

  // def. of solver types
  enum SOLVER_TYPE { S_CBS,
                     S_ECBS,
                     S_iECBS,
                     S_WHCA,
                     S_HCA,
                     S_PPS,
                     S_TP,
                     S_PIBT,
                     S_winPIBT };

  // params of problem setting
  struct EnvConfig {
    PROBLEM_TYPE PTYPE;
    SOLVER_TYPE  STYPE;
    std::string field;   // file name
    int agentnum;
    int timesteplimit;
    int tasknum;
    float taskfrequency;
    int seed;  // seed
    bool log;
    bool printlog;
    bool printtime;
  };

  // params of solver
  struct SolverConfig {
    // for all
    bool WarshallFloyd;

    // for CBS, ECBS, iECBS, independet operation
    bool ID;

    // for whca*, winPIBT
    int window;

    // for ecbs
    float suboptimal;

    // for winPIBT
    bool softmode;
  };

  struct VisualConfig {
    bool showicon;
    std::string icon;
  };
};


void setParams(std::string filename,
               Param::EnvConfig* env,
               Param::SolverConfig* solver,
               Param::VisualConfig* visual)
{
  // file open
  std::ifstream file(filename);
  if (!file) {
    std::cout << "error@setParams, file "
              << filename
              <<" cannot read." << "\n";
    std::exit(1);
  }

  // regex
  std::regex r_problem_type = std::regex(R"(PROBLEM_TYPE=(.+))");
  std::regex r_solver_type = std::regex(R"(SOLVER_TYPE=(.+))");
  std::regex r_field = std::regex(R"(field=(.+))");
  std::regex r_agentnum = std::regex(R"(agentnum=(\d+))");
  std::regex r_timesteplimit = std::regex(R"(timesteplimit=(\d+))");
  std::regex r_tasknum = std::regex(R"(tasknum=(\d+))");
  std::regex r_taskfrequency = std::regex(R"(taskfrequency=(\d+[\.]?\d*))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_log = std::regex(R"(log=(\d+))");
  std::regex r_printlog = std::regex(R"(printlog=(\d+))");
  std::regex r_printtime = std::regex(R"(printtime=(\d+))");
  std::regex r_WarshallFloyd = std::regex(R"(WarshallFloyd=(\d+))");
  std::regex r_ID = std::regex(R"(ID=(\d+))");
  std::regex r_window = std::regex(R"(window=(\d+))");
  std::regex r_suboptimal = std::regex(R"(suboptimal=(\d+[\.]?\d*))");
  std::regex r_softmode = std::regex(R"(softmode=(\d+))");
  std::regex r_showicon = std::regex(R"(showicon=(\d+))");
  std::regex r_icon = std::regex(R"(icon=(.+))");

  std::string line, tmpstr;
  std::smatch results;

  // read files
  while (getline(file, line)) {
    if (std::regex_match(line, results, r_problem_type)) {
      tmpstr = results[1].str();
      if (tmpstr == "MAPF") {
        env->PTYPE = Param::PROBLEM_TYPE::P_MAPF;
      } else if (tmpstr == "MAPD") {
        env->PTYPE = Param::PROBLEM_TYPE::P_MAPD;
      } else if (tmpstr == "IMAPF") {
        env->PTYPE = Param::PROBLEM_TYPE::P_IMAPF;
      } else if (tmpstr == "MAPF_STATION") {
        env->PTYPE = Param::PROBLEM_TYPE::P_MAPF_STATION;
      } else if (tmpstr == "IMAPF_STATION") {
        env->PTYPE = Param::PROBLEM_TYPE::P_IMAPF_STATION;
      } else {
        std::cout << "error@setParams, problem type "
                  << tmpstr
                  << " does not exist" << "\n";
        std::exit(1);
      }
    } else if (std::regex_match(line, results, r_solver_type)) {
      tmpstr = results[1].str();
      if (tmpstr == "HCA") {
        env->STYPE = Param::SOLVER_TYPE::S_HCA;
      } else if (tmpstr == "WHCA") {
        env->STYPE = Param::SOLVER_TYPE::S_WHCA;
      } else if (tmpstr == "CBS") {
        env->STYPE = Param::SOLVER_TYPE::S_CBS;
      } else if (tmpstr == "ECBS") {
        env->STYPE = Param::SOLVER_TYPE::S_ECBS;
      } else if (tmpstr == "iECBS") {
        env->STYPE = Param::SOLVER_TYPE::S_iECBS;
      } else if (tmpstr == "PPS") {
        env->STYPE = Param::SOLVER_TYPE::S_PPS;
      } else if (tmpstr == "TP") {
        env->STYPE = Param::SOLVER_TYPE::S_TP;
      } else if (tmpstr == "PIBT") {
        env->STYPE = Param::SOLVER_TYPE::S_PIBT;
      } else if (tmpstr == "winPIBT") {
        env->STYPE = Param::SOLVER_TYPE::S_winPIBT;
      } else {
        std::cout << "error@setParams, solver type "
                  << tmpstr
                  << " does not exist" << "\n";
        std::exit(1);
      }
    } else if (std::regex_match(line, results, r_field)) {
      env->field = results[1].str();
    } else if (std::regex_match(line, results, r_agentnum)) {
      env->agentnum = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_timesteplimit)) {
      env->timesteplimit = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_tasknum)) {
      env->tasknum = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_taskfrequency)) {
      env->taskfrequency = std::stof(results[1].str());
    } else if (std::regex_match(line, results, r_seed)) {
      env->seed = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_log)) {
      env->log = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_printlog)) {
      env->printlog = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_printtime)) {
      env->printtime = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_ID)) {
      solver->ID = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_WarshallFloyd)) {
      solver->WarshallFloyd = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_window)) {
      solver->window = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_suboptimal)) {
      solver->suboptimal = std::stof(results[1].str());
    } else if (std::regex_match(line, results, r_softmode)) {
      solver->softmode = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_showicon)) {
      visual->showicon = std::stoi(results[1].str());
    } else if (std::regex_match(line, results, r_icon)) {
      visual->icon = results[1].str();
    }
  }
}
