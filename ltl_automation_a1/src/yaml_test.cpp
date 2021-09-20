//
// Created by ziyi on 7/16/21.
//

#include <yaml-cpp/yaml.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    YAML::Node config = YAML::LoadFile("/home/ziyi/code/ltl_ws/src/ltl_automation_a1/config/a1_ts.yaml");

    if (config["SOMETHING"]) {
        std::cout << config["SOMETHING"] << "\n";
    }
    std::cout << config["state_dim"].as<std::vector<std::string>>()[0] << std::endl;
    auto aa = config["actions"].begin();
    std::cout << aa->first.as<std::string>() << std::endl;
    std::cout << config["actions"]["goto_r1"]["attr"]["pose"].as<std::vector<std::vector<double>>>()[0][0] << std::endl;

    return 0;
}