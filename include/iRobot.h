#pragma once
#include <string>

class IRobot {
protected:
    const std::string IP;
public: 
    IRobot(const std::string &IP) : IP(IP) {}
    virtual ~IRobot() = default;

    virtual bool conn_flange_shm(const std::string &shm_name) = 0;
    virtual void disconnect() = 0;
};
