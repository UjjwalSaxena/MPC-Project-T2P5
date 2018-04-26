#include "PID.h"
// #include <chrono>
#include <iostream>
#include <string>
// #include <ctime>
// using namespace std;
// using namespace std::chrono;
// using std::chrono::system_clock;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// double timestamp=0.0;
// double delta_t=1.0;
// double timestamp=0;
// high_resolution_clock::time_point today = high_resolution_clock::now();
// std::time_t t = high_resolution_clock::to_time_t ( today );
void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp=Kp;
    this->Ki=Ki;
    this->Kd=Kd;
    this->d_error = 0;
    this->p_error = 0;
    this->i_error = 0;
    // today = high_resolution_clock::now();
    // t = high_resolution_clock::to_time_t ( today );
    // timestamp= std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void PID::UpdateError(double cte) {

    // double curr_timestamp= std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); 
    // delta_t= (curr_timestamp- timestamp)/1000;
    // std::cout << " td:"<<delta_t;
    // d_error = (cte - p_error);/delta_t ;
    d_error = (cte - p_error); // differential error
    p_error = cte; // proportional error
    i_error += cte; // integral error
    // today = high_resolution_clock::now();
    // std::time_t tt = high_resolution_clock::to_time_t ( today );
    // // std::cout << "today is: " << ctime(&tt)<<" ts:"<<tt;
    // std::cout << " td:"<<tt-t;
    // t=tt;
    
    
    // timestamp=curr_timestamp;
}

double PID::TotalError() {
    return -Kp*p_error -Kd*d_error -Ki*i_error;

}

