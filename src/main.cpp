#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  if(x==0.0)
    return coeffs[0];
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }

  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
          double dt = 0.1; // timestep
          const double Lf = 2.67; 
          const double coeff_miles_per_hour_to_m_per_sec=0.44704;



          Eigen::VectorXd waypoints_x(ptsx.size());
          Eigen::VectorXd waypoints_y(ptsy.size());



          v=v*coeff_miles_per_hour_to_m_per_sec; //converting to m/s

          // transform waypoints to be from car's perspective
          // this means we can consider px = 0, py = 0, and psi = 0
          // greatly simplifying future calculations
          for(unsigned int i=0; i < ptsx.size(); i++){
      	    double x = ptsx[i] - px;
      	    double y = ptsy[i] - py;

      		//rotation of coordinates
      	    ptsx[i] =  x*cos(psi) + y*sin(psi);
      	    ptsy[i] = -x*sin(psi) + y*cos(psi);
            waypoints_x[i] = ptsx[i];
            waypoints_y[i] = ptsy[i];
      	  }



          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);  //generating coeff from x,y cooerdinates of the line to be followed.
          double cte = polyeval(coeffs, 0);  // px = 0, py = 0
          double epsi = -atan(coeffs[1]);  // p


          Eigen::VectorXd state(6);
          double x_t1 = 0+ v*dt;
          double y_t1 = 0.0;
          double psi_t1 = - v*steer_value*dt/Lf;
          double v_t1 = v + throttle_value*dt;
          double cte_t1 = cte + v*sin(epsi)*dt;
          double epsi_t1 = epsi + psi_t1;
          // cout<<"cte: "<<cte<<endl;

          state << x_t1, y_t1, psi_t1, v_t1, cte_t1, epsi_t1;
          mpc.Solve(state, coeffs); 



          json msgJson;
          double steer=-mpc.steering/ deg2rad(25);
          msgJson["steering_angle"] = steer;
          msgJson["throttle"] = mpc.throttle*(1-fabs(steer)); // decreasing throttle on turns
          msgJson["mpc_x"] = mpc.mpc_x_vals;
          msgJson["mpc_y"] = mpc.mpc_y_vals;
          msgJson["next_x"] = ptsx;
          msgJson["next_y"] = ptsy;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency

          this_thread::sleep_for(chrono::milliseconds(100));
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
