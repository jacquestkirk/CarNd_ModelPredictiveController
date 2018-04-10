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



class TimeTracker
{
public:
	std::chrono::steady_clock::time_point lastTime;
	std::chrono::steady_clock::time_point currentTime;
	vector<double> times;
	double elapsedTime_ms;

	double averageElapsedTime_ms=0;
	int count;
	int max_count;
	bool validLastSample;
	bool validFirstSample;

	TimeTracker(int maxCount)
	{
		max_count = maxCount;
		count = 0;
		validLastSample = false;
		validFirstSample = false;
		times = {};
	}

	void currentTimeToLastTime()
	{
		lastTime = currentTime;
		validLastSample = true;
	}
	void getLastTime()
	{
		lastTime = std::chrono::steady_clock::now();
		validLastSample = true;
	}
	void getCurrentTime()
	{
		validFirstSample = true;
		currentTime = std::chrono::steady_clock::now();
	}
	void FindElapsedTime_ms()
	{
		if (validFirstSample && validLastSample)
		{
			elapsedTime_ms = (double)std::chrono::duration_cast<std::chrono::microseconds>(currentTime - lastTime).count() / 1000000;

			addToAverage(elapsedTime_ms);
		}
	}

	

	void addToAverage(double newVal)
	{
		count += 1;

		if (count > max_count)
		{
			times.erase(times.begin());
			count = max_count;
		}

		times.push_back(newVal);

		double sum = 0;
		for (int i = 0; i < count; i++)
		{
			sum += times[i];
		}

		averageElapsedTime_ms = sum / count;
	}
};

int main() {
  uWS::Hub h;

  TimeTracker latencyTracker = TimeTracker(5);
  TimeTracker dtTracker = TimeTracker(5);
  
  // MPC is initialized here!
  MPC mpc;
#ifdef UWS_VCPKG
  h.onMessage([&mpc, &latencyTracker, &dtTracker](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
	  uWS::OpCode opCode) {
#else
  h.onMessage([&mpc, &latencyTracker, &dtTracker](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
	  uWS::OpCode opCode) {
#endif

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
		  double old_steer = j[1]["steering_angle"];
		  double a = j[1]["throttle"];
		  double delta = -old_steer;

		  //calculate average dt over the last couple of samples
		  latencyTracker.getLastTime();

		  //calculate the state of the car after a delay dt
		  const double Lf = 2.67;

		  double x_end = px;//(px + v * cos(psi) * delay);
		  double y_end = py;//(py + v * sin(psi) * delay);
		  double psi_end = psi;//(psi + v * delta / Lf * delay);
		  double v_end = v;//(v + a * delay);

		  //cout << "x_end"<<x_end << endl;
		  //cout << "y_end" << y_end << endl;
		  //cout << "psi_end" << psi_end << endl;
		  //cout << "v_end" << v_end << endl;


		  vector<double> ptsx_car = {};
		  vector<double> ptsy_car = {};

		  //calculate in car reference frame
		  for (int i = 0; i < (int)ptsx.size(); i++)
		  {
			  double px_car = ptsx[i] - x_end;
			  double py_car = ptsy[i] - y_end;

			  double px_rot_car = (px_car * cos(- psi_end)) - (py_car *sin(- psi_end));
			  double py_rot_car = (px_car * sin(- psi_end)) + (py_car *cos(- psi_end));

			  ptsx_car.push_back(px_rot_car);
			  ptsy_car.push_back(py_rot_car);

		  }

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */


		  //convert to Eigen::VectorXd

		  Eigen::VectorXd ptsx_eigen(ptsx_car.size());
		  Eigen::VectorXd ptsy_eigen(ptsx_car.size());

		  for (int i = 0; i < (int)ptsx_car.size(); i++)
		  {
			  ptsx_eigen[i] = ptsx_car[i];
			  ptsy_eigen[i] = ptsy_car[i];
		  }


		  //fit a polynomial to the car reference framed points

		  Eigen::VectorXd coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);

		  //std::cout << "x: " << ptsx_eigen << std::endl;
		  //std::cout << "y: " << ptsy_eigen << std::endl;
		  //std::cout << "coeffs: " << coeffs << std::endl;
		  

		  //calcualate errors
		  //double cte = polyeval(coeffs, px) - py;
		  double cte = polyeval(coeffs, 0); //commented above line is reduced to this because we centered around car frame
		  //double epsi = psi - atan(coeffs[1] + coeffs[2] * px + coeffs[3] *px *px); //derivative of the fit line
		  double epsi = 0 - atan(coeffs[1]); ////commented above line is reduced to this because we centered around car frame (px = 0, psi = 0)

		  //double steer_value = j[1]["steering angle"];
		  //double throttle_vaue = j[1]["throttle"];

		  //assemble the state
		  Eigen::VectorXd state(6);
		  //state << px, py, psi, v, cte, epsi;
		  //compensate for delay

		  double delay;
		  if (latencyTracker.validFirstSample)
		  {
			  delay = latencyTracker.averageElapsedTime_ms;
		  }
		  else
		  {
			  delay = 0;
		  }

		  double x_latency = v*delay;//(px + v * cos(psi) * delay);
		  double y_latency = 0;//(py + v * sin(psi) * delay);
		  double psi_latency = v * delta / Lf * delay;//(psi + v * delta / Lf * delay);
		  double v_latency = (v + a * delay);
		  double cte_latency = cte + v * sin(epsi) * delay;
		  double epsi_latency = epsi + v * delta / Lf * delay;

		  state << x_latency, y_latency, psi_latency, v_latency, cte_latency, epsi_latency; //simplify due to frame of reference change
		  //state << 0, 0, 0, v_end, cte, epsi; //simplify due to frame of reference change

		  //Find the optimal values
		  //mpc.SetDt(dtTracker.averageElapsedTime_ms);

		  vector<double> vars = mpc.Solve(state, coeffs);


          double steer_value;
          double throttle_value;



		  steer_value =  -vars[0]/deg2rad(25)/Lf;  //negative sign beacause simulator steers clockwise
		  throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

		  mpc_x_vals = mpc.GetxPts(); 
		  mpc_y_vals = mpc.GetYPts();


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

		  double x_step = 2.5;
		  int num_points = 25;

		  for (int i = 1; i < num_points; i++) {
			  double x_val = x_step * i; 
			  next_x_vals.push_back(x_val);
			  next_y_vals.push_back(polyeval(coeffs, x_val));
		  }

		  //next_x_vals = ptsx_car;
		  //next_y_vals = ptsy_car;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
		  

          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
#ifdef UWS_VCPKG
		  // code fixed for latest uWebSockets
		  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		  // leave original code here
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
		  latencyTracker.getCurrentTime();
		  latencyTracker.FindElapsedTime_ms();
		  double dtCommand_sec = latencyTracker.averageElapsedTime_ms;


		  double dtSamples_sec;

		  if(dtTracker.validLastSample)
		  {
			  dtTracker.FindElapsedTime_ms();
			  dtSamples_sec = dtTracker.averageElapsedTime_ms;

			  dtTracker.currentTimeToLastTime();
			  dtTracker.getCurrentTime();
		  }
		  else if(dtTracker.validFirstSample)
		  {
			  dtSamples_sec = 0.1;

			  dtTracker.currentTimeToLastTime();
			  dtTracker.getCurrentTime();

		  }
		  else
		  {
			  dtSamples_sec = 0.1;
			  dtTracker.getCurrentTime();
		  }
		  

		  std::cout << "time between samples: " << dtSamples_sec << std::endl;
		  std::cout << "delay: " << dtCommand_sec << std::endl;



          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";

#ifdef UWS_VCPKG
		// code fixed for latest uWebSockets
		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		// leave original code here
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
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

#ifdef UWS_VCPKG
  // code fixed for latest uWebSockets
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
#else
  // leave original code here
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef UWS_VCPKG
  // code fixed for latest uWebSockets
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
#else
  // leave original code here
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
#endif
#ifdef UWS_VCPKG
	  // code fixed for latest uWebSockets
	  ws->close();
#else
	  // leave original code here
	  ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
