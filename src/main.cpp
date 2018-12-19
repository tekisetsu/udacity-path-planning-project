#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "behavior.cpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double PREDICTION_TIME_S = 1.0;
double PERIOD_S = 0.020;
int NUMBER_OF_PERIODS = (int) PREDICTION_TIME_S/PERIOD_S;

// we decrease slightly the limits to avoid flags
double const SAFETY_DISTANCE_M = 30.0; // m
double const LANE_WIDTH = 4.0; // m
double const MAX_JERK = 9.5; // m.s-3
double const MAX_ACCELERATION = 9.5; // m.s-2
double const MAX_SPEED_MPS = 48.0 * 1609.34 / 3600.0; // m.s-1

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

map<int, vector<vector<double>>> predict(vector<vector<double>> sensor_fusion) {

  map<int, vector<vector<double>>> prevision_by_lane = { { 0, {} }, { 1, {} }, { 2, {} } };

  for(auto neighbour_car : sensor_fusion)
  {
    vector<double> car_predicted_path;

    //getting data from sensor_fusion
    double car_id = neighbour_car[0];
    double v_x = neighbour_car[3];
    double v_y = neighbour_car[4];
    double s = neighbour_car[5];
    double d = neighbour_car[6];
    int car_lane = (int) d/LANE_WIDTH;

    if( car_lane < 0 && car_lane > 2) {
      continue;
    }

    double s_dot = sqrt(pow(v_x,2) + pow(v_y,2));
    car_predicted_path.push_back(car_id);
    car_predicted_path.push_back(s_dot);

    for(float t = 0.0; t <= PREDICTION_TIME_S; t+=PERIOD_S)
    {
      double next_s = s + s_dot*t;
      car_predicted_path.push_back(next_s);
    }

    prevision_by_lane[car_lane].push_back(car_predicted_path);
  }

  //sort by s
  for(int i =0; i<= 2; ++i) {
    sort(prevision_by_lane[i].begin(), prevision_by_lane[i].end(),
      [](const vector<double>& a, const vector<double>& b) {return a[2] < b[2];}
    );
  }
  return prevision_by_lane;
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            int car_lane = (int) car_d/LANE_WIDTH;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            int prev_size = previous_path_x.size();

            //prediction
            auto prevision_by_lane = predict(sensor_fusion);

            //behavior: in this part we will define the goals (desired lane and desired speed)
            double desired_speed;
            double desired_lane;
            
            vector<double> current_lane_front_car;
            for(auto prediction : prevision_by_lane[car_lane]) {
              if(prediction[2] < car_s) {
                continue;
              } else {
                current_lane_front_car = prediction;
                break;
              }
            }

            if(!current_lane_front_car.empty() && (current_lane_front_car[prev_size+1] - end_path_s < SAFETY_DISTANCE_M) ) // There is a car in front of us
            {
              //std::cout << "There is a car in front of us" << std::endl;
              // check if we can go left and then right
              bool will_switch = false;
              if(car_lane >= 1) {
                bool can_go_left = canGoToLane(prevision_by_lane[car_lane-1], prev_size, end_path_s);
                if(can_go_left) {
                  will_switch = true;
                  desired_speed = MAX_SPEED_MPS;
                  desired_lane = car_lane-1;
                }
              }

              if(!will_switch && car_lane < 2) {
                bool can_go_right = canGoToLane(prevision_by_lane[car_lane+1], prev_size, end_path_s);
                if(can_go_right) {
                  will_switch = true;
                  desired_speed = MAX_SPEED_MPS;
                  desired_lane = car_lane+1;
                }
              }

              if(!will_switch) {
                //std::cout << "There is a car in front of us, reducing the speed to : " << desired_speed*3600.0/1609.34 << std::endl;
                desired_speed = current_lane_front_car[1];
                desired_lane = car_lane;
              }
            } else {
              desired_speed = MAX_SPEED_MPS;
              desired_lane = car_lane;
            }

            //cout << "desired_speed : " << desired_speed << std::endl;
            //cout << "desired_lane : " << desired_lane << std::endl;

            vector<double> ptsx;
            vector<double> ptsy;

            // switching the coordinates system
            double ref_s;
            double ref_x;
            double ref_y;
            double ref_yaw;
            double car_speed_end_prevision;

            // Forcing the will-be fitted polynom to have a yaw equal to the current one to have a good continuity.
            // We can take the 2 last points of the previous points if we have previous points
            // If we don't we create a point behind the current position of the car so that the line formed with both these points
            // will have a angle equal to the current yaw

            if ( prev_size < 2 ) {

                ref_x = car_x;
                ref_y = car_y;
                ref_s = car_s;
                ref_yaw = deg2rad(car_yaw);

                // creating a point that is behind the current position of the car so the 
                // the line created by these two point will make an angle equal to the current yaw of the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);

                car_speed_end_prevision = car_speed* 1609.34 / 3600.0; // in m.s-1
            } else {

              ref_s = end_path_s;
              // Use the last two points.
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

              car_speed_end_prevision = sqrt(pow(ref_x-ref_x_prev,2) + pow(ref_y-ref_y_prev,2))/PERIOD_S;
            }

            // Taking points that are 30, 60 and 90m and that belong to the lane we want to go or stay on
            // We will feed these points to our polynom fitting library
            vector<double> next_wp0 = getXY(ref_s + 30, 2 + 4*desired_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(ref_s + 60, 2 + 4*desired_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(ref_s + 90, 2 + 4*desired_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);


            // Using the car coordinate system
            for ( int i = 0; i < ptsx.size(); i++ ) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // Create the polynom.
            tk::spline s;
            s.set_points(ptsx, ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            //start with the previous values given
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // meh approximation
            // Because the spline is a convex function on the small interval in which we are in,
            // we will take a poiint that is far from the current position, and take the x position 
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            for( int i = 1; i < 50 - prev_size; i++ ) {

              double current_acceleration;

              //std::cout << "car speed end prevision: " <<  car_speed_end_prevision*3600.0/1609.34 << std::endl;
              //std::cout << "desired_speed: " <<  desired_speed*3600.0/1609.34 << std::endl;

              if ( car_speed_end_prevision <= desired_speed ) {
                current_acceleration = min(MAX_ACCELERATION, (desired_speed - car_speed_end_prevision)/PERIOD_S);
              } else {
                current_acceleration = max(-MAX_ACCELERATION, (desired_speed - car_speed_end_prevision)/PERIOD_S);
              }

              car_speed_end_prevision = car_speed_end_prevision + current_acceleration*PERIOD_S;

              double N = target_dist/(PERIOD_S*car_speed_end_prevision);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point ;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }


          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
