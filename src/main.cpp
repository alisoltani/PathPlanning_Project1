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
#include <list>

using namespace std;

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

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
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

class car {

public:
   unsigned int id;
   double speed;
   double s;
   double d;
};

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

  // save some previous and reference values
  // Start in lane 1;
  static int lane = 1;
  // Reference velocity
  static double ref_vel = 0; // mph
  // Change lane
  static bool change_lane = false;

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

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	int prev_path_size = previous_path_x.size();
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
          	double prev_car_x = 0;
          	double prev_car_y = 0;

          	if (prev_path_size > 0)
          	{
          	  car_s = end_path_s;
          	}

          	bool too_close = false;
          	bool way_too_close = false;
          	bool change_lane_left = false;
          	bool change_lane_right = false;
          	double target_speed = 0.0;

            std::list<car> cars_left_lane;
            std::list<car> cars_right_lane;

            cars_left_lane.clear();
            cars_right_lane.clear();


          	// try to find the relavitve velocity between cars ahead of us
          	for (int i = 0; i < sensor_fusion.size(); i++)
          	{
          	  float d = sensor_fusion[i][6];
          	  //printf("found a car! and it has %f \n", d);

          	  // car in our lane
          	  if ((d < (2 + 4*lane + 2)) && (d > (2 + 4*lane - 2)))
          	  {
          	    double vx = sensor_fusion[i][3];
          	    double vy = sensor_fusion[i][4];
          	    double check_speed = sqrt(vx*vx + vy*vy);
          	    double check_car_s = sensor_fusion[i][5];

          	    check_car_s += ((double)prev_path_size*0.02*check_speed);

          	    if ((check_car_s > car_s) && (check_car_s-car_s < 25))
          	    {
          	      too_close = true;
          	      target_speed = check_speed;

          	      if (check_car_s - car_s < 10)
          	      {
          	        way_too_close = true;
          	      }
          	    }
          	  }
          	  // find cars in the left lane to the left
              if ((d < (2 + 4*(lane-1) + 2)) && (d > (2 + 4*(lane-1) - 2)) && ((2 + 4*(lane-1) - 2) >= 0))
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double)prev_path_size*0.02*check_speed);

                if ((check_car_s-car_s < 30) && (check_car_s - car_s > -5 ))
                {
                  car left_car;
                  left_car.id = i;
                  left_car.d = d;
                  left_car.s = check_car_s;
                  left_car.speed = check_speed;
                  cars_left_lane.push_back(left_car);
                }

              }

              // find cars in lane to the right
              if ((d < (2 + 4*(lane+1) + 2)) && (d > (2 + 4*(lane+1) - 2)) && ((2 + 4*(lane+1) - 2) < 12))
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double)prev_path_size*0.02*check_speed);

                if ((check_car_s-car_s < 30) && (check_car_s - car_s > -5 ))
                {
                  car right_car;
                  right_car.id = i;
                  right_car.d = d;
                  right_car.s = check_car_s;
                  right_car.speed = check_speed;
                  cars_right_lane.push_back(right_car);
                }

              }
          	}
          	if (too_close)
          	{
          	  //printf("Too close? %d \n", too_close);
              //printf("Go left? %d \n", change_lane_left);
              //printf("Go right? %d \n", change_lane_right);

              // can I go into the left lane?
              if (cars_left_lane.empty())
              {
                change_lane_left = true;
              }
              if (cars_right_lane.empty())
              {
                change_lane_right = true;
              }
          	}


          	if (too_close)
          	{
          	  if (way_too_close)
          	  {
          	    ref_vel -= 0.448;
          	  }
          	  else
          	  {
                //if (ref_vel - target_speed < 0.224)
                //{
                //  ref_vel = target_speed;
                //}
                //else
                //{
                  ref_vel -= 0.224; // 5mps
                //}
          	  }

          	  if ((change_lane_left) && lane > 0)
          	  {
          	    lane = lane - 1;
          	  }
          	  else if ((change_lane_right) && lane < 2)
          	  {
          	    lane = lane + 1;
          	  }
          	}
          	else if ((ref_vel < 49.5) && (ref_vel > 15))
          	{
          	  ref_vel += 0.224; // 5mps
          	}
          	else if (ref_vel < 15)
          	{
          	  ref_vel += 0.448; // just under 10mps
          	}

            /*if (!too_close)
            {
              if ((lane = 0) && (change_lane_right))
              {
                lane = 1;
              }
              if ((lane = 2) && (change_lane_left))
              {
                lane = 1;
              }
            }*/

          	// Creates a list of evenly spaced (x,y) waypoints to be later filled in with the spline
          	vector<double> x_points;
          	vector<double> y_points;

          	if (prev_path_size < 2) // we dont have points from the previous path waypoints
          	{
          	  prev_car_x = car_x - cos(car_yaw);
          	  prev_car_y = car_y - sin(car_yaw);
          	}
          	else // we can use previous points to update our reference points
          	{
          	  ref_x = previous_path_x[prev_path_size-1];
          	  ref_y = previous_path_y[prev_path_size-1];
          	  prev_car_x = previous_path_x[prev_path_size-2];
          	  prev_car_y = previous_path_y[prev_path_size-2];
          	  ref_yaw = atan2(ref_y-prev_car_y,ref_x-prev_car_x);
          	}

            // we now have 2 points to push, (car_x, car_y) and (prev_car_x,prev_car_y)
            x_points.push_back(prev_car_x);
            x_points.push_back(ref_x);
            y_points.push_back(prev_car_y);
            y_points.push_back(ref_y);


            // Now I will add 3 points evenly spaced at 30m each in Frenet coordinates
            int lane_number = (2 + 4*lane);
            vector<double> waypoint1 = getXY(car_s + 30, lane_number, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> waypoint2 = getXY(car_s + 60, lane_number, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> waypoint3 = getXY(car_s + 90, lane_number, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //vector<double> waypoint4 = getXY(car_s + 100, lane_number, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            x_points.push_back(waypoint1[0]);
            x_points.push_back(waypoint2[0]);
            x_points.push_back(waypoint3[0]);
            //x_points.push_back(waypoint4[0]);

            y_points.push_back(waypoint1[1]);
            y_points.push_back(waypoint2[1]);
            y_points.push_back(waypoint3[1]);
            //y_points.push_back(waypoint4[1]);


            // shift car reference to make the car be at (0,0) and driving at 0 degrees reference plane
            for (int i = 0; i < x_points.size(); i++)
            {
              double shift_x = x_points[i] - ref_x;
              double shift_y = y_points[i] - ref_y;

              x_points[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
              y_points[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
            }


            // create spline
            tk::spline s;

            s.set_points(x_points, y_points);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// Add the previous waypoint points that the car has yet to pass
          	for (int i = 0; i < previous_path_x.size(); i++)
          	{
          	  next_x_vals.push_back(previous_path_x[i]);
          	  next_y_vals.push_back(previous_path_y[i]);
          	}

          	// Calculate the points to be taken inside the spline while traveling at the desired
          	// speed

          	// In this section, I try to generate the waypoints assuming that we are moving linearly
          	// which in some maneuvers was not really the case and can cause issues.

          	double target_x = 25.0; // Chose a random point
          	double target_y = s(target_x);
          	double target_dist = sqrt(target_x*target_x + target_y*target_y);

          	double x_add_on = 0;

          	// the number of points in the waypoints is 50
          	for (int i = 0; i <= (50 - previous_path_x.size()); i++)
          	{
          	  double N = target_dist/(0.02*ref_vel/2.24); // This will give as the step for each point assuming the target velocity
          	  double x_point = x_add_on + target_x/N;
          	  double y_point = s(x_point);

          	  x_add_on = x_point;

          	  double x_ref = x_point;
          	  double y_ref = y_point;

          	  // now I have to rotate back the coordinates
          	  x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          	  y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

          	  x_point += ref_x;
          	  y_point += ref_y;

          	  next_x_vals.push_back(x_point);
          	  next_y_vals.push_back(y_point);

          	}

            json msgJson;

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
