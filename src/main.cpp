#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
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
struct vehicle {
  int lane;
  double s;
  double d; 
  double v;
  string state;
};

const float MAX_SPEED = 49.5;
const float EFFICIENCY = 10;
const float COLLISION = 100;
const float LANE_CHANGE = 1;
const float BUFFER = 2;
vector<float> weight_list = {EFFICIENCY, COLLISION, LANE_CHANGE};

  
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


    // start in lane 1
    int lane = 1;
    // jhave a reference valocity to target
    string state = "KL"; // current state of the vehicle with default of maintaining lane
    double ref_val  = 0;
  h.onMessage([&state, &ref_val, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    int lanes_available = 3;
    map <string,int> lane_direction = { {"LCL",-1}, {"RCL",1},{"KL",0}};

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
            cout << " car yaw, car_s, car_d, car_x, car_y" << car_yaw << "\t" << car_s << "\t" << car_d << "\t" << car_x 
              << "\t" << car_y<< endl;
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// 
            // TODO: define a path made up of (x
            // check car lane and set possible state change options and get interested lanes
            int current_car_lane = (car_d)/4;
            float current_d = car_d;
            bool too_close = false;
            int prev_size = previous_path_x.size();
            if (prev_size > 0 ) { 
              car_s = end_path_s; 
              car_d = end_path_d;
            }
            float lane_changing = 0;
            int car_lane = (car_d)/4;
            cout << "car lane and car d" << current_car_lane << "\t" << car_lane << "\t" << current_d << "\t" << car_d << endl;
            if ((current_car_lane != lane ) || (fabs((car_lane*4 + 2 ) - current_d)>1) ) {
              lane_changing = 1;
            }
            vector <string> succ_states;
            succ_states.push_back("KL");
            if (car_lane !=0) {
                succ_states.push_back("LCL");
              }
            if (car_lane!=lanes_available-1) {
              succ_states.push_back("RCL");
            
            }
            vehicle car;

            car.d = car_d;
            car.s = car_s;
            car.v = car_speed;
            car.lane = car_d / 4;

            vector<vehicle> vehicles(sensor_fusion.size());

            for (int i = 0; i < sensor_fusion.size(); i++) {
              double d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double) prev_size * .02 * check_speed); 
              vehicles[i].s = check_car_s;
              vehicles[i].d = sensor_fusion[i][6];
              vehicles[i].v = check_speed;
              vehicles[i].lane = d / 4;
            }
            string best_state = "KL";
            float min_cost = 100;
            for (auto new_state : succ_states) {
              float collision_cost = 0;
              float lane_change_cost = 0;
              float buffer_cost = 0;
              float dist_ahead = 90;
              float dist_behind = 90;
              float lane_speed = MAX_SPEED;

              int new_lane = car_lane + lane_direction[new_state];
              
              lane_change_cost = fabs(new_lane - car_lane);
              
              float new_d = new_lane * 4;
              int min_s = car_s + 150;
              int max_s = 0;
              for (vehicle v : vehicles){
               if (fabs(v.s - car.s) > 300 ) {
                  continue;
                }  
                double nearest_dist = sqrt(pow(car.s - v.s,2) + pow(new_d - v.d, 2));
                if (v.lane == new_lane && nearest_dist < 2*1.5) {  // assuming vehicle radius on 1 m.
                  collision_cost = 1;
                }
                if (v.lane == new_lane && v.s > car.s && v.s < min_s) {
                  min_s = v.s;
                  lane_speed = v.v*2.24;
                  dist_ahead = v.s - car.s;
                }
                if (v.lane == new_lane && v.s <= car.s && v.s > max_s) {
                  max_s = v.s;
                  dist_behind = car.s - v.s;
                }

              }
              lane_speed = fmin(lane_speed, MAX_SPEED);
              float  efficiency_cost =  (MAX_SPEED- lane_speed )/ MAX_SPEED;
              float state_change_cost = 0;
              if (lane_changing==1 && (new_lane != lane))
                state_change_cost = 5;
              if (new_state != "KL") 
                buffer_cost = fmax((90 - dist_behind)/dist_behind,0.0);
              float total_cost = buffer_cost *BUFFER + efficiency_cost * EFFICIENCY + 
                + state_change_cost + collision_cost*COLLISION + lane_change_cost * LANE_CHANGE;
              cout << "ln cng: " << lane_changing << " " << state <<" new st " << new_state << " spd  " << lane_speed << " buf" << buffer_cost << "stcost " << state_change_cost << " totcost " << total_cost << endl;
                if (total_cost < min_cost) {
                  min_cost = total_cost;
                  best_state = new_state;
                }
            }
            lane = car_lane + lane_direction[best_state];
            state = best_state;
            cout << "new car state " << best_state << " and lane " << lane << endl; 
//            choose_next_state(car, vehicles, succ_states);
            // check if car in my lane
            for (int i = 0; i < sensor_fusion.size(); i++) {
              double d = sensor_fusion[i][6];
              if (d > lane * 4 && d < (lane+1) * 4) {
                  // in my lane check speed
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                check_car_s += ((double) prev_size * .02 * check_speed); 
                if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)){
                 too_close = true;
                }
              }
            }
            // check trajectory for keeping lane, left change, right change
            // left change check if already not in left most lane, similarly on the right
            // write methods for get vechiles ahead in each of the interested lane
            if (too_close) {
              ref_val -= .224;
            } 
            else if (ref_val < MAX_SPEED) {
              ref_val += .224;
            }
            cout << "too close and ref val " << too_close << "\t" << ref_val << "\t" << MAX_SPEED;




            vector<double> ptsx;
            vector<double> ptsy;
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            if (prev_size < 2) {
              double prev_carx = car_x - cos(ref_yaw);
              double prev_cary = car_y - sin(ref_yaw);
              ptsx.push_back(prev_carx);
              ptsy.push_back(prev_cary);
              ptsx.push_back(car_x);
              ptsy.push_back(car_y);                 
              //cout << car_yaw << '\t' << ref_yaw << '\t' << prev_cary << '\t' << car_y << endl;
            }
            else {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
              ptsx.push_back(ref_x_prev);
              ptsy.push_back(ref_y_prev);                 
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y);
              cout << car_yaw << '\t' << ref_yaw << '\t' << ref_y_prev << '\t' << ref_y << endl;
            }

            vector<double> p3  =  getXY(car_s+30, (2 + 4 * lane), map_waypoints_s,map_waypoints_x, map_waypoints_y);
            vector<double> p4  =  getXY(car_s+60, (2 + 4 * lane), map_waypoints_s,map_waypoints_x, map_waypoints_y);
            vector<double> p5  =  getXY(car_s+90, (2 + 4 * lane), map_waypoints_s,map_waypoints_x, map_waypoints_y);

            ptsx.push_back(p3[0]);
            ptsx.push_back(p4[0]);
            ptsx.push_back(p5[0]);

            ptsy.push_back(p3[1]);
            ptsy.push_back(p4[1]);
            ptsy.push_back(p5[1]);
            // change to car coordinates
            for (int i = 0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw)); 
            }
            // add to spline
            tk::spline s;
            s.set_points(ptsx, ptsy);

            double dist_inc = .5;
            double angle = ref_yaw;
            /*
            for (int i = 0; i < 50 - prev_size; i++) {
              //double angle = ref_yaw + (i+1)*pi()/100.0;
              //cout << angle << endl; 
              double dist_i = dist_inc * (i + 1);
              next_x_vals.push_back(ref_x+(dist_i)*cos(angle));
              next_y_vals.push_back(ref_y+(dist_i)*sin(angle));
              ref_x += dist_i*cos(angle);
              ref_y += dist_i*sin(angle);
            }
            */
            
            // add any previous points 
            for (int i = 0; i < prev_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            double target_x = 30;
            double target_y = s(target_x); 
            double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
            double x_add = 0;

            
            for (int i = 0; i < 50 - prev_size; i++) {
             /* if (too_close) {
                ref_val -= .224;
              } 
              else if (ref_val < 49.5) {
                ref_val += .224;
              }
              */
              double N = target_dist/(ref_val * .02 / 2.24) ;
              double x_point = x_add + (target_x) / N;
              double y_point = s(x_point); 
              x_add = x_point;
              double x_ref = x_point;
              double y_ref = y_point;
              
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)); 

              x_point += ref_x; 
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }



            /*

            for (int i = 0; i < 50 - prev_size; i++) {
              double next_s = car_s + (i+1)*dist_inc;
              double next_d = 6;
              vector<double> xy = getXY(next_s, next_d, map_waypoints_s,map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
              //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
              //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            }
           */ 
            // END
            //
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
