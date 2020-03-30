#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

int main() {
  uWS::Hub h;
  ofstream  f;
  f.open("../output.txt");
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  //AA
  int lane=1;
  double ref_vel=0.0; // meter per second
  double max_vel=22.2;

  h.onMessage([&ref_vel, &max_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&f]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        float car_lane;

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          car_speed = car_speed*0.444;
          // find the lane of car is
          if (car_d>0 && car_d<=3) {
            car_lane=0;
          }

         if (car_d>3 && car_d<=5) {
            car_lane=0.5;
          }
         if (car_d>5 && car_d<=7) {
            car_lane=1;
          }
         if (car_d>7 && car_d<=9) {
            car_lane=1.5;
          }
         if (car_d>9 && car_d<=12) {
            car_lane=2;
          }


          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          //AA
          int prev_size=previous_path_x.size();

          json msgJson;

          //AA
          //if(prev_size>0)
	  //{
          //   car_s=end_path_s;
          // }

          f<<"start a new loop, get data from similator"<<endl;
          f<<"end_path_s="<<end_path_s<<"end_path_d="<<end_path_d<<" car_s="<<car_s<<" car_d="<<car_d<<" car_speed="<<car_speed<<endl;
          for(int k=0;k<prev_size;k++){
            f<<"previous_path  x:"<<previous_path_x[k]<<" y:"<<previous_path_y[k]<<endl;
          }
          f<<"checking other cars position and change lane or not"<<endl;


          if(prev_size==0)
	  {
             end_path_s=car_s;
             end_path_d=car_d;
          }




          bool too_close= false;
          bool CHG_lane= false;
          bool safe_change_0=false;
          bool safe_change_1=false;
          bool safe_change_2=false;

          vector<int>  L0_Car(151,0);
          vector<int>  L1_Car(151,0);
          vector<int>  L2_Car(151,0);
          vector<double> L0_Vel(151,0);
          vector<double> L1_Vel(151,0);
          vector<double> L2_Vel(151,0);
          vector<int>  Lane_no_car(3,100);

          // fill in the L0_Car, L0_Vel, Lane_no_car
          for(int i=0; i<sensor_fusion.size();i++)
          {
            double  other_d=sensor_fusion[i][6];
            double  other_s=sensor_fusion[i][5];
            int   dist_s=rint(other_s-car_s);
            //cout<<dist_s<<endl;
            //tell if there is other cars around
            if (dist_s>=-50 && dist_s<=100)
            {

               double other_vx=sensor_fusion[i][3];
               double other_vy=sensor_fusion[i][4];
               double other_speed=sqrt(other_vx*other_vx+other_vy*other_vy);

               // Lane 0
               if (other_d>=0 && other_d<4 )
               {
                 L0_Car[dist_s+50]=1;
                 L0_Vel[dist_s+50]=other_speed;
                //cout<<"left lane: dist_s="<<dist_s<<" speed="<<other_speed<<endl;
                 if (dist_s<Lane_no_car[0] && dist_s>0)
                 {
                   Lane_no_car[0]=dist_s;
                 }
               }

               // Lane 1
               if (other_d>=4 && other_d<8)
               {
                 L1_Car[dist_s+50]=1;
                 L1_Vel[dist_s+50]=other_speed;


                 if (dist_s<Lane_no_car[1] && dist_s>0)
                 {
                   Lane_no_car[1]=dist_s;
                 }
                 //cout<<"middle lane: dist_s="<<dist_s<<" speed="<<other_speed<<endl;


               }
               //Lane 2
               if (other_d>=8 && other_d<=12)
               {
                 L2_Car[dist_s+50]=1;
                 L2_Vel[dist_s+50]=other_speed;
                 if (dist_s<Lane_no_car[2] && dist_s>0)
                 {
                   Lane_no_car[2]=dist_s;
                 }
                 //cout<<"right lane: dist_s="<<dist_s<<" speed="<<other_speed<<endl;
               }
            }
          }

         bool reduce_speed=false;
         double diff_speed_front=2;
         double diff_speed_rear=4;

        // ego car is  in left lane, change to middle lane?
         safe_change_1=true;
         if (car_lane==0)
         {
           for(int k=25; k<=150;k++)
           {
             // car ahead of ego car in 30 meters
             if(k>51 && k<=80 && L0_Car[k]==1) {
               too_close=true;
               cout<<"ahead car speed:"<<L0_Vel[k]<<" ego car speed:"<<car_speed<<endl;
               if (L0_Vel[k]<car_speed-diff_speed_front) {
               reduce_speed=true;
               }
             }

             // a car is very close on lane 1
             if(k>35 && k<80 && L1_Car[k]==1) {
               safe_change_1=false;
             }
             // a faster car behind on lane 1
             if(k<=35 && L1_Car[k]==1 && L1_Vel[k]>car_speed+diff_speed_rear) {
               safe_change_1=false;
             }
             // a slow car ahead
             if(k>=80 && k<=100 && L1_Car[k]==1 && L1_Vel[k]<car_speed-diff_speed_front) {
               safe_change_1=false;
             }



           }

           if (too_close==true && safe_change_1==true)
           {
             lane=1;
           }
           if(Lane_no_car[0] <50 && (Lane_no_car[1]==100 || Lane_no_car[2]==100) && safe_change_1==true)
           {
             lane=1;
           }

         }

         // ego are in right lane,  change to middle lane?
         safe_change_1=true;
         if (car_lane==2)
         {

           for(int k=25; k<=150;k++)
           {
             // care ahead of ego car?
             if(k>51 && k<=80 && L2_Car[k]==1) {
               too_close=true;
               cout<<"ahead car speed:"<<L2_Vel[k]<<" ego car speed:"<<car_speed<<endl;
               if (L2_Vel[k]<car_speed-diff_speed_front) {
                 reduce_speed=true;
               }

             }

            // a car very close
            if(k>35 && k<80 && L1_Car[k]==1) {
               safe_change_1=false;
             }
             // a faster car behind on lane 1
             if(k<=35 && L1_Car[k]==1 && L1_Vel[k]>car_speed+diff_speed_rear) {
               safe_change_1=false;
             }
             // a slow car ahead
             if(k>=80 && k<=100 && L1_Car[k]==1 && L1_Vel[k]<car_speed-diff_speed_front) {
               safe_change_1=false;
             }

           }

           if (too_close==true && safe_change_1==true)
           {
             lane=1;
           }

           if(Lane_no_car[2]<50 &&(Lane_no_car[0]==100 || Lane_no_car[1]==100) && safe_change_1==true)
           {
             lane=1;
           }


         }

         // ego are in middle lane,  change to other?
         safe_change_0=true;
         safe_change_2=true;
         if (car_lane==1)
         {

           for(int k=25; k<=150;k++)
           {
             if(k>51 && k<=80 && L1_Car[k]==1) {
               too_close=true;
               cout<<"Ahead car speed="<<L1_Vel[k]<<" ego car speed:"<<car_speed<<"reduce_speed="<<reduce_speed<<endl;
               if(L1_Vel[k]<car_speed-diff_speed_front){
                 reduce_speed=true;
               }
             }

            // lane 0
            // a car very close
            if(k>35 && k<80 && L0_Car[k]==1) {
               safe_change_0=false;
             }
             // a faster car behind on lane 0
             if(k<=35 && L0_Car[k]==1 && L0_Vel[k]>car_speed+diff_speed_rear) {
               safe_change_0=false;
             }
             // a slow car ahead
             if(k>=80 && k<=100 && L0_Car[k]==1 && L0_Vel[k]<car_speed-diff_speed_front) {
               safe_change_0=false;
             }
            // lane 2
            // a car very close
            if(k>35 && k<80 && L2_Car[k]==1) {
               safe_change_2=false;
             }
             // a faster car behind on lane 2
             if(k<=35 && L2_Car[k]==1 && L2_Vel[k]>car_speed+diff_speed_rear) {
               safe_change_2=false;
             }
             // a slow car ahead
             if(k>=80 && k<=100 && L2_Car[k]==1 && L2_Vel[k]<car_speed-diff_speed_front) {
               safe_change_2=false;
             }

           }

           if (too_close==true)
           {
             //both left and right side are safe to turn
             if (safe_change_0==true && safe_change_2==true)
             {
                if(Lane_no_car[0]>Lane_no_car[2])
                {
                  lane=0;
                }
                else
                {
                  lane=2;
                }

             }

             // lane 2 is safe only
             if (safe_change_0==false && safe_change_2==true)
             {
                lane=2;
             }
             // lane 0 is safe only
             if (safe_change_0==true && safe_change_2==false)
             {
                lane=0;
             }
            }

           if(Lane_no_car[1]<50 && Lane_no_car[0]==100 && safe_change_0==true)
           {
             lane=0;
           }
           if(Lane_no_car[1]<50 && Lane_no_car[2]==100 && safe_change_2==true)
           {
             lane=2;
           }




         }


         if(too_close) {
           if(reduce_speed && car_lane==lane){
            ref_vel-=0.1; //m/s
            cout<<"ref_vel-0.1="<<ref_vel<<" ";
           }

         }
         else if(ref_vel<max_vel)  {
           ref_vel+=0.1;//m/s
           cout<<"ref_vel+0.1="<<ref_vel<<" ";
         }

         cout<<"Dist "<< Lane_no_car[0]<<"  "<<Lane_no_car[1]<<"  "<<Lane_no_car[2];
         cout<<" too_close="<<too_close<<" Reduce_speed="<<reduce_speed<<" car_lane="<<car_lane<<" target="<<lane<<endl;

         if (car_lane==lane) {
           f<<"Stay the same lane: "<<lane <<endl;
         }
         else{
           f<<"Change from lane: "<<car_lane<<" to lane: "<<lane<<endl;
         }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x=car_x;
          double ref_y=car_y;
          double ref_yaw=deg2rad(car_yaw);

          if(prev_size<2)
          {

            double prev_car_x=car_x-cos(ref_yaw); // original was car_yaw
            double prev_car_y=car_y-sin(ref_yaw); // original was car_yaw

            //debug
            f<<"prev_size<2"<<" prev_size="<<prev_size<<endl;
            f<<"car_x="<<car_x<<" car_y="<<car_y<<" car_yaw="<<car_yaw<<" sin(ref_yaw)="<<sin(ref_yaw)<<" cos(ref_yaw)="<<cos(ref_yaw)<<endl;
            f<<"prev_car_x="<<prev_car_x<<" prev_car_y="<<prev_car_y<<" ref_yaw="<<ref_yaw<<endl;

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

            f<<"size of ptsx="<<ptsx.size()<<endl<<endl;
          }
          else
          {

            f<<"prev_size>=2"<<" prev_size="<<prev_size<<endl;

            ref_x=previous_path_x[prev_size-1];
            ref_y=previous_path_y[prev_size-1];

            double ref_x_prev=previous_path_x[prev_size-2];
            double ref_y_prev=previous_path_y[prev_size-2];
            ref_yaw=atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

            f<<"size of ptsx="<<ptsx.size()<<endl;


         }


          vector<double> next_wp0=getXY(end_path_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1=getXY(end_path_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2=getXY(end_path_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          f<<"wp0[0]="<<next_wp0[0]<<"  "<<"wp0[1]="<<next_wp0[1];
          f<<" wp1[0]="<<next_wp1[0]<<"  "<<"wp1[1]="<<next_wp1[1];
          f<<" wp2[0]="<<next_wp2[0]<<"  "<<"wp2[1]="<<next_wp2[1]<<endl;
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          f<<"after add 3 wp points,size of ptsx="<<ptsx.size()<<endl<<endl;

          for (int i=0;i<ptsx.size(); i++)
          {
            double shift_x=ptsx[i]-ref_x; 
            double shift_y=ptsy[i]-ref_y;
            f<<"shift_x="<<shift_x<<" shift_y="<<shift_y<<" ref_yaw="<<ref_yaw<<endl;
            ptsx[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          }


          for (int i=0;i<ptsx.size(); i++)
          {
             f<<"ptsx["<<i<<"]="<<ptsx[i]<<" ptsy["<<i<<"]="<<ptsy[i]<<endl;
          }


          tk::spline s;
          s.set_points(ptsx,ptsy);

          f<<"set spline s"<<endl<<endl;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i=0;i<previous_path_x.size();i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);

          }

          double target_x=30.0;
          double target_y=s(target_x);
          double target_dist=sqrt(target_x*target_x+target_y*target_y);
          double x_add_on=0;

          f<<"target_x="<<target_x<<" target_y="<<target_y<<" target_dist="<<target_dist<<endl;
          f<<"previous_path_x.size()="<<previous_path_x.size()<<endl;

          for (int i=0;i<50-previous_path_x.size();i++)
          {
            double N=target_dist/(0.02*ref_vel); // 
            double x_point=x_add_on+target_x/N;
            double y_point=s(x_point);

            f<<"N="<<N<<" i="<<i<<" local x:"<<x_point<<" y:"<<y_point;

            x_add_on=x_point;

            double x_ref=x_point;
            double y_ref=y_point;

            x_point=x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
            y_point=x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

            x_point+=ref_x;
            y_point+=ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            f<<"map x:"<<x_point<<" y:"<<y_point<<endl;


          }

          //debug
          f<<"-----Below is list of next vals-------------"<<endl;
          for (int i=0; i<next_x_vals.size();i++)
	  {
            f<<"next_x_vals["<<i<<"]="<<next_x_vals[i]<<"next_y_vals["<<i<<"]="<<next_y_vals[i]<<endl;

          }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // debug code
          /* vector<double> next_x_vals_debug;
          vector<double> next_y_vals_debug;

          double dist_inc=0.3;
          for (int i=0; i<50;i++)
          {
            double  next_s=car_s+(i+1)*dist_inc;
            double  next_d=6;
            vector<double> xy=getXY(next_s, next_d,map_waypoints_s, map_waypoints_x,map_waypoints_y);
            next_x_vals_debug.push_back(xy[0]);
            next_y_vals_debug.push_back(xy[1]);

          }

          f<<"-------------------"<<endl;
          for (int i=0; i<next_x_vals_debug.size();i++)
	  {
            f<<"next_x_vals_debug["<<i<<"]="<<next_x_vals_debug[i]<<endl;
            f<<"next_y_vals_debug["<<i<<"]="<<next_y_vals_debug[i]<<endl;

          }*/


          f<<"------------------------"<<endl;
          f<<"Sent data to simulator, finish this loop"<<endl;
          f<<"------------------------"<<endl<<endl<<endl;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
  f.close();
}
