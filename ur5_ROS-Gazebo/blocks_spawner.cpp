// spawn the red blocks on the conveyor belt
// and give them initial speed (by apply_body_wrench) to slide on conveyor

//ros communications:
  // spawn model throught gazebo service: /gazebo/spawn_urdf_model
  // initialize blocks speed: /gazebo/apply_body_wrench
  // get urdf file path of blocks from parameter servicer
  //publish all current blocks through topic: /current_blocks

  #include <ros/ros.h>
  #include <iostream>
  #include <sstream>
  #include <fstream>
  #include <string>
  #include <urdf/model.h>
  #include <gazebo_msgs/SpawnModel.h>
  #include <gazebo_msgs/ApplyBodyWrench.h>
  #include <std_msgs/Float32MultiArray.h>
  #include <gazebo_msgs/SetModelState.h>
  #include <ur5_notebook/blocks_info.h>
  #include <ur5_notebook/Tracker.h>
  #include "pugixml.hpp"
  #include <vector>

  //int to string converter
  std::string intToString (int a) {
     std::stringstream ss;
     ss << a;
     return ss.str();
  }

  std::vector<std::vector<int>> streamOfBoxes;
  ros::Publisher block_info_publisher;
  ros::ServiceClient spawnClient;
  ros::ServiceClient wrenchClient;
  ros::ServiceClient setstateClient;
  bool get_red_path;
  std::string red_box_path;
  std::vector<int> maxSize={20, 20, 20};
  std::vector<int> minSize={10, 10, 10};

  bool isValidItem(std::vector<int> &item){
      for (int i=0;i<3;i++){
          if(item[i]<minSize[i] || item[i]>maxSize[i])
            return false;
      }
      return true;
  }
  void generateStreamOfBoxes(){

     std::vector<std::vector<int>> invalid;
      invalid.push_back(std::vector<int>({40, 60, 40, 0, 0, 0}));
      while(invalid.size()>0){
          int invalidItemIndex=rand()%invalid.size();
          std::vector<int> invalidItem= invalid[invalidItemIndex];
          invalid.erase(invalid.begin() + invalidItemIndex);
          std::vector<int> axis;
          if(invalidItem[0]>maxSize[0])
            axis.push_back(0);
          if(invalidItem[1]>maxSize[1])
            axis.push_back(1);
          if(invalidItem[2]>maxSize[2])
            axis.push_back(2);
          int axisChosen=axis[rand()%axis.size()];
          int cutLen=rand()%(invalidItem[axisChosen]-2*minSize[axisChosen]);
          std::vector<int> item1={invalidItem[0], invalidItem[1], invalidItem[2],invalidItem[3], invalidItem[4], invalidItem[5]};
          std::vector<int> item2={invalidItem[0], invalidItem[1], invalidItem[2],invalidItem[3], invalidItem[4], invalidItem[5]};
          item1[axisChosen]=minSize[axisChosen]+cutLen;
          item2[axisChosen]=invalidItem[axisChosen]-item1[axisChosen];
          item2[3+axisChosen]=item2[3+axisChosen]+item1[axisChosen];
          if(isValidItem(item1))
            streamOfBoxes.push_back(item1);
          else 
            invalid.push_back(item1);
          if(isValidItem(item2))
            streamOfBoxes.push_back(item2);
          else 
            invalid.push_back(item2);

      }
      //streamOfBoxes.push_back(std::vector<int>({20,30,10}));
  }
  void pickedUpCallback(const ur5_notebook::Tracker &tracker){
      gazebo_msgs::SpawnModel::Request spawn_model_req;
      gazebo_msgs::SpawnModel::Response spawn_model_resp;
      gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
      gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;
      gazebo_msgs::SetModelState::Request set_state_req;
      gazebo_msgs::SetModelState::Response set_state_resp;
      std_msgs::Float32MultiArray current_blocks_msg;
      current_blocks_msg.data.clear();

      // make sure /gazebo/spawn_urdf_model service is service_ready
      bool service_ready = false;
      while (!service_ready){
        service_ready = ros::service::exists("/gazebo/spawn_urdf_model", true);
        ROS_INFO("waiting for spawn_urdf_model service");
        ros::Duration(0.5).sleep();
      }
      ROS_INFO("spawn_urdf_model service is ready");

      service_ready = false;
      while (!service_ready) {
          service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
          ROS_INFO("waiting for apply_body_wrench service");
          ros::Duration(0.5).sleep();
      }
      ROS_INFO("apply_body_wrench service is ready");

      service_ready = false;
      while (!service_ready) {
          service_ready = ros::service::exists("/gazebo/set_model_state", true);
          ROS_INFO("waiting for set_model_state service");
          ros::Duration(0.5).sleep();
      }
      ROS_INFO("set_model_state service is ready");

      //get file path of blocks from parameter service
      

      if (!(get_red_path)){
          return ;}
          else{ROS_INFO_STREAM(red_box_path << " has been extracted");
}
      pugi::xml_document doc;
      static int i =0;
      pugi::xml_parse_result result = doc.load_file(red_box_path.c_str());
      //std::string str = doc.child("robot").attribute("name").value();
      double x_size=streamOfBoxes[i][0]/100.0, y_size=streamOfBoxes[i][1]/100.0, z_size=streamOfBoxes[i][2]/100.0;
      //double x_size=.1, y_size=.1, z_size=.1;
      std::string x_size_str=std::to_string(x_size), y_size_str=std::to_string(y_size),  z_size_str=std::to_string(z_size);
      std::string pos_temp=x_size_str+" "+y_size_str+" "+z_size_str;
      char * pos=&pos_temp[0];
      doc.select_node("/robot/link/collision/geometry/box").node().attribute("size").set_value(pos);
      doc.select_node("/robot/link/visual/geometry/box").node().attribute("size").set_value(pos);
      /*std::ifstream red_inXml(red_box_path.c_str());*/
      std::stringstream red_strStream;
      std::string red_xmlStr;
      doc.save(red_strStream,"  ");
      /*red_inXml.open(red_box_path.c_str());*/
      //red_strStream << red_inXml.rdbuf();
      red_xmlStr = red_strStream.str();
      //ROS_INFO_STREAM("ashish: "<<red_xmlStr);
     // ROS_INFO_STREAM("urdf: \n" <<red_xmlStr);
      // prepare the pawn model service message
      spawn_model_req.initial_pose.position.x = 0; 
      spawn_model_req.initial_pose.position.y = 0;
      spawn_model_req.initial_pose.position.z = 0.2;
      spawn_model_req.initial_pose.orientation.x=0.0;
      spawn_model_req.initial_pose.orientation.y=0.0;
      spawn_model_req.initial_pose.orientation.z=0.0;
      spawn_model_req.initial_pose.orientation.w=1.0;
      spawn_model_req.reference_frame = "world";

      ros::Time time_temp(0, 0);
      ros::Duration duration_temp(0, 1000000);
      /*apply_wrench_req.wrench.force.x = -100;
      apply_wrench_req.wrench.force.y = 0.0;
      apply_wrench_req.wrench.force.z = 0.0;
      apply_wrench_req.start_time = time_temp;
      apply_wrench_req.duration = duration_temp;
      apply_wrench_req.reference_frame = "world";*/

          std::string index = intToString(i);
          std::string model_name;

         /* spawn_model_req.initial_pose.position.y = (float)rand()/(float)(RAND_MAX) * 0.4;  // random between -0.4 to 0.4
          ROS_INFO_STREAM("y position of new box: "
          << spawn_model_req.initial_pose.position.y);*/

          model_name = "red_blocks_" + index;  // initialize model_name
          spawn_model_req.model_name = model_name;
          //set_state_req.model_state.model_name=model_name;
          spawn_model_req.robot_namespace = model_name;
          spawn_model_req.model_xml = red_xmlStr;

          bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);
          if (call_service) {
              if (spawn_model_resp.success) {
                  ROS_INFO_STREAM(model_name << " has been spawned");
              }
              else {
                  ROS_INFO_STREAM(model_name << " spawn failed");
              }
          }
          else {
              ROS_INFO("fail in first call");
              ROS_ERROR("fail to connect with gazebo server");
              return ;
          }
          ur5_notebook:: blocks_info blocks_info_msg;
          blocks_info_msg.name=model_name;
          blocks_info_msg.x=x_size;
          blocks_info_msg.y=y_size;
          blocks_info_msg.z=z_size;
          block_info_publisher.publish(blocks_info_msg);

          i = i + 1;
          ROS_INFO_STREAM("");
  }

  int main(int argc, char **argv) {
      ros::init(argc, argv, "blocks_spawner");
      ros::NodeHandle nh;

      srand(time(0));
      //service client for service /gazebo/spawn_urdf_model
      block_info_publisher
        = nh.advertise<ur5_notebook::blocks_info>("blocks_info", 1);
      ros::Subscriber current_subscriber = nh.subscribe("block_pickedup"
        , 1, pickedUpCallback);
      spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
      wrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
      setstateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      get_red_path = nh.getParam("/red_box_path", red_box_path);
      generateStreamOfBoxes();
      std::cout<<"streamOfboxes: "<<streamOfBoxes[0][0]<<" "<<streamOfBoxes[0][1]<<" "<<streamOfBoxes[0][2];
      ur5_notebook::Tracker tracker;
      tracker.flag2=1;
      pickedUpCallback(tracker);
      ros::spin();
      
      return 0;
  }
