#ifndef _FOOTPRINT_HELPER_H
#define _FOOTPRINT_HELPER_H
#include<geometry_msgs/Polygon.h>
#include<geometry_msgs/Point32.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

double sign(double n){
    if(n>0) return 1.0;
    else if(n<0) return -1.0;
    else return 0.0;
}

/**
* @brief  Grab the footprint of the robot from the parameter server if available
*/
geometry_msgs::Polygon loadRobotFootprint(ros::NodeHandle node){
    geometry_msgs::Polygon footprint;
    geometry_msgs::Point32 pt;
    double padding;

    std::string padding_param, footprint_param;
    if(!node.searchParam("footprint_padding", padding_param))
      padding = 0.01;
    else
      node.param(padding_param, padding, 0.01);

    //grab the footprint from the parameter server if possible
    XmlRpc::XmlRpcValue footprint_list;
    std::string footprint_string;
    std::vector<std::string> footstring_list;
    if(node.searchParam("footprint", footprint_param)){
      node.getParam(footprint_param, footprint_list);
      if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        footprint_string = std::string(footprint_list);

        //if there's just an empty footprint up there, return
        if(footprint_string == "[]" || footprint_string == "")
          return footprint;

        boost::erase_all(footprint_string, " ");

        boost::char_separator<char> sep("[]");
        boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
        footstring_list = std::vector<std::string>(tokens.begin(), tokens.end());
      }
      //make sure we have a list of lists
      if(!(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray && footprint_list.size() > 2) && !(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString && footstring_list.size() > 5)){
        ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s", footprint_param.c_str(), std::string(footprint_list).c_str());
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
      }

      if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int i = 0; i < footprint_list.size(); ++i){
          //make sure we have a list of lists of size 2
          XmlRpc::XmlRpcValue point = footprint_list[i];
          if(!(point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() == 2)){
            ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
          }
       
          //make sure that the value we're looking at is either a double or an int
          if(!(point[0].getType() == XmlRpc::XmlRpcValue::TypeInt || point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
            ROS_FATAL("Values in the footprint specification must be numbers");
            throw std::runtime_error("Values in the footprint specification must be numbers");
          }
          pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);
          pt.x += sign(pt.x) * padding;
       
          //make sure that the value we're looking at is either a double or an int
          if(!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt || point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
            ROS_FATAL("Values in the footprint specification must be numbers");
            throw std::runtime_error("Values in the footprint specification must be numbers");
          }
          pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);
          pt.y += sign(pt.y) * padding;
       
          footprint.points.push_back(pt);

          node.deleteParam(footprint_param);
          std::ostringstream oss;
          bool first = true;
          BOOST_FOREACH(geometry_msgs::Point32 p, footprint.points) {
            if(first) {
              oss << "[[" << p.x << "," << p.y << "]";
              first = false;
            }
            else {
              oss << ",[" << p.x << "," << p.y << "]";
            }
          }
          oss << "]";
          node.setParam(footprint_param, oss.str().c_str());
          node.setParam("footprint", oss.str().c_str());
        }
      }

      else if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        bool valid_foot = true;
        BOOST_FOREACH(std::string t, footstring_list) {
          if( t != "," ) {
            boost::erase_all(t, " ");
            boost::char_separator<char> pt_sep(",");
            boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
            std::vector<std::string> point(pt_tokens.begin(), pt_tokens.end());

            if(point.size() != 2) {
              ROS_WARN("Each point must have exactly 2 coordinates");
              valid_foot = false;
              break;
            }

            std::vector<double>tmp_pt;
            BOOST_FOREACH(std::string p, point) {
              std::istringstream iss(p);
              double temp;
              if(iss >> temp) {
                tmp_pt.push_back(temp);
              }
              else {
                ROS_WARN("Each coordinate must convert to a double.");
                valid_foot = false;
                break;
              }
            }
            if(!valid_foot)
              break;

            geometry_msgs::Point32 pt;
            pt.x = tmp_pt[0];
            pt.y = tmp_pt[1];

            footprint.points.push_back(pt);
          }
        }
        if (!valid_foot) {
          ROS_FATAL("This footprint is not vaid it must be specified as a list of lists with at least 3 points, you specified %s", footprint_string.c_str());
          throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }
        node.setParam("footprint", footprint_string);
      }
    }
    return footprint;
}

#endif
