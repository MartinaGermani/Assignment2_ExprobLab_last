/*!
 * \file marker_server.cpp
 * \brief Service callback function for the room_information service.
 * \author Martina Germani
 */

#include <ros/ros.h>
#include <patrol_robot/RoomConnection.h>
#include <patrol_robot/RoomInformation.h>

/*!
 * \param req Service request message. Contains the ID of the marker.
 * \param res Service response message. Contains the room name, x and y coordinates, and a list of connections for the room associated with the marker ID in the request message.
 *
 * \return True if the service call was successful.
 */

bool markerCallback(patrol_robot::RoomInformation::Request &req, patrol_robot::RoomInformation::Response &res){

	//! Create a RoomConnection message
	patrol_robot::RoomConnection conn;

	//! Check the marker ID and fill out the response message with the corresponding room information
	switch (req.id){
	case 11:
		res.room = "E";
		res.x = 1.5;
		res.y = 8.0;
		conn.connected_to = "C1";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		break;
	case 12: 
		res.room = "C1";
		res.x = -1.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		conn.connected_to = "R2";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 13: 
		res.room = "C2";
		res.x = 3.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		conn.connected_to = "C1";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R3";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		conn.connected_to = "R4";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	case 14: 
		res.room = "R1";
		res.x = -7.0;
		res.y = 3.0;
		conn.connected_to = "C1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		break;
	case 15: 
		res.room = "R2";
		res.x = -7.0;
		res.y = -4.0;
		conn.connected_to = "C1";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 16: 
		res.room = "R3";
		res.x = 9.0;
		res.y = 3.0;
		conn.connected_to = "C2";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		break;
	case 17: 
		res.room = "R4";
		res.x = 9.0;
		res.y = -4.0;
		conn.connected_to = "C2";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	default:
		res.room = "no room associated with this marker id";
	}
	return true;
}	





int main(int argc, char **argv)
{
	//! Initialize ROS and the node
	ros::init(argc, argv, "assignment2");
	ros::NodeHandle nh;

  	//! Advertise the room_info service
	ros::ServiceServer oracle = nh.advertiseService( "/room_info",markerCallback);
	ros::spin();
	ros::shutdown();
	return 0;
}
