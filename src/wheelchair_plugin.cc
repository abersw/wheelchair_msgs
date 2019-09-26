#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "wheelchair_msgs/WheelVels.h"

namespace gazebo
{
	/// \brief A plugin to control a Velodyne sensor.
	class VelodynePlugin : public ModelPlugin
	{
		/// \brief Construct8or
		public: VelodynePlugin() {}

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Safety check
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}
	
			// Store the model pointer for convenience.
			this->model = _model;
	
			// Get the first joint. We are making an assumption about the model
			// having one joint that is the rotational joint.
			this->jLeftWheel = _model->GetJoints()[0];
			this->jRightWheel = _model->GetJoints()[1];
	
			// Setup a P-controller, with a gain of 0.1.
			this->pidLeft = common::PID(0.1, 0, 0);
			this->pidRight = common::PID(0.1, 0, 0);
	
			// Apply the P-controller to the joint.
			this->model->GetJointController()->SetVelocityPID(
					this->jLeftWheel->GetScopedName(), this->pidLeft);
			this->model->GetJointController()->SetVelocityPID(
					this->jRightWheel->GetScopedName(), this->pidRight);
	
			// Default to zero velocity
			double velocity = 0;
	
			// Check that the velocity element exists, then read the value
			//if (_sdf->HasElement("velocity"))
			//	velocity = _sdf->Get<double>("velocity");
	
			this->SetVelocity(velocity,velocity);
	
			//**** Initialise ROS Node and callback function ****//
	
			int argc = 0;
			char **argv = NULL;
			// Create our ROS node.
			ros::init(argc, argv, "gazebo_client");
			ros::NodeHandle nh;
			ROS_INFO("Created ROS node");
	
			rosSub = nh.subscribe("wheelchair_description/vel_cmd", 1, &VelodynePlugin::OnRosMsg, this);
			ROS_INFO("Subscribed to topic wheelchair_description/vel_cmd");
	
	
			// Can't use ros::spin() here, so spin up the queue helper thread instead.
			this->rosQueueThread =
					std::thread(std::bind(&VelodynePlugin::QueueThread, this));
	
		}
	
		/// \brief Set the velocity of the Velodyne
		/// \param[in] _vel New target velocity
		public: void SetVelocity(const double &_velLeft, const double &_velRight)
		{
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(
					this->jLeftWheel->GetScopedName(), _velLeft);
			this->model->GetJointController()->SetVelocityTarget(
					this->jRightWheel->GetScopedName(), _velRight);
		}


		/// \brief ROS call back function
		/// \param[in] _msg A float value that is used to set the velocity
		/// of the Velodyne.
		public: void OnRosMsg(const wheelchair_msgs::WheelVels &_msg)
		{
			ROS_INFO("Recieved message");
			this->SetVelocity(_msg.leftVel, _msg.rightVel);
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
			ros::Rate rate(10);
			while(ros::ok())
			{
				ros::spinOnce();
				rate.sleep();
			}
		}

		/// \brief Pointer to the model.
		private: physics::ModelPtr model;

		/// \brief Pointer to the joint.
		private: physics::JointPtr jLeftWheel;
		private: physics::JointPtr jRightWheel;

		/// \brief A PID controller for the joint.
		private: common::PID pidLeft;
		private: common::PID pidRight;



		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;

		/// \brief A thread that keeps checking the rosQueue for new messages
		private: std::thread rosQueueThread;
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif