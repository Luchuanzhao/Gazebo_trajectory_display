/************************************************************************
* @file draw_traj_plugin.cc
* @brief Draw trajectory of the robot in Gazebo
* Modefied from Unitree Robotics
*
* @author Luchuanzhao
* @version 1.0.0
* @date 2023.6.26
************************************************************************/

/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/rendering/rendering.hh>

namespace gazebo
{
    class UnitreeDrawForcePlugin : public VisualPlugin
    {
        public:
        UnitreeDrawForcePlugin():line(NULL){}
        ~UnitreeDrawForcePlugin(){
            this->visual->DeleteDynamicLine(this->line);
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
        {
            // scene = rendering::get_scene();
            this->visual = _parent; //parent就是xacro文件中的gazebo reference
            // this->visual = scene->GetVisual();
            this->visual_namespace = "visual/";
            if (!_sdf->HasElement("topicName")){ //这里订阅的是关节的位置真值话题
                ROS_INFO("Force draw plugin missing <topicName>, defaults to /default_force_draw");
                this->topic_name = "/default_force_draw";
            } else{
                this->topic_name = _sdf->Get<std::string>("topicName");
            }
            if (!ros::isInitialized()){
                int argc = 0;
                char** argv = NULL;
                ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
            }

            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
#if GAZEBO_MAJOR_VERSION >= 10
            // https://github.com/osrf/gazebo/blob/gazebo11/Migration.md#gazebo-8x-to-9x
            // gazebo 9 deprecations removed on gazebo 10
            // https://github.com/osrf/gazebo/commit/0bd72b9500e7377c873e32d25b8db772e782bd6f
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), ignition::math::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), ignition::math::Color(0, 1, 0, 1.0));
#else
            // this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), common::Color(0, 1, 0, 1.0));
            // this->line->AddPoint(ignition::math::Vector3d(16.5, 16.5, 0), common::Color(1, 0, 0, 1.0));
            // this->line->AddPoint(ignition::math::Vector3d(-16.5, 16.5, 0), common::Color(1, 0, 0, 1.0));
#endif
            this->line->setMaterial("Gazebo/Purple");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            this->rosnode = new ros::NodeHandle(this->visual_namespace);
            // this->force_sub = this->rosnode->subscribe(this->topic_name+"/"+"the_force", 30, &UnitreeDrawForcePlugin::GetForceCallback, this);
            this->force_sub = this->rosnode->subscribe(this->topic_name, 30, &UnitreeDrawForcePlugin::GetForceCallback, this);
            this->update_connection = event::Events::ConnectPreRender(boost::bind(&UnitreeDrawForcePlugin::OnUpdate, this));
            ROS_INFO("Load %s Draw Force plugin.", this->topic_name.c_str());
        }

        void OnUpdate()
        {
            // this->line->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
            this->line->AddPoint(ignition::math::Vector3d(Fx, Fy, Fz), common::Color(0, 0, 1, 1.0));
            // ROS_INFO("Fx= %d, Fy= %d, Fz= %d", Fx, Fy, Fz);
        }

        void GetForceCallback(const nav_msgs::Odometry & msg)
        {
            Fx = msg.pose.pose.position.x * 17; //坐标不准，需要微调
            Fy = msg.pose.pose.position.y * 17;
            Fz = msg.pose.pose.position.z * 20;
            // ROS_INFO("Fx= %f, Fy= %f, Fz= %f", Fx, Fy, Fz);
            // std::cout << Fx << " " << Fy << ' ' << Fz << std::endl;
            // Fx = msg.wrench.force.x;
            // Fy = msg.wrench.force.y;
            // Fz = msg.wrench.force.z;
        }

        private:
            ros::NodeHandle* rosnode;
            std::string topic_name;
            rendering::VisualPtr visual;
            rendering::DynamicLines *line;
            std::string visual_namespace;
            ros::Subscriber force_sub;
            double Fx = 0, Fy = 0, Fz = 0;
            event::ConnectionPtr update_connection;
    };
    GZ_REGISTER_VISUAL_PLUGIN(UnitreeDrawForcePlugin)
}
