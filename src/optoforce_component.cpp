/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the Warsaw University of Technology nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "rtt_rosclock/rtt_rosclock.h"

#include <string>
#include <math.h>
#include "optoforce_can/OptoforceSensor.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "Eigen/Dense"

using RTT::InputPort;
using RTT::OutputPort;

class OptoforceComponent : public RTT::TaskContext {
 private:

  OptoforceSensor *os_;

  // port variables
  geometry_msgs::Vector3Stamped force0_out_;
  geometry_msgs::Vector3Stamped force1_out_;
  geometry_msgs::Vector3Stamped force2_out_;
  geometry_msgs::Vector3Stamped force0_scaled_out_;
  geometry_msgs::Vector3Stamped force1_scaled_out_;
  geometry_msgs::Vector3Stamped force2_scaled_out_;
  Eigen::Vector4d total_force_out_;
  std_msgs::Empty tare_in_;

  // OROCOS ports
  OutputPort<geometry_msgs::Vector3Stamped> port_force0_out_;
  OutputPort<geometry_msgs::Vector3Stamped> port_force1_out_;
  OutputPort<geometry_msgs::Vector3Stamped> port_force2_out_;
  OutputPort<geometry_msgs::Vector3Stamped> port_force0_scaled_out_;
  OutputPort<geometry_msgs::Vector3Stamped> port_force1_scaled_out_;
  OutputPort<geometry_msgs::Vector3Stamped> port_force2_scaled_out_;
  OutputPort<Eigen::Vector4d> port_total_force_out_;

  InputPort<std_msgs::Empty> port_tare_in_;

  // ROS parameters
  std::string dev_name_;
  std::string prefix_;
  int n_sensors_;
  int can_rx_id_;
  int can_tx_id_;

  OptoforceSensor::Speed speed_;
  OptoforceSensor::Filter filter_;
  double nominal_capacity_fx_;
  double nominal_capacity_fy_;
  double nominal_capacity_fz_;
  double counts_nc_0_x_;
  double counts_nc_0_y_;
  double counts_nc_0_z_;
  double counts_nc_1_x_;
  double counts_nc_1_y_;
  double counts_nc_1_z_;
  double counts_nc_2_x_;
  double counts_nc_2_y_;
  double counts_nc_2_z_;
  bool test_mode_;

 public:
  explicit OptoforceComponent(const std::string& name)
      : TaskContext(name, PreOperational),
        os_(NULL),
        n_sensors_(0),
        can_rx_id_(0x100),
        can_tx_id_(0x101),
        test_mode_(0),
        speed_(OptoforceSensor::Speed333),
        filter_(OptoforceSensor::Filter150) {
    this->ports()->addPort("force0_out", port_force0_out_);
    this->ports()->addPort("force1_out", port_force1_out_);
    this->ports()->addPort("force2_out", port_force2_out_);
    this->ports()->addPort("force0_scaled_out", port_force0_scaled_out_);
    this->ports()->addPort("force1_scaled_out", port_force1_scaled_out_);
    this->ports()->addPort("force2_scaled_out", port_force2_scaled_out_);
    this->ports()->addPort("total_measured_force_out", port_total_force_out_);
    this->ports()->addPort("tare_in", port_tare_in_);
    this->addProperty("device_name", dev_name_);
    this->addProperty("prefix", prefix_);
    this->addProperty("n_sensors", n_sensors_);
    this->addProperty("can_rx_id", can_rx_id_);
    this->addProperty("can_tx_id", can_tx_id_);
    this->addProperty("test_mode", test_mode_);
    this->addProperty("nominal_capacity_x", nominal_capacity_fx_);
    this->addProperty("nominal_capacity_y", nominal_capacity_fy_);
    this->addProperty("nominal_capacity_z", nominal_capacity_fz_);
    this->addProperty("counts_nc_0_x", counts_nc_0_x_);
    this->addProperty("counts_nc_0_y", counts_nc_0_y_);
    this->addProperty("counts_nc_0_z", counts_nc_0_z_);
    this->addProperty("counts_nc_1_x", counts_nc_1_x_);
    this->addProperty("counts_nc_1_y", counts_nc_1_y_);
    this->addProperty("counts_nc_1_z", counts_nc_1_z_);
    this->addProperty("counts_nc_2_x", counts_nc_2_x_);
    this->addProperty("counts_nc_2_y", counts_nc_2_y_);
    this->addProperty("counts_nc_2_z", counts_nc_2_z_);
  }

  ~OptoforceComponent() {
  }

  void cleanupHook() {
    if (os_ != NULL) {
      delete os_;
      os_ = NULL;
    }
  }

  // RTT configure hook
  bool configureHook() {
    std::cout << "OptoforceComponent::configureHook, parameters: dev_name="
        << dev_name_ << "  prefix=" << prefix_ << "  n_sensors=" << n_sensors_
        << std::endl;
    if (!test_mode_) {
      if (!dev_name_.empty() && !prefix_.empty() && n_sensors_ > 0
          && os_ == NULL) {

        if (n_sensors_ == 3) {
          os_ = new OptoforceSensor(dev_name_, OptoforceSensor::SensorType4Ch,
                                    can_rx_id_, can_tx_id_);

          port_force0_out_.setDataSample(force0_out_);
          port_force1_out_.setDataSample(force1_out_);
          port_force2_out_.setDataSample(force2_out_);
          port_force0_scaled_out_.setDataSample(force0_scaled_out_);
          port_force1_scaled_out_.setDataSample(force1_scaled_out_);
          port_force2_scaled_out_.setDataSample(force2_scaled_out_);

          total_force_out_.setZero();

          if (os_->isDevOpened()) {
            os_->setConfiguration(speed_, filter_, OptoforceSensor::ZeroSet);
            std::cout << "OptoforceComponent::configureHook success"
                << std::endl;
            return true;
          }
        } else if (n_sensors_ == 1) {
          os_ = new OptoforceSensor(dev_name_, OptoforceSensor::SensorType1Ch,
                                    can_rx_id_, can_tx_id_);

          port_force0_out_.setDataSample(force0_out_);

          total_force_out_.setZero();

          if (os_->isDevOpened()) {
            os_->setConfiguration(speed_, filter_, OptoforceSensor::ZeroSet);
            std::cout << "OptoforceComponent::configureHook success"
                << std::endl;
            return true;
          }
        }
      }

      std::cout << "OptoforceComponent::configureHook failure" << std::endl;
      return false;

    } else {
      std::cout << "OptoforceComponent test mode::configureHook success"
          << std::endl;
      return true;
    }
  }

  // RTT start hook
  bool startHook() {
    return true;
  }

  void stopHook() {
  }

  Eigen::Vector3d scaleForce(const Eigen::Vector3d &force, double counts_nc_x,
                             double counts_nc_y, double counts_nc_z,
                             double nc_x, double nc_y, double nc_z) {
    return Eigen::Vector3d(force(0) / counts_nc_x * nc_x,
                           force(1) / counts_nc_y * nc_y,
                           force(2) / counts_nc_z * nc_z);
  }

  // RTT update hook
  void updateHook() {
    if (!test_mode_) {
      if (port_tare_in_.read(tare_in_) == RTT::NewData) {
        os_->setConfiguration(speed_, filter_, OptoforceSensor::ZeroRestore);
        os_->setConfiguration(speed_, filter_, OptoforceSensor::ZeroSet);
      }
    }

    if (n_sensors_ == 3) {
      Eigen::Vector3d f1, f2, f3;
      if (!test_mode_) {
        if (os_->read(f1, f2, f3)) {
          Eigen::Vector3d f1_scaled = scaleForce(f1, counts_nc_0_x_,
                                                 counts_nc_0_y_, counts_nc_0_z_,
                                                 nominal_capacity_fx_,
                                                 nominal_capacity_fy_,
                                                 nominal_capacity_fz_);
          Eigen::Vector3d f2_scaled = scaleForce(f2, counts_nc_1_x_,
                                                 counts_nc_1_y_, counts_nc_1_z_,
                                                 nominal_capacity_fx_,
                                                 nominal_capacity_fy_,
                                                 nominal_capacity_fz_);
          Eigen::Vector3d f3_scaled = scaleForce(f3, counts_nc_2_x_,
                                                 counts_nc_2_y_, counts_nc_2_z_,
                                                 nominal_capacity_fx_,
                                                 nominal_capacity_fy_,
                                                 nominal_capacity_fz_);

          force0_out_.header.stamp = ros::Time::now();
          force0_out_.vector.x = f1(0);
          force0_out_.vector.y = f1(1);
          force0_out_.vector.z = f1(2);
          force0_scaled_out_.header.stamp = ros::Time::now();
          force0_scaled_out_.vector.x = f1_scaled(0);
          force0_scaled_out_.vector.y = f1_scaled(1);
          force0_scaled_out_.vector.z = f1_scaled(2);

          force1_out_.header.stamp = ros::Time::now();
          force1_out_.vector.x = f2(0);
          force1_out_.vector.y = f2(1);
          force1_out_.vector.z = f2(2);
          force1_scaled_out_.header.stamp = ros::Time::now();
          force1_scaled_out_.vector.x = f2_scaled(0);
          force1_scaled_out_.vector.y = f2_scaled(1);
          force1_scaled_out_.vector.z = f2_scaled(2);

          force2_out_.header.stamp = ros::Time::now();
          force2_out_.vector.x = f3(0);
          force2_out_.vector.y = f3(1);
          force2_out_.vector.z = f3(2);
          force2_scaled_out_.header.stamp = ros::Time::now();
          force2_scaled_out_.vector.x = f3_scaled(0);
          force2_scaled_out_.vector.y = f3_scaled(1);
          force2_scaled_out_.vector.z = f3_scaled(2);

          port_force0_out_.write(force0_out_);
          port_force1_out_.write(force1_out_);
          port_force2_out_.write(force2_out_);
          port_force0_scaled_out_.write(force0_scaled_out_);
          port_force1_scaled_out_.write(force1_scaled_out_);
          port_force2_scaled_out_.write(force2_scaled_out_);

          total_force_out_(0) = f1_scaled.norm();
          total_force_out_(1) = f2_scaled.norm();
          total_force_out_(2) = f3_scaled.norm();
          port_total_force_out_.write(total_force_out_);
        }
      } else {
        port_force0_out_.write(force0_out_);
        port_force1_out_.write(force1_out_);
        port_force2_out_.write(force2_out_);
        port_force0_scaled_out_.write(force0_scaled_out_);
        port_force1_scaled_out_.write(force1_scaled_out_);
        port_force2_scaled_out_.write(force2_scaled_out_);

        port_total_force_out_.write(total_force_out_);
      }
    } else if (n_sensors_ == 1) {
      Eigen::Vector3d f;
      if (!test_mode_) {
        if (os_->read(f)) {
          Eigen::Vector3d f_scaled = scaleForce(f, counts_nc_0_x_,
                                                counts_nc_0_y_, counts_nc_0_z_,
                                                nominal_capacity_fx_,
                                                nominal_capacity_fy_,
                                                nominal_capacity_fz_);

          force0_out_.header.stamp = ros::Time::now();
          force0_out_.vector.x = f(0);
          force0_out_.vector.y = f(1);
          force0_out_.vector.z = f(2);
          force0_scaled_out_.vector.x = f_scaled(0);
          force0_scaled_out_.vector.y = f_scaled(1);
          force0_scaled_out_.vector.z = f_scaled(2);

          port_force0_out_.write(force0_out_);
          port_force0_scaled_out_.write(force0_scaled_out_);

          total_force_out_(0) = f_scaled.norm();
          port_total_force_out_.write(total_force_out_);
        }
      } else {
        port_force0_out_.write(force0_out_);
        port_force0_scaled_out_.write(force0_scaled_out_);
        port_total_force_out_.write(total_force_out_);
      }
    }
  }
};
ORO_CREATE_COMPONENT(OptoforceComponent)

