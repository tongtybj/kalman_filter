// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

#include <cassert>
#include <cmath>
#include <iostream>

/* math */
#include <Eigen/Core>
#include <Eigen/Dense>

/* ros */
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class LowPassFilter
{
public:
  LowPassFilter() {}
  virtual ~LowPassFilter() = default;

  virtual void setInitValues(const Eigen::VectorXd& init_value) = 0;

  /* overwrite function for 1 dimension */
  virtual void setInitValues(const double& init_value)
  {
    Eigen::VectorXd init_vec(1);
    init_vec << init_value;
    setInitValues(init_vec);
  }

  /* overwrite function for 3 dimension: tf::Vector3 */
  virtual void setInitValues(const geometry_msgs::msg::Vector3& init_value)
  {
    Eigen::Vector3d init_vec;
    tf2::fromMsg(init_value, init_vec);
    setInitValues(init_vec);
  }

  /* overwrite function for 3 dimension: geometry_msgs::Point */
  virtual void setInitValues(const geometry_msgs::msg::Point& init_value)
  {
    Eigen::Vector3d init_vec;
    tf2::fromMsg(init_value, init_vec);
    setInitValues(init_vec);
  }

  virtual const Eigen::VectorXd filterFunction(const Eigen::VectorXd& input) = 0;

  /* overwrite function for 1 dimension */
  virtual const double filterFunction(const double& input)
  {
    Eigen::VectorXd in(1);
    in << input;
    return filterFunction(in)(0);
  }

  // /* overwrite function for 3 dimension: geometry_msgs::msg::Vector3*/
  // virtual const geometry_msgs::msg::Vector3 filterFunction(const geometry_msgs::msg::Vector3& input)
  // {
  //   Eigen::Vector3d input_vec;
  //   tf2::fromMsg(input, input_vec);
  //   Eigen::Vector3d out_e = filterFunction(input_vec);
  //   geometry_msgs::msg::Vector3 out;
  //   out.x = out_e.x();
  //   out.y = out_e.y();
  //   out.z = out_e.z();
  //   return out;
  // }

  /* overwrite function for 3 dimension: geometry_msgs::Point */
  virtual const geometry_msgs::msg::Point filterFunction(const geometry_msgs::msg::Point& input)
  {
    Eigen::Vector3d input_vec;
    tf2::fromMsg(input, input_vec);
    Eigen::Vector3d out_e = filterFunction(input_vec);
    geometry_msgs::msg::Point out;
    out.x = out_e.x();
    out.y = out_e.y();
    out.z = out_e.z();
    return out;
  }
};

class IirFilter : public LowPassFilter
{
private:
  double sample_freq_{}, cutoff_freq_{};
  double a_, w0_, a1_, a2_, b0_, b1_, b2_;
  Eigen::VectorXd pre1_, pre2_;

public:
  IirFilter() = default;

  IirFilter(double sample_freq, double cutoff_freq,
            int dimension = 1, bool verbose = false)
  {
    setSampleFreq(sample_freq);
    setCutoffFreq(cutoff_freq);
    setInitParam(verbose);
    pre1_ = Eigen::VectorXd::Zero(dimension);
    pre2_ = Eigen::VectorXd::Zero(dimension);
  }

  inline void setSampleFreq(double f) { sample_freq_ = f; }
  inline void setCutoffFreq(double f) { cutoff_freq_ = f; }

  void setInitParam(bool verbose = false)
  {
    assert(sample_freq_ > cutoff_freq_);
    w0_ = std::tan(M_PI * cutoff_freq_ / sample_freq_);
    a_  = std::sin(w0_) / 0.707;
    a1_ = 2 * std::cos(w0_) / (1 + a_);
    a2_ = (a_ - 1) / (a_ + 1);
    b0_ = (1 - std::cos(w0_)) / 2 / (1 + a_);
    b1_ = (1 - std::cos(w0_)) / (1 + a_);
    b2_ = (1 - std::cos(w0_)) / 2 / (1 + a_);

    if (verbose) {
      std::cout << "IIR Filter: w0: " << w0_
                << ", a1: " << a1_ << ", a2: " << a2_
                << ", b0: " << b0_ << ", b1: " << b1_
                << ", b2: " << b2_ << std::endl;
    }
  }

  void setInitValues(const Eigen::VectorXd& init_value) override
  {
    assert(init_value.size() == pre1_.size());
    pre2_ = init_value / (2 - 2 * std::cos(w0_)) * (a_ + 1);
    pre1_ = pre2_;
  }

  const Eigen::VectorXd filterFunction(const Eigen::VectorXd& input) override
  {
    assert(input.size() == pre1_.size());
    Eigen::VectorXd reg_pos = input + a1_ * pre1_ + a2_ * pre2_;
    Eigen::VectorXd out    = b0_ * reg_pos + b1_ * pre1_ + b2_ * pre2_;
    pre2_ = pre1_;
    pre1_ = reg_pos;
    return out;
  }

  const double filterFunction(const double& input) {return  LowPassFilter::filterFunction(input); }
};

class FirFilter : public LowPassFilter
{
private:
  double filter_factor_{1.0};
  Eigen::VectorXd output_val_;

public:
  FirFilter() = default;

  FirFilter(double factor, int dimension = 1)
  {
    setFilterFactor(factor);
    output_val_ = Eigen::VectorXd::Zero(dimension);
  }

  inline void setFilterFactor(double f) { assert(f <= 1); filter_factor_ = f; }
  inline double getFilterFactor() const    { return filter_factor_; }

  void setInitValues(const Eigen::VectorXd& init_value) override
  {
    assert(init_value.size() == output_val_.size());
    output_val_ = init_value;
  }

  const Eigen::VectorXd filterFunction(const Eigen::VectorXd& input) override
  {
    assert(input.size() == output_val_.size());
    output_val_ += filter_factor_ * (input - output_val_);
    return output_val_;
  }
  const double filterFunction(const double& input) {return  LowPassFilter::filterFunction(input); }
};

class FirFilterQuaternion
{
private:
  double filter_factor_{1.0};
  Eigen::Quaterniond output_val_;

public:
  FirFilterQuaternion() = default;
  explicit FirFilterQuaternion(double factor) { setFilterFactor(factor); }

  inline void setFilterFactor(double f) { assert(f <= 1); filter_factor_ = f; }
  inline double getFilterFactor() const { return filter_factor_; }

  void setInitValues(const Eigen::Quaterniond& init_value)
  {
    output_val_ = init_value;
  }

  void setInitValues(const geometry_msgs::msg::Quaternion& init_value)
  {
    Eigen::Quaterniond q;
    tf2::fromMsg(init_value, q);
    output_val_ = q;
  }

  Eigen::Quaterniond filterFunction(const Eigen::Quaterniond& input)
  {
    output_val_ = output_val_.slerp(filter_factor_, input);
    return output_val_;
  }

  geometry_msgs::msg::Quaternion filterFunction(const geometry_msgs::msg::Quaternion& input)
  {
    Eigen::Quaterniond q;
    tf2::fromMsg(input, q);
    Eigen::Quaterniond out_q = filterFunction(q);
    return tf2::toMsg(out_q);
  }
};

#endif  // DIGITAL_FILTER_H
