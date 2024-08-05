/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FILTERS__PARAM_TEST_HPP_
#define FILTERS__PARAM_TEST_HPP_

#include "filters/filter_base.hpp"

namespace filters
{

/**
 * \brief A mean filter which works on doubles.
 */
template<typename T>
class ParamTest : public FilterBase<T>
{
public:
  /**
   * \brief Construct the filter with the expected width and height
   */
  ParamTest();

  /**
   * \brief Destructor to clean up
   */
  ~ParamTest() override;

  bool configure() override;

  /**
   * \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  bool update(const T & data_in, T & data_out) override;

  void preSetParamsCallback(std::vector<rclcpp::Parameter> & params) const override;

  rcl_interfaces::msg::SetParametersResult onSetParamsCallback(
    const std::vector<rclcpp::Parameter> & params) const override;

  void postSetParamsCallback(const std::vector<rclcpp::Parameter> & params) override;
};

template<typename T>
ParamTest<T>::ParamTest()
{
}

template<typename T>
ParamTest<T>::~ParamTest()
{
}

template<typename T>
bool ParamTest<T>::configure()
{
  // We'll use this parameter as the output value, which we can check
  // for in testing.
  if (!this->declareParam("output_value", T(), false)) {
    return false;
  }

  // Declare some more parameters which we use to test writing parameters
  // and parameter setting callbacks.
  // "a" is a writeable int.
  if (!this->declareParam("a", 7, true)) {
    return false;
  }

  // "b" must be greater than "a".
  if (!this->declareParam("b", 8, true)) {
    return false;
  }

  // "c" is automatically set to a string representation of "a"
  if (!this->declareParam("c", 8, true)) {
    return false;
  }

  return true;
}

template<typename T>
bool ParamTest<T>::update(const T & /*data_in*/, T & data_out)
{
  return this->getParam("output_value", data_out);
}

template<typename T>
void ParamTest<T>::preSetParamsCallback(std::vector<rclcpp::Parameter> & params) const
{
}

template<typename T>
rcl_interfaces::msg::SetParametersResult ParamTest<T>::onSetParamsCallback(
  const std::vector<rclcpp::Parameter> & params) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.set__successful(true);
  return result;
}

template<typename T>
void ParamTest<T>::postSetParamsCallback(const std::vector<rclcpp::Parameter> & params)
{

}

}  // namespace filters

#endif  // FILTERS__PARAM_TEST_HPP_
