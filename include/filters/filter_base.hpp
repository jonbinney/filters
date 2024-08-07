/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef FILTERS__FILTER_BASE_HPP_
#define FILTERS__FILTER_BASE_HPP_

#include <limits>
#include <string>
#include <typeinfo>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rclcpp/rclcpp.hpp"

namespace filters
{

namespace impl
{

inline std::string normalize_param_prefix(std::string prefix)
{
  if (!prefix.empty()) {
    if ('.' != prefix.back()) {
      prefix += '.';
    }
  }
  return prefix;
}

}  // namespace impl

/**
 * \brief A Base filter class to provide a standard interface for all filters
 */
template<typename T>
class FilterBase
{
public:
  /**
   * \brief Default constructor used by Filter Factories
   */
  FilterBase()
  : configured_(false) {}

  /**
   * \brief Virtual Destructor
   */
  virtual ~FilterBase() = default;

  /**
   * \brief Configure the filter from the parameter server
   * \param The parameter from which to read the configuration
   * \param node_handle The optional node handle, useful if operating in a different namespace.
   */
  bool configure(
    const std::string & param_prefix,
    const std::string & filter_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logger,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params)
  {
    if (configured_) {
      RCLCPP_WARN(
        node_logger->get_logger(),
        "Filter %s already being reconfigured",
        filter_name_.c_str());
    }
    if (!node_params) {
      throw std::runtime_error("Need a parameters interface to function.");
    }

    configured_ = false;

    filter_name_ = filter_name;
    param_prefix_ = impl::normalize_param_prefix(param_prefix);
    params_interface_ = node_params;
    logging_interface_ = node_logger;

    configured_ = configure();

    pre_set_parameters_callback_handle_ = params_interface_->add_pre_set_parameters_callback(
        std::bind(&FilterBase::internalPreSetParamsCallback, this, std::placeholders::_1));
    on_set_parameters_callback_handle_ = params_interface_->add_on_set_parameters_callback(
        std::bind(&FilterBase::internalOnSetParamsCallback, this, std::placeholders::_1));
    post_set_parameters_callback_handle_ = params_interface_->add_post_set_parameters_callback(
        std::bind(&FilterBase::internalPostSetParamsCallback, this, std::placeholders::_1));

    return configured_;
  }

  /**
   * \brief Update the filter and return the data seperately
   * This is an inefficient way to do this and can be overridden in the derived class
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   */
  virtual bool update(const T & data_in, T & data_out) = 0;

  /**
   * \brief Get the name of the filter as a string
   */
  inline const std::string & getName() {return filter_name_;}

private:
  void internalPreSetParamsCallback(std::vector<rclcpp::Parameter> & params) const
  {
    std::set<std::string> updated_params;
    std::map<std::string, rclcpp::ParameterValue> & params_after_update;
    if(!params_interface_->get_parameters_by_prefix(param_prefix_, params_after_update)) {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Failed to get parameters for prefix %s", name.c_str());
      return;
    }
    for (const auto & param : params) {
      updated_params.insert(param.name);
      params_after_update[param.name] = param.get_parameter_value();
    }
    return preSetParamsCallback(params);
  }

  rcl_interfaces::msg::SetParametersResult internalOnSetParamsCallback(
    const std::vector<rclcpp::Parameter> & params) const
  {
    return onSetParamsCallback(params);
  }

  void internalPostSetParamsCallback(const std::vector<rclcpp::Parameter> & params)
  {
    return postSetParamsCallback(params);
  }

protected:
  /**
   * \brief Pure virtual function for the sub class to configure the filter
   * This function must be implemented in the derived class.
   */
  virtual bool configure() = 0;

/**
 */
  template<typename PT>
  bool declareParam(
    const std::string & name,
    const PT & default_value,
    bool read_only)
  {
    std::string param_name = param_prefix_ + name;

    // Special case: ROS2 doesn't have unsigned int or size_t parameter types, but we support them here
    // for backwards compatibility by converting to and from a signed integer.
    rclcpp::ParameterValue default_param_value;
    if constexpr (std::is_same<PT, unsigned int>::value or std::is_same<PT, size_t>::value) {
      if (default_value > std::numeric_limits<int>::max()) {
        return false;
      }
      default_param_value = rclcpp::ParameterValue(static_cast<int>(default_value));
    } else {
      default_param_value = rclcpp::ParameterValue(default_value);
    }

    rcl_interfaces::msg::ParameterDescriptor param_descriptor;
    param_descriptor.read_only = read_only;
    rclcpp::ParameterValue new_param_value;
    try {
      params_interface_->declare_parameter(
          param_name, default_param_value, param_descriptor);
    } catch (rclcpp::ParameterTypeException & e) {
      RCLCPP_ERROR(
          logging_interface_->get_logger(),
          "Failed to declare parameter %s", name.c_str());
      return false;
    }

    return true;
  }

  template<typename PT>
  bool getParam(const std::string & name, PT & value_out)
  {
    std::string param_name = param_prefix_ + name;

    // For backwards compatibility. At some point auto-creation of the parameter
    // here should be deprecated.
    if (!params_interface_->has_parameter(param_name)) {
      if (!declareParam(param_name, PT(), false)) {
        return false;
      }
    }

    // Special case: ROS2 doesn't have unsigned int or size_t parameter types, but we support them here
    // for backwards compatibility by converting to and from a signed integer.
    if constexpr (std::is_same<PT, unsigned int>::value or std::is_same<PT, size_t>::value) {
      int signed_value_out;
      try {
        signed_value_out = params_interface_->get_parameter(param_name).get_parameter_value().get<int>();
      } catch (rclcpp::exceptions::InvalidParameterTypeException e) {
        RCLCPP_ERROR(
            logging_interface_->get_logger(),
            "Failed to get parameter %s", name.c_str());
        return false;
      }
      if(signed_value_out < 0) {
        return false;
      }
      value_out = static_cast<PT>(signed_value_out);
    } else {
      value_out = params_interface_->get_parameter(param_name).get_parameter_value().get<PT>();
    }

    return true;
  }

  virtual void preSetParamsCallback(
    __attribute__((unused)) const std::set<std::string> updated_params,
    __attribute__((unused)) std::map<std::string, rclcpp::Parameter> & params_after_update) const {}

  virtual rcl_interfaces::msg::SetParametersResult onSetParamsCallback(
    __attribute__((unused)) const std::vector<rclcpp::Parameter> & params) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  virtual void postSetParamsCallback(
    __attribute__((unused)) const std::vector<rclcpp::Parameter> & params) {}

  /// The name of the filter
  std::string filter_name_;
  /// Whether the filter has been configured.
  bool configured_;

  std::string param_prefix_;

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;

  // Handles for the paramereter callbacks that we register with rclcpp.
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
    pre_set_parameters_callback_handle_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_parameters_callback_handle_;
};


template<typename T>
class MultiChannelFilterBase : public FilterBase<T>
{
public:
  MultiChannelFilterBase()
  : number_of_channels_(0)
  {
  }

  /**
   * \brief Virtual Destructor
   */
  virtual ~MultiChannelFilterBase() = default;

  virtual bool configure()
  {
    return true;
  }

  /**
   * \brief Configure the filter from the parameter server
   * \param number_of_channels How many parallel channels the filter will process
   * \param The parameter from which to read the configuration
   * \param node_handle The optional node handle, useful if operating in a different namespace.
   */
  bool configure(
    size_t number_of_channels,
    const std::string & param_prefix,
    const std::string & filter_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logger,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params)
  {
    number_of_channels_ = number_of_channels;

    return FilterBase<T>::configure(param_prefix, filter_name, node_logger, node_params);
  }

  /**
   * \brief Update the filter and return the data seperately
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   * This funciton must be implemented in the derived class.
   */
  virtual bool update(const std::vector<T> & data_in, std::vector<T> & data_out) = 0;

  virtual bool update(const T & /*data_in*/, T & /*data_out*/)
  {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(),
      "THIS IS A MULTI FILTER DON'T CALL SINGLE FORM OF UPDATE");
    return false;
  }

protected:
  /// How many parallel inputs for which the filter is to be configured
  size_t number_of_channels_;
};

}  // namespace filters

#endif  // FILTERS__FILTER_BASE_HPP_
