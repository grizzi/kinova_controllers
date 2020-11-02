/*!
 * @file     ik_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */

#include <pluginlib/class_list_macros.h>
#include "kinova_ik_controller/ik_controller.h"

PLUGINLIB_EXPORT_CLASS(kinova_controllers::IKControllerKinova, controller_interface::ControllerBase)
