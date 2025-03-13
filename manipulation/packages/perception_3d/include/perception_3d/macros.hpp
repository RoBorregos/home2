#ifndef MACROS_HPP
#define MACROS_HPP
#define _IN_
#define _OUT_

enum u32bStatusErrorCodes {
  OK = 0,
  COULD_NOT_CONVERT_POINT_CLOUD = 1,
  COULD_NOT_ESTIMATE_PLANAR_MODEL = 2,
  COULD_NOT_SAVE_POINT_CLOUD = 3,
  NO_POINTCLOUD = 4,
  INVALID_INPUT_FILTER = 5,
  NO_POINT_DETECTED = 6,
  NO_OBJECT_TO_CLUSTER_AT_POINT = 7,
  POINT_CLOUD_EMPTY = 8,
  COULDNT_TRANSFORM_TO_BASE_LINK = 9,
};

#define ASSERT(var, val, message)                                              \
  if (var != val) {                                                            \
    RCLCPP_ERROR(this->get_logger(), message);                                 \
  }

#define ASSERT_AND_RETURN(var, val, message)                                   \
  if (var != val) {                                                            \
    RCLCPP_ERROR(this->get_logger(), message);                                 \
    return;                                                                    \
  }
#define ASSERT_AND_RETURN_CODE(var, val, message, code)                        \
  if (var != val) {                                                            \
    RCLCPP_ERROR(this->get_logger(), message, code);                           \
    return;                                                                    \
  }

#define ASSERT_AND_RETURN_CODE_VALUE(var, val, message, code)                  \
  if (var != val) {                                                            \
    RCLCPP_ERROR(this->get_logger(), message, code);                           \
    return code;                                                               \
  }

#endif