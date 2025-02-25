#ifndef MACROS_HPP
#define MACROS_HPP
#define _IN_
#define _OUT_

// u32b status error codes
// #define OK 0
// #define COULD_NOT_CONVERT_POINT_CLOUD 1

enum u32bStatusErrorCodes {
  OK = 0,
  COULD_NOT_CONVERT_POINT_CLOUD = 1,
  COULD_NOT_ESTIMATE_PLANAR_MODEL = 2,
  COULD_NOT_SAVE_POINT_CLOUD = 3,
  NO_POINTCLOUD = 4,
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

#endif