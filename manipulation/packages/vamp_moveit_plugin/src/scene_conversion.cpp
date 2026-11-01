#include "vamp_moveit_plugin/scene_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <octomap/octomap.h>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace vamp_moveit_plugin
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("vamp_moveit_plugin.scene_conversion");

void addSphere(const Eigen::Vector3d& center, double radius, std::vector<geometry_msgs::msg::Point>& centers,
               std::vector<double>& radii)
{
  centers.push_back(tf2::toMsg(center));
  radii.push_back(radius);
}

void addBoundingSphere(const shapes::Shape& shape, const Eigen::Isometry3d& pose,
                       std::vector<geometry_msgs::msg::Point>& centers, std::vector<double>& radii)
{
  Eigen::Vector3d center;
  double radius;
  shapes::computeShapeBoundingSphere(&shape, center, radius);
  addSphere(pose * center, radius, centers, radii);
}

// Spheres along the cylinder axis that stay inside its caps, so a held object never pokes into the table it sits on
void addCylinderSpheres(const shapes::Cylinder& cylinder, const Eigen::Isometry3d& pose,
                        std::vector<geometry_msgs::msg::Point>& centers, std::vector<double>& radii)
{
  const double half_span = std::max(0.0, cylinder.length / 2.0 - cylinder.radius);
  const int count = 1 + static_cast<int>(std::ceil(2.0 * half_span / cylinder.radius));
  for (int i = 0; i < count; ++i)
  {
    const double z = count == 1 ? 0.0 : -half_span + 2.0 * half_span * i / (count - 1);
    addSphere(pose * Eigen::Vector3d(0.0, 0.0, z), cylinder.radius, centers, radii);
  }
}

// Expands pruned octree leaves back to full resolution so every point stands for one voxel of the same size
void addOctree(const octomap::OcTree& octree, const Eigen::Isometry3d& pose, srv::VampPlan::Request& request)
{
  const double resolution = octree.getResolution();
  for (auto it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
  {
    if (!octree.isNodeOccupied(*it))
    {
      continue;
    }
    const double size = it.getSize();
    const int cells = std::max(1, static_cast<int>(std::lround(size / resolution)));
    const Eigen::Vector3d first_cell = Eigen::Vector3d(it.getX(), it.getY(), it.getZ()) -
                                       Eigen::Vector3d::Constant((size - resolution) / 2.0);
    for (int i = 0; i < cells; ++i)
    {
      for (int j = 0; j < cells; ++j)
      {
        for (int k = 0; k < cells; ++k)
        {
          request.point_cloud.push_back(tf2::toMsg(Eigen::Vector3d(pose * (first_cell + resolution * Eigen::Vector3d(i, j, k)))));
        }
      }
    }
  }
  request.point_radius = std::max(request.point_radius, resolution * std::sqrt(3.0) / 2.0);
}

void addWorldShape(const std::string& object_id, const shapes::Shape& shape, const Eigen::Isometry3d& pose,
                   srv::VampPlan::Request& request)
{
  switch (shape.type)
  {
    case shapes::SPHERE:
      addSphere(pose.translation(), static_cast<const shapes::Sphere&>(shape).radius, request.sphere_centers,
                request.sphere_radii);
      break;
    case shapes::BOX:
    {
      const auto& box = static_cast<const shapes::Box&>(shape);
      geometry_msgs::msg::Vector3 size;
      size.x = box.size[0];
      size.y = box.size[1];
      size.z = box.size[2];
      request.box_poses.push_back(tf2::toMsg(pose));
      request.box_sizes.push_back(size);
      break;
    }
    case shapes::CYLINDER:
    {
      const auto& cylinder = static_cast<const shapes::Cylinder&>(shape);
      request.cylinder_poses.push_back(tf2::toMsg(pose));
      request.cylinder_radii.push_back(cylinder.radius);
      request.cylinder_lengths.push_back(cylinder.length);
      break;
    }
    case shapes::CONE:
    case shapes::MESH:
      addBoundingSphere(shape, pose, request.sphere_centers, request.sphere_radii);
      break;
    case shapes::OCTREE:
    {
      const auto& octree = static_cast<const shapes::OcTree&>(shape).octree;
      if (octree)
      {
        addOctree(*octree, pose, request);
      }
      break;
    }
    default:
      RCLCPP_WARN(LOGGER, "Object '%s' has a shape VAMP cannot represent (type %d); it is ignored", object_id.c_str(),
                  static_cast<int>(shape.type));
      break;
  }
}

}  // namespace

void addWorldObstacles(const planning_scene::PlanningScene& scene, srv::VampPlan::Request& request)
{
  const auto& world = *scene.getWorld();
  for (const auto& object_id : world.getObjectIds())
  {
    const auto object = world.getObject(object_id);
    for (std::size_t i = 0; i < object->shapes_.size(); ++i)
    {
      addWorldShape(object_id, *object->shapes_[i], object->global_shape_poses_[i], request);
    }
  }
}

void addAttachedBodies(const moveit::core::RobotState& state, const std::string& end_effector_link,
                       srv::VampPlan::Request& request)
{
  std::vector<const moveit::core::AttachedBody*> bodies;
  state.getAttachedBodies(bodies);
  const Eigen::Isometry3d world_to_end_effector = state.getGlobalLinkTransform(end_effector_link).inverse();

  for (const auto* body : bodies)
  {
    const auto& shapes = body->getShapes();
    const auto& poses = body->getGlobalCollisionBodyTransforms();
    for (std::size_t i = 0; i < shapes.size(); ++i)
    {
      const Eigen::Isometry3d pose = world_to_end_effector * poses[i];
      if (shapes[i]->type == shapes::CYLINDER)
      {
        addCylinderSpheres(static_cast<const shapes::Cylinder&>(*shapes[i]), pose, request.attached_sphere_centers,
                           request.attached_sphere_radii);
      }
      else
      {
        addBoundingSphere(*shapes[i], pose, request.attached_sphere_centers, request.attached_sphere_radii);
      }
    }
  }
}

}  // namespace vamp_moveit_plugin
