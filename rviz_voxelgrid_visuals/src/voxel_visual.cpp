#include "voxel_visual.h"

#include <std_msgs/Float32MultiArray.h>

namespace rviz_voxelgrid_visuals {

VoxelGridVisual::VoxelGridVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;

  frame_node_ = parent_node->createChildSceneNode();

  voxel_grid_points_.reset(new rviz::PointCloud());
  voxel_grid_points_->setRenderMode(rviz::PointCloud::RM_BOXES);
  frame_node_->attachObject(voxel_grid_points_.get());
}

VoxelGridVisual::~VoxelGridVisual() { scene_manager_->destroySceneNode(frame_node_); }

void VoxelGridVisual::reset() { voxel_grid_points_->clear(); }

// Position and orientation are passed through to the SceneNode.
void VoxelGridVisual::setFramePosition(const Ogre::Vector3& position) { frame_node_->setPosition(position); }

void VoxelGridVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void VoxelGridVisual::setColor(float r, float g, float b, float a) {
  r_ = r;
  g_ = g;
  b_ = b;
  a_ = a;
}

void VoxelGridVisual::setBinaryDisplay(bool binary_display) { binary_display_ = binary_display; }

void VoxelGridVisual::setThreshold(float threshold) { threshold_ = threshold; }

void VoxelGridVisual::setHidden(bool hidden) { hidden_ = hidden; }

/*****************************************************************
 * Dense Voxel Grid Visual
 ****************************************************************/
void DenseVoxelGridVisual::setMessage(const rviz_voxelgrid_visuals_msgs::VoxelgridStamped::ConstPtr& msg) {
  latest_msg = *msg;
  updatePointCloud();
}

void DenseVoxelGridVisual::updatePointCloud() {
  if (hidden_) {
    voxel_grid_points_->clear();
    return;
  }

  if (latest_msg.occupancy.layout.dim.empty()) {
    return;
  }

  double scale = latest_msg.scale;
  voxel_grid_points_->setDimensions((float)scale, (float)scale, (float)scale);

  const std::vector<float>& data = latest_msg.occupancy.data;
  const std::vector<std_msgs::MultiArrayDimension>& dims = latest_msg.occupancy.layout.dim;
  int data_offset = latest_msg.occupancy.layout.data_offset;

  std::vector<rviz::PointCloud::Point> points;
  for (int i = 0; i < dims[0].size; i++) {
    for (int j = 0; j < dims[1].size; j++) {
      for (int k = 0; k < dims[2].size; k++) {
        float val = data[data_offset + dims[0].stride * i + dims[1].stride * j + dims[2].stride * k];
        if (val < threshold_) {
          continue;
        }

        rviz::PointCloud::Point p;
        p.position.x = scale / 2 + i * scale + latest_msg.origin.x;
        p.position.y = scale / 2 + j * scale + latest_msg.origin.y;
        p.position.z = scale / 2 + k * scale + latest_msg.origin.z;

        if (binary_display_) {
          val = 1.0;
        }

        if (latest_msg.has_color) {
          p.setColor(latest_msg.color.r, latest_msg.color.g, latest_msg.color.b,
                     std::min(val * latest_msg.color.a * a_, 1.0f));
        } else {
          p.setColor(r_, g_, b_, std::min(val * a_, 1.0f));
        }

        points.push_back(p);
      }
    }
  }

  voxel_grid_points_->clear();

  // The per-point alpha setting is not great with alpha=1, so in
  // certain cases do not use it
  bool use_per_point = !(a_ >= 1.0 && binary_display_);
  voxel_grid_points_->setAlpha(a_, use_per_point);

  voxel_grid_points_->addPoints(&points.front(), points.size());
}

void DenseVoxelGridVisual::reset() {
  latest_msg = rviz_voxelgrid_visuals_msgs::VoxelgridStamped();
  VoxelGridVisual::reset();
}

/*****************************************************************
 * Sparse Voxel Grid Visual
 ****************************************************************/
void SparseVoxelGridVisual::setMessage(const rviz_voxelgrid_visuals_msgs::SparseVoxelgridStamped::ConstPtr& msg) {
  latest_msg = *msg;
  updatePointCloud();
}

void SparseVoxelGridVisual::updatePointCloud() {
  if (hidden_) {
    voxel_grid_points_->clear();
    return;
  }

  double scale = latest_msg.scale;
  voxel_grid_points_->setDimensions((float)scale, (float)scale, (float)scale);

  std::vector<rviz::PointCloud::Point> points;
  for (const auto& voxel : latest_msg.voxels) {
    if (voxel.value < threshold_) {
      continue;
    }

    rviz::PointCloud::Point p;
    p.position.x = scale / 2 + voxel.i * scale + latest_msg.origin.x;
    p.position.y = scale / 2 + voxel.j * scale + latest_msg.origin.y;
    p.position.z = scale / 2 + voxel.k * scale + latest_msg.origin.z;

    float val = binary_display_ ? 1.0f : voxel.value;
    p.setColor(r_, g_, b_, std::min(val * a_, 1.0f));
    points.push_back(p);
  }

  voxel_grid_points_->clear();

  // The per-point alpha setting is not great with alpha=1, so in
  // certain cases do not use it
  bool use_per_point = !(a_ >= 1.0 && binary_display_);
  voxel_grid_points_->setAlpha(a_, use_per_point);

  voxel_grid_points_->addPoints(&points.front(), points.size());
}

void SparseVoxelGridVisual::reset() {
  latest_msg = rviz_voxelgrid_visuals_msgs::SparseVoxelgridStamped();
  VoxelGridVisual::reset();
}

}  // end namespace rviz_voxelgrid_visuals
