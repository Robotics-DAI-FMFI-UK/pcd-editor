#pragma once

#include "core.h"
#include "world.h"

// used by Vis_AlignFloorToGRound()
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
// end used


#define VIS_FLOOR_VOXEL_LEAF          0.1f   // meters; downsample leaf used only for plane detection (speed knob)
#define VIS_FLOOR_RANSAC_DIST_THRESH  0.03f   // meters; max distance from plane to count as inlier ("thinness")
#define VIS_FLOOR_RANSAC_MAX_ITERS    250     // RANSAC iterations
#define VIS_FLOOR_TARGET_UP_AXIS      1       // 0 = leave native (Z-up), 1 = remap to Y-up for the scene/camera
#define VIS_FLOOR_DEBUG_PRINT         0       // 1 = print diagnostics to stderr, 0 = silent

bool Vis_FloorToGroundAlignment(const pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Affine3f &outTransform);
void Vis_AlignFloorToGround(pcl::PointCloud<pcl::PointXYZI> &cloud);


#define VIS_RAY_PLANE_EPSILON 1e-6f   // below this, ray is treated as parallel to plane

bool Vis_RayPlaneIntersect(const glm::vec3 &rayOrigin, const glm::vec3 &rayDir, const glm::vec3 &planePoint, const glm::vec3 &planeNormal, glm::vec3 &outIntersection);


#define VIS_SCREEN_RAY_NEAR_Z 0.0f
#define VIS_SCREEN_RAY_FAR_Z  1.0f

void Vis_ScreenPointToRay(float screenX, float screenY, glm::vec3 &outRayOrigin, glm::vec3 &outRayDir);
void Vis_GetMouseRay(glm::vec3 &outRayOrigin, glm::vec3 &outRayDir);
