#include "algo.h"

// Detects the floor plane in `cloud` via RANSAC. On success, writes the
// affine transform that rotates it level and translates its centroid to
// the origin into `outTransform` and returns true. Does NOT touch `cloud`
// -- pure computation. On failure (empty cloud, or RANSAC finds zero
// inliers), sets `outTransform` to identity and returns false.
bool Vis_FloorToGroundAlignment(const pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Affine3f &outTransform) {
	constexpr float VIS_RAD2DEG = 57.29578f;

	outTransform = Eigen::Affine3f::Identity();

#if VIS_FLOOR_DEBUG_PRINT
	std::cerr << "Vis_FloorToGroundAlignment: input cloud has " << cloud.size() << " points" << std::endl;
#endif
	if (cloud.empty()) return false;

	pcl::PointCloud<pcl::PointXYZI>::Ptr working(new pcl::PointCloud<pcl::PointXYZI>(cloud));
	pcl::VoxelGrid<pcl::PointXYZI> voxel;
	voxel.setInputCloud(working);
	voxel.setLeafSize(VIS_FLOOR_VOXEL_LEAF, VIS_FLOOR_VOXEL_LEAF, VIS_FLOOR_VOXEL_LEAF);
	pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
	voxel.filter(*downsampled);
	if (downsampled->size() < 10) downsampled = working;

#if VIS_FLOOR_DEBUG_PRINT
	std::cerr << "Vis_FloorToGroundAlignment: downsampled to " << downsampled->size() << " points for detection" << std::endl;
#endif

	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(VIS_FLOOR_RANSAC_DIST_THRESH);
	seg.setMaxIterations(VIS_FLOOR_RANSAC_MAX_ITERS);
	seg.setInputCloud(downsampled);

	pcl::ModelCoefficients coeffs;
	pcl::PointIndices inliers;
	seg.segment(inliers, coeffs);

#if VIS_FLOOR_DEBUG_PRINT
	std::cerr << "Vis_FloorToGroundAlignment: RANSAC found " << inliers.indices.size() << " inliers" << std::endl;
#endif
	if (inliers.indices.empty()) {
		std::cerr << "Vis_FloorToGroundAlignment: no planar surface found" << std::endl;
		return false;
	}

	Eigen::Vector3f normal(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
	normal.normalize();

	// Centroid of the detected floor plane -- this is the point that gets
	// translated to the origin below, so the floor passes through (0,0,0)
	// after the transform, not just "somewhere flat."
	Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
	for (int idx : inliers.indices) {
		const auto &p = (*downsampled)[idx];
		centroid += Eigen::Vector3f(p.x, p.y, p.z);
	}
	centroid /= static_cast<float>(inliers.indices.size());

#if VIS_FLOOR_DEBUG_PRINT
	std::cerr << "Vis_FloorToGroundAlignment: raw plane normal = (" << normal.x() << ", " << normal.y() << ", " << normal.z()
		<< "), centroid = (" << centroid.x() << ", " << centroid.y() << ", " << centroid.z() << ")" << std::endl;
#endif

	int above = 0;
	for (const auto &p : downsampled->points) {
		if (normal.dot(Eigen::Vector3f(p.x, p.y, p.z) - centroid) > 0.0f) ++above;
	}
	bool flipped = above < static_cast<int>(downsampled->size()) / 2;
	if (flipped) normal = -normal;

#if VIS_FLOOR_DEBUG_PRINT
	std::cerr << "Vis_FloorToGroundAlignment: " << above << "/" << downsampled->size() << " points above plane"
		<< (flipped ? " (normal flipped to point up)" : "") << std::endl;
#endif

	Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(normal, Eigen::Vector3f(0, 0, 1));

#if VIS_FLOOR_TARGET_UP_AXIS == 1
	// Fixed axis remap, independent of the detected floor: Z-up -> Y-up.
	// Flip the sign of the angle if this comes out mirrored for your
	// engine's handedness.
	Eigen::Quaternionf zToY(Eigen::AngleAxisf(-static_cast<float>(M_PI) / 2.0f, Eigen::Vector3f(1, 0, 0)));
	rotation = zToY * rotation;
#endif

#if VIS_FLOOR_DEBUG_PRINT
	float angleDeg = Eigen::AngleAxisf(rotation).angle() * VIS_RAD2DEG;
	std::cerr << "Vis_FloorToGroundAlignment: total rotation angle = " << angleDeg << " degrees" << std::endl;
#endif

	// Rotate about the floor centroid, then translate that centroid to the
	// origin: final = R*(p - centroid) = R*p - R*centroid. One combined
	// affine transform -- no scaling, no points added/removed/reordered.
	outTransform.linear() = rotation.toRotationMatrix();
	outTransform.translation() = -(rotation * centroid);

	return true;
}

// Original behavior, unchanged from the caller's point of view: detects
// the floor plane in `cloud` and applies the leveling transform in place.
// Leaves `cloud` untouched if no plane is found.
void Vis_AlignFloorToGround(pcl::PointCloud<pcl::PointXYZI> &cloud) {
#if VIS_FLOOR_DEBUG_PRINT
	bool haveSample = !cloud.empty();
	pcl::PointXYZI beforeSample;
	if (haveSample) beforeSample = cloud.points[0];
#endif

	Eigen::Affine3f transform;
	if (!Vis_FloorToGroundAlignment(cloud, transform)) return; // reason, if any, already printed above

	transformPointCloud(cloud, cloud, transform);

#if VIS_FLOOR_DEBUG_PRINT
	if (haveSample) {
		const pcl::PointXYZI &afterSample = cloud.points[0];
		std::cerr << "Vis_AlignFloorToGround: sample point[0] before = (" << beforeSample.x << ", " << beforeSample.y << ", " << beforeSample.z
			<< "), after = (" << afterSample.x << ", " << afterSample.y << ", " << afterSample.z << ")" << std::endl;
	}
#endif
}


// ============================================================
// Vis_RayPlaneIntersect - configurable constants
// ============================================================
#define VIS_RAY_PLANE_EPSILON 1e-6f   // below this, ray is treated as parallel to plane

bool Vis_RayPlaneIntersect(const glm::vec3 &rayOrigin, const glm::vec3 &rayDir, const glm::vec3 &planePoint, const glm::vec3 &planeNormal, glm::vec3 &outIntersection) {
	glm::vec3 n = glm::normalize(planeNormal);
	float denom = glm::dot(n, rayDir);
	if (!denom) return false;

	if (fabsf(denom) < VIS_RAY_PLANE_EPSILON) {
		// ray direction is parallel (or nearly parallel) to the plane
		return false;
	}

	float t = glm::dot(planePoint - rayOrigin, n) / denom;

	if (t < 0.0f) {
		// plane is behind the ray origin
		return false;
	}

	outIntersection = rayOrigin + t * rayDir;
	return true;
}

// ============================================================
// Vis_ScreenPointToRay / Vis_GetMouseRay - configurable constants
// ============================================================
#define VIS_SCREEN_RAY_NEAR_Z 0.0f
#define VIS_SCREEN_RAY_FAR_Z  1.0f

void Vis_ScreenPointToRay(float screenX, float screenY, glm::vec3 &outRayOrigin, glm::vec3 &outRayDir) {
	int w, h;
	glfwGetFramebufferSize(VIS_GLFW_WINDOW, &w, &h);

	float winX = screenX;
	float winY = (float)h - screenY;   // flip to GL's Y-up window space

	glm::vec4 viewport(0.0f, 0.0f, (float)w, (float)h);
	glm::vec3 up; Vis_Camera_GetBasis(NULL, NULL, &up);
	glm::mat4 view = glm::lookAt(VIS_CAMERA.eye, VIS_CAMERA.pivot, up);
	glm::mat4 proj = Vis_Camera_GetProjMatrix(w, h);

	glm::vec3 nearPoint = glm::unProject(glm::vec3(winX, winY, VIS_SCREEN_RAY_NEAR_Z), view, proj, viewport);
	glm::vec3 farPoint = glm::unProject(glm::vec3(winX, winY, VIS_SCREEN_RAY_FAR_Z), view, proj, viewport);

	outRayOrigin = nearPoint;
	outRayDir = glm::normalize(farPoint - nearPoint);
}

void Vis_GetMouseRay(glm::vec3 &outRayOrigin, glm::vec3 &outRayDir) {
	double mx, my;
	glfwGetCursorPos(VIS_GLFW_WINDOW, &mx, &my);
	Vis_ScreenPointToRay((float)mx, (float)my, outRayOrigin, outRayDir);
}
