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


bool Vis_RayPlaneIntersect(const glm::vec3 &rayOrigin, const glm::vec3 &rayDir, const glm::vec3 &planePoint, const glm::vec3 &planeNormal, glm::vec3 &outIntersection) {
	glm::vec3 n = glm::normalize(planeNormal);
	float denom = glm::dot(n, rayDir);
	if (!denom) return false;

	if (fabsf(denom) < 1e-6f) {
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



#define VIS_ZONE_Y_SCALE_SPEED             4.0f
#define VIS_ZONE_XZ_SCALE_SPEED            1.0f
#define VIS_ZONE_MAX_SIZE                  1000.0f



Vis_CuboidCorners Vis_CuboidCenter::toCorners() const {
	return toCorners(*this);
}

Vis_CuboidCorners Vis_CuboidCenter::toCorners(const Vis_CuboidCenter& zone) {
	Vis_CuboidCorners cub{};
	cub.p0 = zone.center - zone.halfExtents;
	cub.p1 = zone.center + zone.halfExtents;
	cub.rot = zone.rot;
	return cub;
}

Vis_CuboidCenter Vis_CuboidCorners::toCenter() const {
	return toCenter(*this);
}

Vis_CuboidCenter Vis_CuboidCorners::toCenter(const Vis_CuboidCorners& cub) {
	Vis_CuboidCenter zone{};
	glm::vec3 extents = glm::abs(cub.p1 - cub.p0);
	zone.halfExtents = extents * 0.5f;
	zone.center = (cub.p0 + cub.p1) * 0.5f;
	zone.rot = cub.rot;
	return zone;
}

glm::mat4 Vis_Zone_GetTransform(const Vis_CuboidCenter& zone) {
	const glm::mat4 T = glm::translate(glm::mat4(1.0f), zone.center);
	const glm::mat4 R = glm::mat4_cast(glm::normalize(zone.rot));
	return T * R;
}


glm::vec3 Vis_Zone_LocalToWorld(const Vis_CuboidCenter& zone, const glm::vec3& local) {
	return glm::vec3(Vis_Zone_GetTransform(zone) * glm::vec4(local, 1.0f));
}

glm::vec3 Vis_Zone_WorldToLocal(const Vis_CuboidCenter& zone, const glm::vec3& world) {
	return glm::vec3(glm::inverse(Vis_Zone_GetTransform(zone)) * glm::vec4(world, 1.0f));
}

// assuming an attcahment point at localRayAttach, apply parametrized force
// to move localRayAttach close to target
void Vis_Zone_ApplyDrag(Vis_CuboidCenter& zone, const glm::vec3& localAttach, const glm::vec3& target) {
	const glm::vec3 worldAttach = Vis_Zone_LocalToWorld(zone, localAttach);
	const glm::vec3 oldCenter = zone.center;
	const glm::vec3 up = VIS_SCENE_WORLD_UP_NORM;

	// spin when grabbed outside the footprint, move when grabbed inside
	const bool spin = fabsf(localAttach.x) > fabsf(zone.halfExtents.x) ||
		fabsf(localAttach.z) > fabsf(zone.halfExtents.z);

	if (spin) {
		const glm::vec3 toAttach = worldAttach - oldCenter;
		const glm::vec3 toTarget = target - oldCenter;

		// flatten both vectors onto the plane perpendicular to up
		const glm::vec3 a = toAttach - up * glm::dot(toAttach, up);
		const glm::vec3 b = toTarget - up * glm::dot(toTarget, up);

		// signed angle from a to b about up (same as the old atan2 on x/z)
		const float angleError = atan2f(glm::dot(up, glm::cross(a, b)), glm::dot(a, b));

		// world-space spin about up, pivoting on the center
		zone.rot = glm::normalize(glm::angleAxis(angleError, up) * zone.rot);
	}
	else {
		zone.center += target - worldAttach;
	}
}

// assuming an attcahment point at localRayAttach, apply parametrized scaling
// to zone - local X or Z dimension(whichever is more prominent, only one of them)
void Vis_Zone_ApplyXZScale(Vis_CuboidCenter& zone, const glm::vec3& localRayAttach, const glm::vec3& target, const glm::vec3& initialRayAttach) {
	glm::vec3 worldAttach = Vis_Zone_LocalToWorld(zone, localRayAttach);

	glm::vec3 toAttachLocal = Vis_Zone_WorldToLocal(zone, worldAttach - zone.center);
	glm::vec3 toTargetLocal = Vis_Zone_WorldToLocal(zone, target - zone.center);
	glm::vec3 delta = toTargetLocal - toAttachLocal;

	// flip so "outward" (away from center, on whichever face was grabbed) always grows the zone
	delta.x *= (initialRayAttach.x < 0.0f) ? -1.0f : 1.0f;
	delta.z *= (initialRayAttach.z < 0.0f) ? -1.0f : 1.0f;

	// scale only in the prominent direction
	float xAxisToggle = (float)(abs(delta.x) > abs(delta.z));
	float zAxisToggle = 1.0f - xAxisToggle;

	zone.halfExtents.x += delta.x * logf(zone.halfExtents.x + 1.01f) * xAxisToggle * VIS_ZONE_XZ_SCALE_SPEED;
	zone.halfExtents.z += delta.z * logf(zone.halfExtents.z + 1.01f) * zAxisToggle * VIS_ZONE_XZ_SCALE_SPEED;

	zone.halfExtents.x = std::clamp(zone.halfExtents.x, 0.0f, VIS_ZONE_MAX_SIZE);
	zone.halfExtents.z = std::clamp(zone.halfExtents.z, 0.0f, VIS_ZONE_MAX_SIZE);
}


// ---- Vis_RayCuboidIntersect configuration ----------------------------------
// 1: if the ray origin is inside the cuboid, report a hit at the origin (t = 0).
// 0: report a miss when the origin is inside (only entry hits count).
#ifndef VIS_RAYCUBOID_HIT_FROM_INSIDE
#define VIS_RAYCUBOID_HIT_FROM_INSIDE 1
#endif
// ----------------------------------------------------------------------------

bool Vis_RayCuboidIntersect(const glm::vec3& rayOrigin, const glm::vec3& rayDir,
	const Vis_CuboidCenter& cuboid, glm::vec3& outIntersection) {
	const glm::mat4 model = Vis_Zone_GetTransform(cuboid);
	if (glm::abs(glm::determinant(model)) < 1e-12f) return false;
	const glm::mat4 inv = glm::inverse(model);

	// Local-space bounds
	const glm::vec3 hi = cuboid.halfExtents;
	const glm::vec3 lo = -hi;

	// Ray in local space. Direction is NOT normalized, so t is the same in both spaces.
	const glm::vec3 o = glm::vec3(inv * glm::vec4(rayOrigin, 1.0f));
	const glm::vec3 d = glm::vec3(inv * glm::vec4(rayDir, 0.0f));

	float tNear = -INFINITY;
	float tFar = INFINITY;

	for (int axis = 0; axis < 3; ++axis) {
		if (glm::abs(d[axis]) < 1e-8f) {
			// Parallel to this slab: must already lie between its planes.
			if (o[axis] < lo[axis] || o[axis] > hi[axis]) return false;
			continue;
		}
		float t0 = (lo[axis] - o[axis]) / d[axis];
		float t1 = (hi[axis] - o[axis]) / d[axis];
		if (t0 > t1) { const float tmp = t0; t0 = t1; t1 = tmp; }
		if (t0 > tNear) tNear = t0;
		if (t1 < tFar)  tFar = t1;
		if (tNear > tFar) return false;
	}

	if (tFar < 0.0f) return false; // cuboid entirely behind the ray

	float tHit = tNear;
	if (tNear < 0.0f) {
#if VIS_RAYCUBOID_HIT_FROM_INSIDE
		tHit = 0.0f;
#else
		return false;
#endif
	}

	outIntersection = rayOrigin + rayDir * tHit;
	return true;
}