#include "maps.h"

#include "core.h"
#include "world.h"
#include "drawing.h"

#include <time.h>
#include <cmath>
#include <algorithm>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

// generates a random float from 0.0f (inclusive) to 1.0f (exclusive)
float randf() {
	return random() / 2147483648.0f; // 2^32
}

// easing curve
float easeOutQuint(float x) {
	return 1.0f - pow(1.0f - x, 5.0f);
}

template<typename T> void shuffle(std::vector<T> &vec) {
	for (size_t i = 0; i < vec.size(); i++) {
		size_t rand_i = floorf(randf() * vec.size());
		std::swap(vec[i], vec[rand_i]);
	}
}



CloudMap::CloudMap(pcl::PointCloud<pcl::PointXYZI> &cloud) {
	this->cloud = cloud;
	auto sz = this->cloud.size();
	appliedTransforms = Eigen::Affine3f::Identity();

	min_pt = pcl::PointXYZI();
	max_pt = pcl::PointXYZI();
	pcl::getMinMax3D(this->cloud, min_pt, max_pt);

	pointIndexSelection = std::vector<size_t>(sz, -1);

	for (size_t i = 0; i < sz; i++)
		pointIndexSelection[i] = i;
	
	shuffle(pointIndexSelection);
}

void CloudMap::shuffleSkip() {
	shuffle(this->pointIndexSelection);
}

void CloudMap::draw() {
	if (DRAW_BOUNDING) {
		glColor3f(1.0f, 0.0f, 0.0f);
		glLineWidth(2.0f);
		Vis_DrawWireCube(XYZ(min_pt), XYZ(max_pt));
	}

	glEnable(GL_POINT_SMOOTH);
	
	auto sz = this->cloud.size();
	for (int i = 0; i < sz && ((float)(sz - i) / (float)sz) > this->SKIP; i++) {
		size_t idx = pointIndexSelection.at(i);
		float time = (float)(idx) / (float)(sz);
		if (time > TIME_END || time < TIME_START)
			continue;

		pcl::PointXYZI& p = this->cloud.at(idx);
		float intf = ((float)p.intensity) / 256.0f;

		float cam_dist = glm::distance(VIS_CAMERA.eye, XYZ(p));
		glPointSize(std::max(1.0f, 20.0f / cam_dist));
		glBegin(GL_POINTS);
		glColor3f(0.0f, std::max(0.3f, easeOutQuint(intf)), 0.0f);
		glVertex3f(p.x, p.y, p.z);
		glEnd();
	}
}

void CloudMap::transformMap(Eigen::Affine3f t) {
	appliedTransforms = t * appliedTransforms;
	transformPointCloud(cloud, cloud, t);
}

// TODO: unify
glm::vec3 CloudMap::original(const glm::vec3& p) {
	Eigen::Vector3f eigen_p(p.x, p.y, p.z);
	Eigen::Vector3f result = appliedTransforms.inverse() * eigen_p;
	return glm::vec3(result.x(), result.y(), result.z());
}

// TODO: unify
glm::vec3 CloudMap::transform(const glm::vec3& p) {
	Eigen::Vector3f eigen_p(p.x, p.y, p.z);
	Eigen::Vector3f result = appliedTransforms * eigen_p;
	return glm::vec3(result.x(), result.y(), result.z());
}

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>

// Eigen rotation -> glm::quat (Eigen::Quaternionf stores x,y,z,w; glm::quat ctor takes w,x,y,z)
static glm::quat Vis_QuatFromEigen(const Eigen::Matrix3f& RE) {
	const Eigen::Quaternionf e(RE);
	return glm::normalize(glm::quat(e.w(), e.x(), e.y(), e.z()));
}

glm::quat CloudMap::original(const glm::quat& q) {
	const glm::quat Ri = Vis_QuatFromEigen(appliedTransforms.inverse().rotation());
	return glm::normalize(Ri * q);
}

glm::quat CloudMap::transform(const glm::quat& q) {
	const glm::quat R = Vis_QuatFromEigen(appliedTransforms.rotation());
	return glm::normalize(R * q);
}