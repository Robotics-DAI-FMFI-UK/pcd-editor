#include <vector>
#include <glm/glm.hpp>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>

#include "world.h"

class CloudMap {
public:
	pcl::PointCloud<pcl::PointXYZI> cloud;
	std::vector<size_t> pointIndexSelection;
	float SKIP = 0.95f;
	float TIME_START = 0.0f;
	float TIME_END = 1.0f;
	int DRAW_BOUNDING = 0;

	CloudMap(pcl::PointCloud<pcl::PointXYZI>& cloud);

	void draw();

	void shuffleSkip();

	void transformMap(Eigen::Affine3f t);

	glm::vec3 original(const glm::vec3& p); // point in transformed map -> point in original map
	glm::vec3 transform(const glm::vec3& p); // inverse

	glm::quat original(const glm::quat& o); // rotation in transformed map -> rotation in original map
	glm::quat transform(const glm::quat& o); // inverse


private:
	pcl::PointXYZI min_pt, max_pt;
	Eigen::Affine3f appliedTransforms;
};
