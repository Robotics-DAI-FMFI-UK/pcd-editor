#include <vector>
#include <glm/glm.hpp>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>


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

private:
	pcl::PointXYZI min_pt, max_pt;
};
