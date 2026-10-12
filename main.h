#include "vis/world.h"

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include "nfd.h"


#define VIS_ZONE_Y_SCALE_SPEED             4.0f
#define VIS_ZONE_XZ_SCALE_SPEED            1.0f
#define VIS_ZONE_MAX_SIZE                  1000.0f

template<typename T>
struct ModifiedSelection {
	static_assert(std::is_trivially_copyable_v<T>,
		"ModifiedSelection does a raw memcpy, so T must be trivially copyable");
	
	T mod{};
	T* applyTo = nullptr;

	ModifiedSelection() = default;
	ModifiedSelection(T& item) {
		std::memcpy(&mod, &item, sizeof(T));
		applyTo = &item;
	}

	void apply() const {
		assert(applyTo != nullptr);
		std::memcpy(applyTo, &mod, sizeof(T));
	}

	bool empty() {
		return applyTo == nullptr;
	}

	// selection->prop = val; instead of selection.mod.prop = val;
	T* operator->() { return &mod; }
	const T* operator->() const { return &mod; }
	T& operator*() { return mod; }
	const T& operator*() const { return mod; }
};

struct Vis_Cuboid; // forward declaration
struct Vis_Zone {
	glm::vec3 center;
	glm::vec3 halfExtents;
	Vis_Orientation rot;

	Vis_Zone() : center(0), halfExtents(0), rot(0, 0, 0) {};
	Vis_Zone(glm::vec3 center, glm::vec3 halfExtents) : center(center), halfExtents(halfExtents), rot(0, 0, 0) {};
	Vis_Zone(glm::vec3 center, glm::vec3 halfExtents, Vis_Orientation rot) : center(center), halfExtents(halfExtents), rot(rot) {};

	Vis_Cuboid toCuboid();
	static Vis_Cuboid toCuboid(const Vis_Zone& zone);
};

struct Vis_Cuboid {
	glm::vec3 p0;
	glm::vec3 p1;
	Vis_Orientation rot;

	Vis_Cuboid() : p0(0), p1(0), rot(0, 0, 0) {};
	Vis_Cuboid(glm::vec3 p0, glm::vec3 p1) : p0(p0), p1(p1), rot(0, 0, 0) {};
	Vis_Cuboid(glm::vec3 p0, glm::vec3 p1, Vis_Orientation rot) : p0(p0), p1(p1), rot(rot) {};

	Vis_Zone toZone();
	static Vis_Zone toZone(const Vis_Cuboid& cub);
};

glm::mat4 Vis_Zone_GetTransform(const Vis_Zone& zone);

glm::vec3 Vis_Zone_LocalToWorld(const Vis_Zone& zone, const glm::vec3& local);

glm::vec3 Vis_Zone_WorldToLocal(const Vis_Zone& zone, const glm::vec3& world);

void Vis_DrawZone(const Vis_Zone& zone);

void Vis_Zone_ApplyDrag(Vis_Zone& zone, const glm::vec3& localAttach, const glm::vec3& target);

void Vis_Zone_ApplyXZScale(Vis_Zone& zone, const glm::vec3& localAttach, const glm::vec3& target, const glm::vec3& initialBoxAttach);


enum DrawState {
	UNINIT,
	START_POINT
};

enum SelectionState {
	EMPTY,
	SINGLE,
	SINGLE_DRAG,
	SINGLE_SCALE
	//(todo:) MULTI
};

void controls();

void mainui();

int main();