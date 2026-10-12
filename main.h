#include "vis/world.h"
#include "vis/algo.h"

#include <ranges>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include "nfd.h"

#define UUID_TIME_GENERATOR
#include "external/stduuid/include/uuid.h"


#define SIZE(x) (sizeof(x)/sizeof((x)[0]))

#define VIS_ZONE_Y_SCALE_SPEED             4.0f
#define VIS_ZONE_XZ_SCALE_SPEED            1.0f
#define VIS_ZONE_MAX_SIZE                  1000.0f


// filtered because with the way the logic is set up, selected zones remain in the `zones` container
// but are delegated to be handled separately (shouldn't be considered when iterating).
// all loop functions are encouraged to use this instead of iterating the `zones` vector directly
// to prevent unwanted behavior with ""invisible"" zones
#define ZONES views::filter(zones, [&](Vis_Zone &z) {for (auto s : selected) { if (s.applyTo == &z) return 0; } return 1;})


template<typename T>
struct ModifiedSelection {
	static_assert(std::is_trivially_copyable_v<T>,
		"ModifiedSelection does a raw memcpy, so T must be trivially copyable");
	
	T mod;
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

	bool empty() const {
		return applyTo == nullptr;
	}

	// selection->prop = val; instead of selection.mod.prop = val;
	T* operator->() { return &mod; }
	const T* operator->() const { return &mod; }
	T& operator*() { return mod; }
	const T& operator*() const { return mod; }
};

struct Vis_BoundTransformable {
	glm::vec3 getCenterPos() const;
	void setCenterPos(glm::vec3 pos);

	glm::vec3 getSize() const;
	void setSize(glm::vec3 size);

	Vis_Orientation getRotation() const;
	void setRotation(Vis_Orientation rot);
};


void Vis_UUID(char* out);

struct Vis_Zone : public Vis_CuboidCenter, public Vis_BoundTransformable {
	Vis_Zone() : color(0.75f, 0.0f, 0.0f) {
		label[0] = 0;
		Vis_UUID(id);
	};
	Vis_Zone(Vis_CuboidCenter o) : Vis_CuboidCenter(o), color(0.75f, 0.0f, 0.0f) {
		label[0] = 0;
		Vis_UUID(id);
	};

	char label[50];
	char id[37];
	glm::vec3 color;
};

void Vis_DrawZone(const Vis_CuboidCenter& zone);


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