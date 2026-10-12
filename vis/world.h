#pragma once

#include "core.h"

#include <vector>
#include <algorithm>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "imgui.h"


// =========================================================================
// CUSTOMIZATION MACROS
// the unified entry point for customizing every visual and behavioral
// aspect of the visualizer. functions with parameters that don't depend
// on runtime should use macros that originate from here.
// =========================================================================

// ---- scene basis ----
// the world "up" axis. orbit yaw rotates around this, and pan's vertical
// direction is derived from it. change this if your data is Z-up.
#define VIS_SCENE_UP_X                 0.0f
#define VIS_SCENE_UP_Y                 1.0f
#define VIS_SCENE_UP_Z                 0.0f

// ---- camera projection ----
#define VIS_CAMERA_FOV_DEG             60.0f
#define VIS_CAMERA_NEAR                0.05f
#define VIS_CAMERA_FAR                 1000.0f
#define VIS_CAMERA_ORTHO_SCALE         0.5f    // world units per pixel-equivalent, scaled by focus distance
#define VIS_CAMERA_PIVOT_DIST          10.0f

// ---- shared pitch limits (applies to both orbit and self-rotate) ----
#define VIS_CAMERA_PITCH_MIN          -89.0f    // stay off the poles - right/up become degenerate at +/-90
#define VIS_CAMERA_PITCH_MAX           89.0f

// ---- orbit controls: mouse drag rotates the camera around a fixed point in front of it ----
#define VIS_CAMERA_ORBIT_MOUSE_BUTTON  ImGuiMouseButton_Right
#define VIS_CAMERA_ORBIT_SPEED_DEG     0.35f    // degrees rotated per pixel dragged

// ---- self-rotate controls: mouse drag + modifier rotates the camera in place ----
#define VIS_CAMERA_LOOK_MOUSE_BUTTON   ImGuiMouseButton_Left
#define VIS_CAMERA_LOOK_SPEED_DEG      0.35f    // separate from orbit speed so it can be tuned independently

// ---- pan controls: mouse drag slides the camera along its own viewing plane ----
#define VIS_CAMERA_PAN_MOUSE_BUTTON    ImGuiMouseButton_Middle
#define VIS_CAMERA_PAN_SPEED           0.002f  // world units per pixel dragged, per world unit of focus distance

// ---- dolly controls: scroll wheel moves the camera along its viewing axis ----
#define VIS_CAMERA_DOLLY_SPEED         0.3f     // distance change per scroll tick
#define VIS_CAMERA_DOLLY_DIST_MIN      1.0f
#define VIS_CAMERA_DOLLY_DIST_MAX      500.0f






// =========================================================================
// WORLD AND INTERFACE SETUP
// the behavior of the visualiser is defined and expected to be tweaked
// here. functions should have comment descriptions detailing the technical
// aspects and implications as the code can get pretty low level.
// =========================================================================

extern const glm::vec3 VIS_SCENE_WORLD_UP;
extern const glm::vec3 VIS_SCENE_WORLD_UP_NORM;
extern const glm::vec3 VIS_WORLD_X_NORM;
extern const glm::vec3 VIS_WORLD_Z_NORM;

enum Vis_ProjectionMode {
	VIS_PROJ_PERSPECTIVE = 0,
	VIS_PROJ_ORTHOGRAPHIC = 1
};

union Vis_Orientation {
	struct { float pitch, yaw, roll; };
	glm::vec3 vec3;

	Vis_Orientation() : vec3(0.0f) {}
	Vis_Orientation(float pitch, float yaw, float roll) : vec3(pitch, yaw, roll) {}
	Vis_Orientation(const glm::vec3& v) : vec3(v) {}

	Vis_Orientation operator+(const Vis_Orientation& o) const { return Vis_Orientation(vec3 + o.vec3); }
	Vis_Orientation operator-(const Vis_Orientation& o) const { return Vis_Orientation(vec3 - o.vec3); }
	Vis_Orientation operator*(float s) const { return Vis_Orientation(vec3 * s); }
	Vis_Orientation operator/(float s) const { return Vis_Orientation(vec3 / s); }

	Vis_Orientation& operator+=(const Vis_Orientation& o) { vec3 += o.vec3; return *this; }
	Vis_Orientation& operator-=(const Vis_Orientation& o) { vec3 -= o.vec3; return *this; }
};

// The camera's real state is pivot + yaw/pitch/roll + focus_distance --
// where it's looking, not where it is. `eye` is derived fresh every frame
// in Vis_UpdateWorld() as `pivot - forward*focus_distance`.
struct Vis_CameraState {
	glm::vec3 pivot = glm::vec3(0.0f);
	Vis_Orientation angles = { -35.0f, -25.0f, 0.0f };
	float pivot_distance = VIS_CAMERA_PIVOT_DIST;

	glm::vec3 eye = glm::vec3(0.0f); // derived, read-only -- see Vis_UpdateWorld

	int projection_mode = VIS_PROJ_PERSPECTIVE;
};

extern Vis_CameraState VIS_CAMERA;

extern glm::mat4 VIS_CAMERA_VIEW_MATRIX;

void Vis_Camera_GetBasis(Vis_Orientation angles, glm::vec3* out_forward = NULL, glm::vec3* out_right = NULL, glm::vec3* out_up = NULL);
void Vis_Camera_GetBasis(glm::vec3* out_forward = NULL, glm::vec3* out_right = NULL, glm::vec3* out_up = NULL);

glm::vec3 Vis_Camera_GetEye(const glm::vec3& pivot, Vis_Orientation angles, float focus_distance);

glm::vec3 Vis_Camera_GetPivot(const glm::vec3& eye, Vis_Orientation angles, float focus_distance);

void Vis_Camera_Reset();

extern bool VIS_CAMERA_CONTROLS;

void Vis_UpdateInput();

struct Vis_FrustumBounds {
	float left, right, bottom, top;
	float zNear, zFar;
};

Vis_FrustumBounds Vis_Camera_ComputeFrustumBounds(int w, int h);

void Vis_Camera_SetupProjection(int w, int h);

glm::mat4 Vis_Camera_GetProjMatrix(int w, int h);

void Vis_UpdateWorld();


// =========================================================================
// COORDINATE FRAMES/SCOPES
// aiming to make trasnformations effortless and scoped, we introduce
// coordinate frames, which each have their own basis vectors and affect
// opengl draw calls
// =========================================================================

// A named coordinate frame: an origin plus three basis vectors (not
// required to be orthonormal, if you ever want sheared/scaled grids). This
// is the single representation of "where a coordinate system is and how
// it's oriented" -- nothing else keeps a second copy.
struct Vis_Coord {
	glm::vec3 origin = glm::vec3(0.0f);
	glm::vec3 axis_x = glm::vec3(1.0f, 0.0f, 0.0f);
	glm::vec3 axis_y = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 axis_z = glm::vec3(0.0f, 0.0f, 1.0f);
};

struct Vis_CoordParams {
	glm::vec3 origin = glm::vec3(0.0f);
	glm::vec3 rotation_deg = glm::vec3(0.0f); // euler angles, applied X then Y then Z, degrees
	float scale = 1.0f;
};

extern Vis_Coord VIS_COORD_WORLD;

extern Vis_Coord VIS_COORD_GROUND;

Vis_Coord Vis_SetCoord(const Vis_CoordParams& params);

glm::mat4 Vis_GetCoordBasis(const Vis_Coord& coord);

void Vis_LoadCoord(const Vis_Coord& coord);
