#pragma once

#include <glm/glm.hpp>

// =========================================================================
// CUSTOMIZATION MACROS
// the unified entry point for customizing every visual and behavioral
// aspect of the visualizer. functions with parameters that don't depend
// on runtime should use macros that originate from here.
// =========================================================================

// ---- origin gizmo ----
#define VIS_SCENE_AXIS_THICKNESS       2.5f
#define VIS_SCENE_AXIS_COLOR_X         1.0f, 0.25f, 0.25f
#define VIS_SCENE_AXIS_COLOR_Y         0.25f, 1.0f, 0.25f
#define VIS_SCENE_AXIS_COLOR_Z         0.3f, 0.55f, 1.0f
#define VIS_SCENE_AXIS_LENGTH_MULT_X   0.75f
#define VIS_SCENE_AXIS_LENGTH_MULT_Y   0.75f
#define VIS_SCENE_AXIS_LENGTH_MULT_Z   0.75f

// ---- ground plane ----
#define VIS_SCENE_GRID_SQUARES         10       // number of squares to display in each direction
#define VIS_SCENE_GRID_SPACING         1.0f     // world units per square
#define VIS_SCENE_GRID_COLOR           0.32f, 0.32f, 0.32f
#define VIS_SCENE_GRID_THICKNESS       1.0f


// =========================================================================
// DRAWING FUNCTIONS
// Should be macro-customizable and include reusable parts of the scene or
// environment, aiming for an unified look accross visualisations and use
// cases.
// =========================================================================

void Vis_DrawFloor();

void Vis_DrawGrid();

void Vis_DrawGizmo(glm::vec3 pos = glm::vec3());

void Vis_DrawWireCube(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& anglesDeg = glm::vec3(0.0f));