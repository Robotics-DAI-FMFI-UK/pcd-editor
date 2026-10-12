#include "core.h"
#include "drawing.h"

// =========================================================================
// DRAWING FUNCTIONS
// Should be macro-customizable and include reusable parts of the scene or
// environment, aiming for an unified look accross visualisations and use
// cases.
// =========================================================================

// must be drawn last for the transparency to work properly
void Vis_DrawFloor() {
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //Set blending function.
	glEnable(GL_BLEND);
	glColor4f(0.3f, 0.3f, 0.3f, 0.6f);

	float sz = 1000.0f;

	glBegin(GL_QUADS);
	glVertex3f(-sz, 0.01f, -sz);
	glVertex3f(sz, 0.01f, -sz);
	glVertex3f(sz, 0.01f, sz);
	glVertex3f(-sz, 0.01f, sz);
	glEnd();
}

void Vis_DrawGrid() {
	glLineWidth(VIS_SCENE_GRID_THICKNESS);
	glColor3f(VIS_SCENE_GRID_COLOR);

	float half = VIS_SCENE_GRID_SQUARES * VIS_SCENE_GRID_SPACING;

	glBegin(GL_LINES);
	for (int i = -VIS_SCENE_GRID_SQUARES; i <= VIS_SCENE_GRID_SQUARES; ++i) {
		float p = i * VIS_SCENE_GRID_SPACING;
		glVertex3f(p, 0, -half);
		glVertex3f(p, 0, half);
		glVertex3f(-half, 0, p);
		glVertex3f(half, 0, p);
	}
	glEnd();
}

void Vis_DrawGizmo(glm::vec3 pos) {
	glLineWidth(VIS_SCENE_AXIS_THICKNESS);
	glBegin(GL_LINES);

	glColor3f(VIS_SCENE_AXIS_COLOR_X);
	glVertex3f(pos.x, pos.y, pos.z); glVertex3f(pos.x + 2.0f * VIS_SCENE_AXIS_LENGTH_MULT_X, pos.y, pos.z);
	glColor3f(VIS_SCENE_AXIS_COLOR_Y);
	glVertex3f(pos.x, pos.y, pos.z); glVertex3f(pos.x, pos.y + 2.0f * VIS_SCENE_AXIS_LENGTH_MULT_Y, pos.z);
	glColor3f(VIS_SCENE_AXIS_COLOR_Z);
	glVertex3f(pos.x, pos.y, pos.z); glVertex3f(pos.x, pos.y, pos.z + 2.0f * VIS_SCENE_AXIS_LENGTH_MULT_Z);

	glEnd();
}

void Vis_DrawWireCube(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& anglesDeg) {
	glm::vec3 lo = glm::min(p0, p1);
	glm::vec3 hi = glm::max(p0, p1);
	glm::vec3 center = (lo + hi) * 0.5f;
	glm::vec3 half = (hi - lo) * 0.5f;

	// corners relative to center — rotation is applied about this local origin
	glm::vec3 corners[8] = {
		{ -half.x, -half.y, -half.z },{ half.x, -half.y, -half.z },
		{ half.x,  half.y, -half.z },{ -half.x, half.y, -half.z },
		{ -half.x, -half.y,  half.z },{ half.x, -half.y,  half.z },
		{ half.x,  half.y,  half.z },{ -half.x, half.y,  half.z },
	};

	// 12 edges as pairs of corner indices: bottom face, top face, then verticals connecting them
	static const int edges[12][2] = {
		{ 0,1 },{ 1,2 },{ 2,3 },{ 3,0 },
		{ 4,5 },{ 5,6 },{ 6,7 },{ 7,4 },
		{ 0,4 },{ 1,5 },{ 2,6 },{ 3,7 },
	};

	glPushMatrix();
	glTranslatef(center.x, center.y, center.z);
	glRotatef(anglesDeg.z, 0.0f, 0.0f, 1.0f);
	glRotatef(anglesDeg.y, 0.0f, 1.0f, 0.0f);
	glRotatef(anglesDeg.x, 1.0f, 0.0f, 0.0f);


	glBegin(GL_LINES);
	for (int i = 0; i < 12; i++)
	{
		const glm::vec3& a = corners[edges[i][0]];
		const glm::vec3& b = corners[edges[i][1]];
		glVertex3f(a.x, a.y, a.z);
		glVertex3f(b.x, b.y, b.z);
	}
	glEnd();
	glPopMatrix();
}