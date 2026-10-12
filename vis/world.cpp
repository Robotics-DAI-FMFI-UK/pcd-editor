#include "core.h"
#include "world.h"

// =========================================================================
// WORLD AND INTERFACE SETUP
// the behavior of the visualiser is defined and expected to be tweaked
// here. functions should have comment descriptions detailing the technical
// aspects and implications as the code can get pretty low level.
// =========================================================================

const glm::vec3 VIS_SCENE_WORLD_UP = glm::vec3(VIS_SCENE_UP_X, VIS_SCENE_UP_Y, VIS_SCENE_UP_Z);
const glm::vec3 VIS_SCENE_WORLD_UP_NORM = glm::normalize(VIS_SCENE_WORLD_UP);
const glm::vec3 VIS_WORLD_X_NORM = glm::normalize(glm::cross(
	(fabsf(VIS_SCENE_WORLD_UP_NORM.x) < 0.99f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1),
	VIS_SCENE_WORLD_UP_NORM));
const glm::vec3 VIS_WORLD_Z_NORM = glm::cross(VIS_WORLD_X_NORM, VIS_SCENE_WORLD_UP_NORM);

Vis_CameraState VIS_CAMERA;

glm::mat4 VIS_CAMERA_VIEW_MATRIX;

// yaw/pitch/roll -> forward/right/up. forward is unaffected by roll; roll
// spins right/up around the forward axis (tilts the horizon).
void Vis_Camera_GetBasis(Vis_Orientation angles, glm::vec3* out_forward, glm::vec3* out_right, glm::vec3* out_up) {
	float yaw = DEG2RAD(angles.yaw);
	float pitch = DEG2RAD(angles.pitch);
	float roll = DEG2RAD(angles.roll);

	glm::vec3 forward_canonical(
		cosf(pitch) * sinf(yaw),
		sinf(pitch),
		-cosf(pitch) * cosf(yaw)
	);

	glm::vec3 forward = glm::normalize(
		VIS_WORLD_X_NORM * forward_canonical.x +
		VIS_SCENE_WORLD_UP_NORM * forward_canonical.y +
		VIS_WORLD_Z_NORM * forward_canonical.z
	);

	glm::vec3 right0 = glm::normalize(glm::cross(forward, VIS_SCENE_WORLD_UP));
	glm::vec3 up0 = glm::normalize(glm::cross(right0, forward));

	glm::vec3 right = cosf(roll) * right0 + sinf(roll) * up0;
	glm::vec3 up = -sinf(roll) * right0 + cosf(roll) * up0;

	if (out_forward != NULL) *out_forward = forward;
	if (out_right != NULL)   *out_right = right;
	if (out_up != NULL)      *out_up = up;
}
void Vis_Camera_GetBasis(glm::vec3* out_forward, glm::vec3* out_right, glm::vec3* out_up) {
	Vis_Camera_GetBasis(VIS_CAMERA.angles, out_forward, out_right, out_up);
}

// Where the eye sits: focus_distance back from pivot, along forward.
glm::vec3 Vis_Camera_GetEye(const glm::vec3& pivot, Vis_Orientation angles, float focus_distance) {
	glm::vec3 forward;
	Vis_Camera_GetBasis(angles, &forward, nullptr, nullptr);
	return pivot - forward * focus_distance;
}

// Inverse: the pivot that would put the eye at `eye`, given an orientation
// and focus distance. Used by free-look, which wants the eye to stay put
// while the orientation changes.
glm::vec3 Vis_Camera_GetPivot(const glm::vec3& eye, Vis_Orientation angles, float focus_distance) {
	glm::vec3 forward;
	Vis_Camera_GetBasis(angles, &forward, nullptr, nullptr);
	return eye + forward * focus_distance;
}

void Vis_Camera_Reset() {
	Vis_CameraState defaults;
	VIS_CAMERA.pivot = defaults.pivot;
	VIS_CAMERA.angles = defaults.angles;
	VIS_CAMERA.pivot_distance = defaults.pivot_distance;
}

bool VIS_CAMERA_CONTROLS = true;

// Reads mouse/scroll input and updates VIS_CAMERA directly -- no
// intermediate target state, so every drag/scroll/reset is applied on the
// same frame it happens.
void Vis_UpdateInput() {
	ImGuiIO& io = ImGui::GetIO();
	if (io.WantCaptureMouse) return;
	if (!VIS_CAMERA_CONTROLS) return;

	if (io.MouseDoubleClicked[ImGuiMouseButton_Left]) {
		Vis_Camera_Reset();
	}

	// ---- dolly: scroll wheel moves the pivot along the viewing axis ----
	if (io.MouseWheel != 0.0f) {
		glm::vec3 forward;
		Vis_Camera_GetBasis(&forward, nullptr, nullptr);

		float step = io.MouseWheel * VIS_CAMERA_DOLLY_SPEED * (log2(VIS_CAMERA.pivot_distance) + 1);
		VIS_CAMERA.pivot += forward * step;
	}

	// ---- free look with the camera ----
	if (ImGui::IsMouseDragging(VIS_CAMERA_LOOK_MOUSE_BUTTON, 0.0f)) {
		ImVec2 delta = ImGui::GetMouseDragDelta(VIS_CAMERA_LOOK_MOUSE_BUTTON);
		ImGui::ResetMouseDragDelta(VIS_CAMERA_LOOK_MOUSE_BUTTON);

		glm::vec3 forward;
		Vis_Camera_GetBasis(&forward, nullptr, nullptr);
		glm::vec3 eye_before = Vis_Camera_GetEye(VIS_CAMERA.pivot, VIS_CAMERA.angles, VIS_CAMERA.pivot_distance);

		VIS_CAMERA.angles.yaw += delta.x * VIS_CAMERA_ORBIT_SPEED_DEG;
		VIS_CAMERA.angles.pitch -= delta.y * VIS_CAMERA_ORBIT_SPEED_DEG;
		VIS_CAMERA.angles.pitch = std::clamp(VIS_CAMERA.angles.pitch, VIS_CAMERA_PITCH_MIN, VIS_CAMERA_PITCH_MAX);

		// free look: pivot location is irrelevant, so just hold the eye
		// exactly fixed as before.
		VIS_CAMERA.pivot = Vis_Camera_GetPivot(eye_before, VIS_CAMERA.angles, VIS_CAMERA.pivot_distance);
	}

	// ---- orbit around ground point ----
	if (ImGui::IsMouseDragging(VIS_CAMERA_ORBIT_MOUSE_BUTTON, 0.0f)) {
		ImVec2 delta = ImGui::GetMouseDragDelta(VIS_CAMERA_ORBIT_MOUSE_BUTTON);
		ImGui::ResetMouseDragDelta(VIS_CAMERA_ORBIT_MOUSE_BUTTON);

		glm::vec3 forward; Vis_Camera_GetBasis(&forward, nullptr, nullptr);
		glm::vec3 eye_before = Vis_Camera_GetEye(VIS_CAMERA.pivot, VIS_CAMERA.angles, VIS_CAMERA.pivot_distance);

		float forward_up = glm::dot(forward, VIS_SCENE_WORLD_UP_NORM);
		if (fabsf(forward_up) > 1e-4f) {
			float eye_up = glm::dot(eye_before, VIS_SCENE_WORLD_UP_NORM);
			float t = -eye_up / forward_up;
			if (t > 0.0f) {
				VIS_CAMERA.pivot = eye_before + forward * t;
				VIS_CAMERA.pivot_distance = t;
			}

			VIS_CAMERA.angles.yaw += delta.x * VIS_CAMERA_ORBIT_SPEED_DEG;
			VIS_CAMERA.angles.pitch -= delta.y * VIS_CAMERA_ORBIT_SPEED_DEG;
			VIS_CAMERA.angles.pitch = std::clamp(VIS_CAMERA.angles.pitch, VIS_CAMERA_PITCH_MIN, VIS_CAMERA_PITCH_MAX);
		}
	}

	// ---- pan: slides the pivot along the screen-space viewing plane ----
	if (ImGui::IsMouseDragging(VIS_CAMERA_PAN_MOUSE_BUTTON, 0.0f)) {
		ImVec2 delta = ImGui::GetMouseDragDelta(VIS_CAMERA_PAN_MOUSE_BUTTON);
		ImGui::ResetMouseDragDelta(VIS_CAMERA_PAN_MOUSE_BUTTON);

		glm::vec3 right, up;
		Vis_Camera_GetBasis(nullptr, &right, &up);

		float pan_scale = VIS_CAMERA_PAN_SPEED * VIS_CAMERA.pivot_distance;
		VIS_CAMERA.pivot += (-delta.x * right + delta.y * up) * pan_scale;
	}
}


// Vis_Camera_ComputeFrustumBounds - single source of truth for
// projection frustum parameters. Both the legacy GL path
// (Vis_Camera_SetupProjection) and the glm path (Vis_Camera_GetProjMatrix,
// used for picking) call this, so they can never disagree.
Vis_FrustumBounds Vis_Camera_ComputeFrustumBounds(int w, int h) {
	if (h <= 0) h = 1;
	float aspect = (float)w / (float)h;

	Vis_FrustumBounds b{};

	if (VIS_CAMERA.projection_mode == VIS_PROJ_ORTHOGRAPHIC) {
		b.top = VIS_CAMERA.pivot_distance * VIS_CAMERA_ORTHO_SCALE;
		b.right = b.top * aspect;
		b.zNear = -VIS_CAMERA_FAR;
		b.zFar = VIS_CAMERA_FAR;
	}
	else {
		float fovRad = VIS_CAMERA_FOV_DEG * 3.14159265f / 180.0f;
		b.top = tanf(fovRad * 0.5f) * VIS_CAMERA_NEAR;
		b.right = b.top * aspect;
		b.zNear = VIS_CAMERA_NEAR;
		b.zFar = VIS_CAMERA_FAR;
	}

	b.left = -b.right;
	b.bottom = -b.top;
	return b;
}

// Your existing function, now just consuming the shared bounds.
void Vis_Camera_SetupProjection(int w, int h) {
	Vis_FrustumBounds b = Vis_Camera_ComputeFrustumBounds(w, h);

	if (VIS_CAMERA.projection_mode == VIS_PROJ_ORTHOGRAPHIC)
		glOrtho(b.left, b.right, b.bottom, b.top, b.zNear, b.zFar);
	else
		glFrustum(b.left, b.right, b.bottom, b.top, b.zNear, b.zFar);
}

// New: identical bounds, expressed as a glm matrix for CPU-side math.
glm::mat4 Vis_Camera_GetProjMatrix(int w, int h) {
	Vis_FrustumBounds b = Vis_Camera_ComputeFrustumBounds(w, h);

	if (VIS_CAMERA.projection_mode == VIS_PROJ_ORTHOGRAPHIC)
		return glm::ortho(b.left, b.right, b.bottom, b.top, b.zNear, b.zFar);
	else
		return glm::frustum(b.left, b.right, b.bottom, b.top, b.zNear, b.zFar);
}

// Sets up viewport + projection, derives `eye` from the smoothed
// pivot/yaw/pitch/roll/focus_distance, and rebuilds VIS_CAMERA_VIEW_MATRIX.
// Also loads GL_MODELVIEW with that matrix as a sane default -- call
// Vis_LoadCoord() afterwards for anything that should be positioned within
// a specific coordinate frame instead. Call once per frame, after
// Vis_UpdateInput().
void Vis_UpdateWorld() {
	int w, h;
	glfwGetFramebufferSize(VIS_GLFW_WINDOW, &w, &h);
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	Vis_Camera_SetupProjection(w, h);

	glm::vec3 forward, up;
	Vis_Camera_GetBasis(&forward, nullptr, &up);
	VIS_CAMERA.eye = VIS_CAMERA.pivot - forward * VIS_CAMERA.pivot_distance;

	VIS_CAMERA_VIEW_MATRIX = glm::lookAt(VIS_CAMERA.eye, VIS_CAMERA.eye + forward, up);

	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(glm::value_ptr(VIS_CAMERA_VIEW_MATRIX));

	glEnable(GL_DEPTH_TEST);
}




// =========================================================================
// COORDINATE FRAMES/SCOPES
// aiming to make trasnformations effortless and scoped, we introduce
// coordinate frames, which each have their own basis vectors and affect
// opengl draw calls
// =========================================================================

// The canonical identity frame -- true OpenGL/world axes X=(1,0,0),
// Y=(0,1,0), Z=(0,0,1), independent of VIS_SCENE_WORLD_UP. The gizmo is
// loaded in this frame so it always shows literal world X/Y/Z.
Vis_Coord VIS_COORD_WORLD;

// Grid/floor are written in local Y-up space (see Vis_DrawGrid/Vis_DrawFloor
// below) and were never rewritten to be up-axis-aware -- instead, this frame
// rotates local Y to point along VIS_SCENE_WORLD_UP before they're drawn, so
// they land on the correct plane for whatever axis the macros call "up".
// Deliberately kept separate from VIS_COORD_WORLD: the origin gizmo must
// keep showing literal world X/Y/Z, not axes twisted to match "up".
Vis_Coord VIS_COORD_GROUND = { glm::vec3(0.0f), VIS_WORLD_X_NORM, VIS_SCENE_WORLD_UP_NORM, VIS_WORLD_Z_NORM };


// Builds a coordinate frame from origin/rotation/scale params.
Vis_Coord Vis_SetCoord(const Vis_CoordParams& params) {
	glm::mat4 rot(1.0f);
	rot = glm::rotate(rot, DEG2RAD(params.rotation_deg.x), glm::vec3(1, 0, 0));
	rot = glm::rotate(rot, DEG2RAD(params.rotation_deg.y), glm::vec3(0, 1, 0));
	rot = glm::rotate(rot, DEG2RAD(params.rotation_deg.z), glm::vec3(0, 0, 1));

	Vis_Coord coord;
	coord.origin = params.origin;
	coord.axis_x = glm::vec3(rot * glm::vec4(1, 0, 0, 0)) * params.scale;
	coord.axis_y = glm::vec3(rot * glm::vec4(0, 1, 0, 0)) * params.scale;
	coord.axis_z = glm::vec3(rot * glm::vec4(0, 0, 1, 0)) * params.scale;
	return coord;
}

// The 4x4 change-of-basis matrix for a coordinate frame (coord space ->
// world space). Use this to compare two coordinate systems, e.g. to find
// the transform that takes you from one to the other.
glm::mat4 Vis_GetCoordBasis(const Vis_Coord& coord) {
	glm::mat4 basis(1.0f);
	basis[0] = glm::vec4(coord.axis_x, 0.0f);
	basis[1] = glm::vec4(coord.axis_y, 0.0f);
	basis[2] = glm::vec4(coord.axis_z, 0.0f);
	basis[3] = glm::vec4(coord.origin, 1.0f);
	return basis;
}

// Resets GL_MODELVIEW to (camera view) * (this coord frame's basis) -- a
// clean slate positioned/oriented at `coord`, independent of whatever
// glTranslatef/glRotatef calls happened for a previously drawn coordinate
// frame. Follow with ordinary glTranslatef/glRotatef to nudge draw calls
// around within that frame, then issue your draw calls.
void Vis_LoadCoord(const Vis_Coord& coord) {
	glm::mat4 model = VIS_CAMERA_VIEW_MATRIX * Vis_GetCoordBasis(coord);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(glm::value_ptr(model));
}

