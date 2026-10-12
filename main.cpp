#include "main.h"

// native libs
#include <cmath>
#include <cstdio>
#include <time.h>
#include <iostream>

#include "vis/shaders.h"
#include "vis/world.h"
#include "vis/drawing.h"
#include "vis/maps.h"
#include "vis/algo.h"

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include "nfd.h"


using namespace std;
using namespace pcl;

// =========================================================================
// UTILITY FUNCTIONS
//
// =========================================================================

#define VIS_ZONE_Y_SCALE_SPEED             4.0f
#define VIS_ZONE_XZ_SCALE_SPEED            1.0f
#define VIS_ZONE_MAX_SIZE                  1000.0f
#define VIS_PLACE_POSE_Z                    0.0f



Vis_Cuboid Vis_Zone::toCuboid() {
   return toCuboid(*this);
}

Vis_Cuboid Vis_Zone::toCuboid(const Vis_Zone& zone) {
   Vis_Cuboid cub{};
   cub.p0 = zone.center - zone.halfExtents;
   cub.p1 = zone.center + zone.halfExtents;
   cub.rot = zone.rot;
   return cub;
}

Vis_Zone Vis_Cuboid::toZone() {
   return toZone(*this);
}

Vis_Zone Vis_Cuboid::toZone(const Vis_Cuboid& cub) {
   Vis_Zone zone{};
   glm::vec3 extents = glm::abs(cub.p1 - cub.p0);
   zone.halfExtents = extents * 0.5f;
   zone.center = (cub.p0 + cub.p1) * 0.5f;
   zone.rot = cub.rot;
   return zone;
}

glm::mat4 Vis_Zone_GetTransform(const Vis_Zone& zone) {
   glm::mat4 m = glm::translate(glm::mat4(1.0f), zone.center);
   m = glm::rotate(m, zone.rot.yaw, VIS_SCENE_WORLD_UP_NORM);
   return m;
}

glm::vec3 Vis_Zone_LocalToWorld(const Vis_Zone& zone, const glm::vec3& local) {
   return glm::vec3(Vis_Zone_GetTransform(zone) * glm::vec4(local, 1.0f));
}

glm::vec3 Vis_Zone_WorldToLocal(const Vis_Zone& zone, const glm::vec3& world) {
   return glm::vec3(glm::inverse(Vis_Zone_GetTransform(zone)) * glm::vec4(world, 1.0f));
}

void Vis_DrawZone(const Vis_Zone& zone) {
   glm::vec3 p0 = zone.center - zone.halfExtents;
   glm::vec3 p1 = zone.center + zone.halfExtents;
   Vis_DrawWireCube(p0, p1, zone.rot.vec3);
}

// assuming an attcahment point at localRayAttach, apply parametrized force
// to move localRayAttach close to target
void Vis_Zone_ApplyDrag(Vis_Zone& zone, const glm::vec3& localAttach, const glm::vec3& target) {
   glm::vec3 worldAttach = Vis_Zone_LocalToWorld(zone, localAttach);
   glm::vec3 oldCenter = zone.center;

   // spin when grabbed on the outside of the zone, move when grabbed on the inside
   float spinToggle = (float)(abs(localAttach.x) > abs(zone.halfExtents.x) || abs(localAttach.z) > abs(zone.halfExtents.z));
   float moveToggle = 1.0f - spinToggle;

   glm::vec3 toAttach = worldAttach - oldCenter;
   glm::vec3 toTarget = target - oldCenter;

   float angleError = atan2f(toAttach.z * toTarget.x - toAttach.x * toTarget.z,
      toAttach.x * toTarget.x + toAttach.z * toTarget.z);

   zone.rot.yaw += angleError * spinToggle;
   zone.center += (target - worldAttach) * moveToggle;
   zone.center.y = zone.halfExtents.y;
}

// assuming an attcahment point at localRayAttach, apply parametrized scaling
// to zone - local X or Z dimension(whichever is more prominent, only one of them)
void Vis_Zone_ApplyXZScale(Vis_Zone& zone, const glm::vec3& localRayAttach, const glm::vec3& target, const glm::vec3& initialRayAttach) {
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

   zone.halfExtents.x = clamp(zone.halfExtents.x, 0.0f, VIS_ZONE_MAX_SIZE);
   zone.halfExtents.z = clamp(zone.halfExtents.z, 0.0f, VIS_ZONE_MAX_SIZE);
}

// =========================================================================
// SCENE LOGIC
// Logic controlling the scene and the objects inside of it.
// =========================================================================

vector<Vis_Zone> zones;
SelectionState selectionState = EMPTY;
ModifiedSelection<Vis_Zone> selection;
DrawState drawState = UNINIT;

Vis_Cuboid drawing_cuboid;            // (TODO: remove?) used as temporary container for new zone
glm::vec3 initialRayAttach;
glm::vec3 rayAttach;
bool showGroundCoordinates = false;
Eigen::Affine3f worldToPcd = Eigen::Affine3f::Identity();

enum CoordinatePickState {
   COORD_LIVE,
   COORD_DRAGGING,
   COORD_LOCKED
};

CoordinatePickState coordinatePickState = COORD_LIVE;
glm::vec3 coordinateStartWorld(0.0f);
glm::vec3 coordinateEndWorld(0.0f);
Eigen::Vector3f coordinateStartMap = Eigen::Vector3f::Zero();
Eigen::Vector3f coordinateEndMap = Eigen::Vector3f::Zero();
Eigen::Vector3f coordinateCursorMap = Eigen::Vector3f::Zero();
bool coordinateCursorValid = false;
bool coordinateOrientationValid = false;
float coordinateYaw = 0.0f;
float coordinateQx = 1.0f;
float coordinateQy = 0.0f;
float coordinateQz = 0.0f;
float coordinateQw = 0.0f;
bool coordinatePoseCopied = false;
char coordinatePoseLine[256] = "";

bool Vis_GetMouseGroundPoint(glm::vec3& worldPoint, Eigen::Vector3f& mapPoint) {
   glm::vec3 rayOrigin, rayDirection;
   Vis_GetMouseRay(rayOrigin, rayDirection);
   if (!Vis_RayPlaneIntersect(
         rayOrigin, rayDirection, VIS_COORD_GROUND.origin,
         VIS_COORD_GROUND.axis_y, worldPoint))
      return false;

   Eigen::Vector3f alignedPoint(worldPoint.x, worldPoint.y, worldPoint.z);
   mapPoint = worldToPcd * alignedPoint;
   return true;
}

void Vis_UpdateCoordinateOrientation() {
   float dx = coordinateEndMap.x() - coordinateStartMap.x();
   float dy = coordinateEndMap.y() - coordinateStartMap.y();
   coordinateOrientationValid = dx * dx + dy * dy >= 0.0004f;
   if (!coordinateOrientationValid)
      return;

   coordinateYaw = atan2f(dy, dx);

   // places.txt stores the Unitree/map pose convention: planar yaw combined
   // with the fixed 180-degree rotation about X between the robot/body frames.
   // q_yaw(Z) * q_roll(X, pi) = (cos(yaw/2), sin(yaw/2), 0, 0).
   coordinateQx = cosf(coordinateYaw * 0.5f);
   coordinateQy = sinf(coordinateYaw * 0.5f);
   coordinateQz = 0.0f;
   coordinateQw = 0.0f;
}

void Vis_CopyCoordinatePose() {
   std::snprintf(
      coordinatePoseLine, sizeof(coordinatePoseLine),
      "%.6f %.6f %.6f %.6f %.6f %.6f %.6f",
      coordinateStartMap.x(), coordinateStartMap.y(), VIS_PLACE_POSE_Z,
      coordinateQx, coordinateQy, coordinateQz, coordinateQw);

   ImGui::SetClipboardText(coordinatePoseLine);
   std::cout << "places.txt pose: " << coordinatePoseLine << std::endl;
   coordinatePoseCopied = true;
}

void coordinateModeControls(bool updateSelection) {
   ImGuiIO& io = ImGui::GetIO();

   if (!updateSelection) {
      if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_C, false)) {
         showGroundCoordinates = !showGroundCoordinates;
         coordinatePickState = COORD_LIVE;
         coordinateCursorValid = false;
         coordinateOrientationValid = false;
         coordinatePoseCopied = false;
         VIS_CAMERA_CONTROLS = !showGroundCoordinates;
      }
      return;
   }

   if (!showGroundCoordinates)
      return;

   // While a drag is active, always observe its release even if the cursor
   // crosses an ImGui window. New selections never start through UI windows.
   if (coordinatePickState != COORD_DRAGGING && io.WantCaptureMouse)
      return;

   if (coordinatePickState == COORD_LOCKED) {
      if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
         coordinatePickState = COORD_LIVE;
         coordinateCursorValid = false;
         coordinateOrientationValid = false;
         coordinatePoseCopied = false;
      }
      return;
   }

   glm::vec3 worldPoint;
   Eigen::Vector3f mapPoint;
   bool hasGroundPoint = Vis_GetMouseGroundPoint(worldPoint, mapPoint);

   if (coordinatePickState == COORD_LIVE) {
      coordinateCursorValid = hasGroundPoint;
      if (hasGroundPoint)
         coordinateCursorMap = mapPoint;

      if (hasGroundPoint && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
         coordinateStartWorld = coordinateEndWorld = worldPoint;
         coordinateStartMap = coordinateEndMap = mapPoint;
         coordinateOrientationValid = false;
         coordinatePoseCopied = false;
         coordinatePickState = COORD_DRAGGING;
      }
      return;
   }

   if (hasGroundPoint) {
      coordinateEndWorld = worldPoint;
      coordinateEndMap = mapPoint;
      Vis_UpdateCoordinateOrientation();
   }

   if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
      coordinatePickState = COORD_LOCKED;
      if (coordinateOrientationValid)
         Vis_CopyCoordinatePose();
   }
}

void Vis_DrawCoordinateSelection() {
   if (!showGroundCoordinates || coordinatePickState == COORD_LIVE)
      return;

   glm::vec3 up = glm::normalize(VIS_COORD_GROUND.axis_y);
   glm::vec3 axisX = glm::normalize(VIS_COORD_GROUND.axis_x);
   glm::vec3 axisZ = glm::normalize(VIS_COORD_GROUND.axis_z);
   glm::vec3 start = coordinateStartWorld + up * 0.03f;
   const float markerSize = 0.12f;

   glDisable(GL_DEPTH_TEST);
   glLineWidth(3.0f);
   glColor3f(1.0f, 0.65f, 0.0f);
   glBegin(GL_LINES);
   glVertex3fv(glm::value_ptr(start - axisX * markerSize));
   glVertex3fv(glm::value_ptr(start + axisX * markerSize));
   glVertex3fv(glm::value_ptr(start - axisZ * markerSize));
   glVertex3fv(glm::value_ptr(start + axisZ * markerSize));

   if (coordinateOrientationValid) {
      glm::vec3 end = coordinateEndWorld + up * 0.03f;
      glm::vec3 direction = glm::normalize(end - start);
      glm::vec3 side = glm::normalize(glm::cross(up, direction));
      float arrowLength = glm::length(end - start);
      float headLength = std::min(0.30f, arrowLength * 0.30f);
      float headWidth = headLength * 0.55f;
      glm::vec3 headBase = end - direction * headLength;

      glVertex3fv(glm::value_ptr(start));
      glVertex3fv(glm::value_ptr(end));
      glVertex3fv(glm::value_ptr(end));
      glVertex3fv(glm::value_ptr(headBase + side * headWidth));
      glVertex3fv(glm::value_ptr(end));
      glVertex3fv(glm::value_ptr(headBase - side * headWidth));
   }
   glEnd();
   glLineWidth(1.0f);
   glEnable(GL_DEPTH_TEST);
}

void controls() {
   ImGuiIO& io = ImGui::GetIO();

   if (showGroundCoordinates) {
      VIS_CAMERA_CONTROLS = false;
      return;
   }

   VIS_CAMERA_CONTROLS = true;
   if (io.WantCaptureMouse)
      return;

   if (ImGui::IsKeyReleased(ImGuiKey_Escape)) {
      drawState = UNINIT;
      selectionState = EMPTY;
      selection = ModifiedSelection<Vis_Zone>();
   }

   // INITIATING SELECTION OPERATIONS
   if (selectionState == SINGLE && (ImGui::IsKeyPressed(ImGuiKey_R) || ImGui::IsKeyPressed(ImGuiKey_S))) {
      glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
      glm::vec3 hit;
      if (Vis_RayPlaneIntersect(rayO, rayD, { 0, 0, 0 }, VIS_SCENE_WORLD_UP_NORM, hit)) {
         initialRayAttach = rayAttach = Vis_Zone_WorldToLocal(*selection, hit);
         if (ImGui::IsKeyPressed(ImGuiKey_R))
            selectionState = SINGLE_DRAG;
         if (ImGui::IsKeyPressed(ImGuiKey_S))
            selectionState = SINGLE_SCALE;
      }
   }

   if (selectionState != EMPTY && ImGui::IsKeyReleased(ImGuiKey_Enter)) {
      selection.apply();
      selectionState = EMPTY;
   }


   // DRAWING A ZONE FROM CUBOID
   if (drawState == UNINIT) {
      if (ImGui::IsKeyPressed(ImGuiKey_D, false)) {
         glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);

         drawing_cuboid = Vis_Cuboid();
         if (Vis_RayPlaneIntersect(rayO, rayD, { 0, 0, 0 }, VIS_SCENE_WORLD_UP_NORM, drawing_cuboid.p0)) {
            drawState = START_POINT;
         }
      }
   }
   else if (drawState == START_POINT) {
      VIS_CAMERA_CONTROLS = false;
      glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
      if (!Vis_RayPlaneIntersect(rayO, rayD, {0, 0, 0}, VIS_SCENE_WORLD_UP_NORM, drawing_cuboid.p1)) {
         drawState = UNINIT;
      }

      if (!ImGui::IsKeyDown(ImGuiKey_D)) {
         drawState = UNINIT;

         if (glm::distance(drawing_cuboid.p0, drawing_cuboid.p1) > 0.1f) {
            selectionState = SINGLE;

            // increase height so it doesn't start at 0
            drawing_cuboid.p1.y = 1;

            Vis_Zone new_zone = drawing_cuboid.toZone();
            zones.push_back(new_zone);
            selection = ModifiedSelection(zones.back());
         }
      }
   }

   // EDITING A SELECTED ZONE
   if (selectionState == SINGLE) {
      VIS_CAMERA_CONTROLS = ImGui::IsMouseDown(ImGuiMouseButton_Right);
      if (VIS_CAMERA_CONTROLS) {
         VIS_CAMERA.pivot.x = selection->center.x;
         VIS_CAMERA.pivot.y = 0;
         VIS_CAMERA.pivot.z = selection->center.z;
      }
   }
   else if (selectionState == SINGLE_DRAG) {
      if (ImGui::IsKeyDown(ImGuiKey_R)) {
         VIS_CAMERA_CONTROLS = false;
         glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
         glm::vec3 target;
         if (Vis_RayPlaneIntersect(rayO, rayD, { 0, 0, 0 }, VIS_SCENE_WORLD_UP_NORM, target))
            Vis_Zone_ApplyDrag(*selection, initialRayAttach, target);
      }
      else {
         selectionState = SINGLE;
      }
   }
   else if (selectionState == SINGLE_SCALE) {
      if (ImGui::IsKeyDown(ImGuiKey_S)) {
         VIS_CAMERA_CONTROLS = false;
         glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
         glm::vec3 target;
         if (Vis_RayPlaneIntersect(rayO, rayD, { 0, 0, 0 }, VIS_SCENE_WORLD_UP_NORM, target))
            Vis_Zone_ApplyXZScale(*selection, rayAttach, target, initialRayAttach);
         rayAttach = Vis_Zone_WorldToLocal(*selection, target);
         
         selection->halfExtents.y = clamp(selection->halfExtents.y - ((float)io.MouseWheel / 100.0f) * logf(selection->halfExtents.y + 1.01f) * VIS_ZONE_Y_SCALE_SPEED, 0.0f, VIS_ZONE_MAX_SIZE);
         selection->center.y = selection->halfExtents.y;
      }
      else {
         selectionState = SINGLE;
      }
   }
}


// =========================================================================
// THE UI
// Anything related to UI lives here. This should allow for localized and
// simple setup of UI for various needs of the current scene.
// =========================================================================


CloudMap* cmap = nullptr;

// Modify this function to make changes to the UI
void makeui() {
   ImGui::Begin("Scene");
   ImGui::SetWindowPos(ImVec2(17, 17), ImGuiCond_Once);
   ImGui::SetWindowSize(ImVec2(200, 200), ImGuiCond_Once);
   VisUI_SliderFloatResettable("Pitch", &VIS_CAMERA.angles.pitch, VIS_CAMERA_PITCH_MIN, VIS_CAMERA_PITCH_MAX);
   VisUI_SliderFloatResettable("Yaw", &VIS_CAMERA.angles.yaw, -180.0f, 180.0f);
   VisUI_SliderFloatResettable("Roll", &VIS_CAMERA.angles.roll, -180.0f, 180.0f);
   ImGui::SliderInt("Projection", &VIS_CAMERA.projection_mode, 0, 1);
   if (ImGui::Button("Reset View"))
      Vis_Camera_Reset();
   ImGui::End();

   if (showGroundCoordinates) {
      ImGui::SetNextWindowPos(ImVec2(17, 230), ImGuiCond_Once);
      ImGui::SetNextWindowBgAlpha(0.75f);
      constexpr ImGuiWindowFlags overlayFlags =
         ImGuiWindowFlags_AlwaysAutoResize |
         ImGuiWindowFlags_NoDecoration |
         ImGuiWindowFlags_NoInputs |
         ImGuiWindowFlags_NoNav;
      ImGui::Begin("Ground coordinates", nullptr, overlayFlags);

      if (coordinatePickState == COORD_LIVE) {
         ImGui::TextUnformatted("Map cursor [C]");
         if (coordinateCursorValid) {
            ImGui::Text("X: %.3f", coordinateCursorMap.x());
            ImGui::Text("Y: %.3f", coordinateCursorMap.y());
            ImGui::TextUnformatted("Drag left mouse to set heading");
         }
         else {
            ImGui::TextUnformatted("X: --");
            ImGui::TextUnformatted("Y: --");
            ImGui::TextUnformatted("Ground is not under cursor");
         }
      }
      else {
         ImGui::TextUnformatted(
            coordinatePickState == COORD_DRAGGING
               ? "Selected pose (drag heading)"
               : "Selected pose");
         ImGui::Text("X: %.3f", coordinateStartMap.x());
         ImGui::Text("Y: %.3f", coordinateStartMap.y());
         ImGui::Text("Z: %.3f (pose origin)", VIS_PLACE_POSE_Z);

         if (coordinateOrientationValid) {
            ImGui::Text("Yaw: %.1f deg", RAD2DEG(coordinateYaw));
            ImGui::Separator();
            ImGui::TextUnformatted("Quaternion (q_x q_y q_z q_w)");
            ImGui::Text("%.6f  %.6f  %.6f  %.6f",
               coordinateQx, coordinateQy, coordinateQz, coordinateQw);
         }
         else {
            ImGui::TextUnformatted("Heading: drag farther to specify");
         }

         if (coordinatePickState == COORD_LOCKED) {
            if (coordinatePoseCopied)
               ImGui::TextUnformatted("Pose copied to clipboard");
            ImGui::TextUnformatted("Click to inspect another point");
         }
      }

      ImGui::End();
   }
   
   
   ImGui::Begin("Map");
   ImGui::SetWindowPos(ImVec2(930, 20), ImGuiCond_Once);
   ImGui::SetWindowSize(ImVec2(250, 400), ImGuiCond_Once);
   VisUI_SliderFloatResettable("Skip Points", &(cmap->SKIP), 0.0f, 1.0f, cmap->SKIP);

   ImGui::SliderFloat("Time Start", &(cmap->TIME_START), 0.0f, cmap->TIME_END);
   ImGui::SliderFloat("Time End", &(cmap->TIME_END), cmap->TIME_START, 1.0f);
   float pos_before, pos = pos_before = (cmap->TIME_START + cmap->TIME_END) / 2;

   ImGui::SliderFloat("Time Shift", &pos, 0.0f, 1.0f);
   cmap->TIME_START += pos - pos_before;
   cmap->TIME_END += pos - pos_before;

   ImGui::SliderInt("Bounding", &(cmap->DRAW_BOUNDING), 0, 1);

   if (ImGui::Button("Random Seed"))
      cmap->shuffleSkip();
   if (ImGui::Button("Reset All")) {
      cmap->TIME_START = 0.0f;
      cmap->TIME_END = 1.0f;
      cmap->SKIP = 0.95f;
      cmap->DRAW_BOUNDING = 0;
      drawState = UNINIT;
   }
   ImGui::End();
}



// =========================================================================
// THE MAIN LOOP
//
// =========================================================================

Shader vertexShader;
Shader fragmentShader;

PointCloud<PointXYZI> pointCloud;

void mainloop() {
   Vis_FrameStart();

   coordinateModeControls(false);
   Vis_UpdateInput();
   Vis_UpdateWorld();
   coordinateModeControls(true);

   Vis_LoadCoord(VIS_COORD_GROUND);
   Vis_DrawGrid();
   // Vis_DrawFloor();

   Vis_LoadCoord(VIS_COORD_WORLD);
   Vis_DrawGizmo();

   /* --------------------------- begin scene drawing -------------------------- */

   // glUseProgram(vertexShader);

   cmap->draw();

   controls();

   // DRAW EXISTING ZONES
   glColor3f(0.75f, 0.0f, 0.0f);
   for (Vis_Zone& zone : zones) {
      if (selectionState != EMPTY && selection.applyTo == &zone)
         continue;
      Vis_DrawZone(zone);
   }

   if (drawState == START_POINT) {
      Vis_DrawWireCube(drawing_cuboid.p0, drawing_cuboid.p1, drawing_cuboid.rot.vec3);
   }

   // DRAW SELECTED ZONE
   if (!selection.empty()) {
      glColor3f(0.25f, 0.25f, 1.0f);
      if (selectionState == SINGLE_DRAG || selectionState == SINGLE_SCALE)
         glColor3f(1.0f, 0.85f, 0.0f);
      Vis_DrawZone(selection.mod);
   }

   // Draw last so the selected pose remains visible over the map and zones.
   Vis_DrawCoordinateSelection();

   /* ---------------------------- end scene drawing --------------------------- */

   makeui();

   Vis_FrameEndAndRender();

   Vis_ShowFPS();
}


// =========================================================================
// ENTRY POINT
//
// =========================================================================


int main() {
   srand(time(NULL));

   /*
   nfdpathset_t outPaths;
   nfdchar_t filters[] = { "pcd" };
   nfdresult_t result = NFD_OpenDialogMultiple(filters, "./i9.pcd", &outPaths);

   if (result == NFD_OKAY) {
      puts("Success!");
      for (int i = 0; i < NFD_PathSet_GetCount(&outPaths); i++) {
         puts(NFD_PathSet_GetPath(&outPaths, i));
      }
   }
   else if (result == NFD_CANCEL) {
      puts("User pressed cancel.");
   }
   else {
      printf("Error: %s\n", NFD_GetError());
   }

   return 0;
   */

   if (Vis_Start() == NULL)
      return -1;

   /* ------------------------------- begin setup ------------------------------ */

   vertexShader = Vis_CreateShader("vis/shaders/test.vert", VERTEX);
   fragmentShader = Vis_CreateShader("vis/shaders/test.frag", FRAGMENT);

   PCLPointCloud2 pcdData;
   {
      PCDReader reader;
      if (reader.read("./i9.pcd", pcdData)) {
         std::cerr << "Failed to read PCD file" << std::endl;
         return 1;
      }
   }

   fromPCLPointCloud2(pcdData, pointCloud);

   Eigen::Affine3f pcdToWorld = Eigen::Affine3f::Identity();
   if (Vis_FloorToGroundAlignment(pointCloud, pcdToWorld)) {
      transformPointCloud(pointCloud, pointCloud, pcdToWorld);
      worldToPcd = pcdToWorld.inverse();
   }

   cmap = new CloudMap(pointCloud);

   /* -------------------------------- end setup ------------------------------- */

   Vis_MainLoop(mainloop);

   Vis_End();
   return 0;
}