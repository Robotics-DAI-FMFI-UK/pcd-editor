#include "main.h"

// native libs
#include <cmath>
#include <time.h>
#include <iostream>
#include <fstream>

#include "vis/shaders.h"
#include "vis/world.h"
#include "vis/drawing.h"
#include "vis/maps.h"


using namespace std;
using namespace pcl;


// =========================================================================
// GLOBALS
//
// =========================================================================

Shader vertexShader;
Shader fragmentShader;

PointCloud<PointXYZI> pointCloud;

CloudMap* cmap = nullptr;

vector<Vis_Zone> zones;
SelectionState selectionState = EMPTY;
vector<ModifiedSelection<Vis_Zone>> selected;
DrawState drawState = UNINIT;

Vis_CuboidCorners drawing_cuboid;            // (TODO: remove?) used as temporary container for new zone
glm::vec3 initialRayAttach;
glm::vec3 rayAttach;


// =========================================================================
// UTILITY FUNCTIONS
//
// =========================================================================

// Writes a CSV field, quoting it if it contains commas, quotes or newlines
static void Vis_CsvField(FILE* f, const char* s) {
   if (!std::strpbrk(s, ",\"\n\r")) { std::fputs(s, f); return; }
   std::fputc('"', f);
   for (; *s; s++) {
      if (*s == '"') std::fputc('"', f);   // escape quotes by doubling
      std::fputc(*s, f);
   }
   std::fputc('"', f);
}

// Splits one CSV line into fields, handling "quoted, fields" and "" escapes
static std::vector<std::string> Vis_CsvSplit(const std::string& line) {
   std::vector<std::string> fields;
   std::string cur;
   bool quoted = false;
   for (size_t i = 0; i < line.size(); i++) {
      char c = line[i];
      if (quoted) {
         if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') { cur += '"'; i++; }
         else if (c == '"') quoted = false;
         else cur += c;
      }
      else if (c == '"') quoted = true;
      else if (c == ',') { fields.push_back(cur); cur.clear(); }
      else if (c != '\r') cur += c;
   }
   fields.push_back(cur);
   return fields;
}

static bool Vis_ParseFloat(const std::string& s, float& out) {
   char* end = nullptr;
   out = std::strtof(s.c_str(), &end);
   return end != s.c_str() && *end == 0;
}

int Vis_ImportZonesCSV(const char* path, std::vector<Vis_Zone>& zones) {
   std::ifstream in(path);
   if (!in) return -1;

   int loaded = 0;
   std::string line;
   while (std::getline(in, line)) {
      if (line.empty() || line.rfind("label,", 0) == 0) continue;   // blank line / header

      std::vector<std::string> f = Vis_CsvSplit(line);
      if (f.size() < 15) continue;

      float v[13];
      bool ok = true;
      for (int i = 0; i < 13; i++) ok &= Vis_ParseFloat(f[2 + i], v[i]);
      if (!ok) continue;

      Vis_Zone z;   // constructor assigns a fresh id and default color
      std::strncpy(z.label, f[0].c_str(), sizeof(z.label) - 1);
      z.label[sizeof(z.label) - 1] = 0;
      if (!f[1].empty()) {
         std::strncpy(z.id, f[1].c_str(), sizeof(z.id) - 1);
         z.id[sizeof(z.id) - 1] = 0;
      }

      z.center = cmap->transform(glm::vec3(v[0], v[1], v[2]));
      z.halfExtents = glm::vec3(v[3], v[4], v[5]);

      glm::quat q(v[9], v[6], v[7], v[8]);
      const float len = glm::length(q);
      z.rot = cmap->transform(len > 1e-6f ? q / len : glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

      z.color = glm::vec3(v[10], v[11], v[12]);

      zones.push_back(z);
      loaded++;
   }
   return loaded;
}

bool Vis_ExportZonesCSV(const char* path, const std::vector<Vis_Zone>& zones) {
   FILE* f = std::fopen(path, "w");
   if (!f) return false;

   std::fputs("label,id,center_x,center_y,center_z,half_x,half_y,half_z,qx,qy,qz,qw,r,g,b\n", f);

   for (const Vis_Zone& z : zones) {
      glm::vec3 c = cmap->original(z.center);        // undo applied transforms
      glm::quat q = cmap->original(z.rot);

      Vis_CsvField(f, z.label); std::fputc(',', f);
      Vis_CsvField(f, z.id);
      std::fprintf(f, ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f\n",
         c.x, c.y, c.z,
         z.halfExtents.x, z.halfExtents.y, z.halfExtents.z,
         q.x, q.y, q.z, q.w,
         z.color.x, z.color.y, z.color.z);
   }

   std::fclose(f);
   return true;
}

inline void Vis_UUID(char* out) {
   static thread_local std::mt19937_64 rng = [] {
      std::random_device rd;
      return std::mt19937_64((std::uint64_t(rd()) << 32) | rd());
      }();

   std::uint64_t a = rng(), b = rng();
   a = (a & 0xFFFFFFFFFFFF0FFFULL) | 0x4000ULL;                 // version 4
   b = (b & 0x3FFFFFFFFFFFFFFFULL) | 0x8000000000000000ULL;     // RFC 4122 variant

   std::snprintf(out, 37, "%08llx-%04llx-%04llx-%04llx-%012llx",
      (unsigned long long)(a >> 32),
      (unsigned long long)((a >> 16) & 0xFFFF),
      (unsigned long long)(a & 0xFFFF),
      (unsigned long long)(b >> 48),
      (unsigned long long)(b & 0xFFFFFFFFFFFFULL));
}

void Vis_DrawZone(const Vis_CuboidCenter& zone) {
   Vis_DrawWireCube(zone.center - zone.halfExtents, zone.center + zone.halfExtents, zone.rot);
}

bool Vis_GetZoneFromRay(glm::vec3 rayO, glm::vec3 rayD, Vis_Zone*& out) {
   Vis_Zone* closest = nullptr;
   float closestDist = FLT_MAX;

   for (Vis_Zone& zone : ZONES) {
      glm::vec3 hitPoint;
      if (!Vis_RayCuboidIntersect(rayO, rayD, zone, hitPoint)) continue;

      float dist = glm::distance(hitPoint, rayO);
      if (dist < closestDist) {
         closestDist = dist;
         closest = &zone;
      }
   }

   if (!closest) return false;   // miss: out is left untouched
   out = closest;
   return true;
}

// =========================================================================
// SCENE LOGIC
// Logic controlling the scene and the objects inside of it.
// =========================================================================

void reset_selection() {
   for (auto s : selected)
      s.apply();
   selected.clear();
   selectionState = EMPTY;
   drawState = UNINIT;
}

void controls() {
   ImGuiIO& io = ImGui::GetIO();

   VIS_CAMERA_CONTROLS = true;
   // if (io.WantCaptureMouse)
   //    return;
   if (io.WantTextInput)
      return;

   if (ImGui::IsKeyReleased(ImGuiKey_Escape)) {
      reset_selection();
   }

   if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
      Vis_Zone* zone;
      if (Vis_GetZoneFromRay(rayO, rayD, zone)) {
         reset_selection();
         selected.push_back(*zone);
         selectionState = SINGLE;
      }
   }

   // INITIATING SELECTION OPERATIONS
   if (selectionState == SINGLE && (ImGui::IsKeyPressed(ImGuiKey_R) || ImGui::IsKeyPressed(ImGuiKey_S))) {
      glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
      glm::vec3 hit;
      for (ModifiedSelection<Vis_Zone> sel : selected) {
         if (Vis_RayPlaneIntersect(rayO, rayD, { 0, 0, 0 }, VIS_SCENE_WORLD_UP_NORM, hit)) {
            initialRayAttach = rayAttach = Vis_Zone_WorldToLocal(*sel, hit);
            if (ImGui::IsKeyPressed(ImGuiKey_R))
               selectionState = SINGLE_DRAG;
            if (ImGui::IsKeyPressed(ImGuiKey_S))
               selectionState = SINGLE_SCALE;
         }
      }
   }

   if (selectionState != EMPTY && ImGui::IsKeyPressed(ImGuiKey_Enter)) {
      reset_selection();
   }

   if (selectionState != EMPTY && ImGui::IsKeyPressed(ImGuiKey_Delete)) {
      for (ModifiedSelection<Vis_Zone> sel : selected) {
         size_t index = sel.applyTo - zones.data();
         zones.erase(zones.begin() + index);
      }
      selected.clear();
      reset_selection();
   }


   // DRAWING A ZONE FROM CUBOID
   if (drawState == UNINIT) {
      if (ImGui::IsKeyPressed(ImGuiKey_D, false)) {
         reset_selection();
         
         glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);

         drawing_cuboid = Vis_CuboidCorners();
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

            Vis_Zone new_zone = drawing_cuboid.toCenter();
            zones.push_back(new_zone);
            selected.push_back(zones.back());
         }
      }
   }

   // EDITING A SELECTED ZONE
   if (selectionState == SINGLE) {
      VIS_CAMERA_CONTROLS = ImGui::IsMouseDown(ImGuiMouseButton_Right);
      if (VIS_CAMERA_CONTROLS) {
         VIS_CAMERA.pivot.x = selected.at(0)->center.x;
         VIS_CAMERA.pivot.y = 0;
         VIS_CAMERA.pivot.z = selected.at(0)->center.z;
      }
   }
   else if (selectionState == SINGLE_DRAG) {
      if (ImGui::IsKeyDown(ImGuiKey_R)) {
         VIS_CAMERA_CONTROLS = false;
         glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
         glm::vec3 target;
         if (Vis_RayPlaneIntersect(rayO, rayD, { 0, 0, 0 }, VIS_SCENE_WORLD_UP_NORM, target))
            Vis_Zone_ApplyDrag(*selected.at(0), initialRayAttach, target);
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
            Vis_Zone_ApplyXZScale(*selected.at(0), rayAttach, target, initialRayAttach);
         rayAttach = Vis_Zone_WorldToLocal(*selected.at(0), target);
         
         selected.at(0)->halfExtents.y = clamp(selected.at(0)->halfExtents.y + ((float)io.MouseWheel / 100.0f) * logf(selected.at(0)->halfExtents.y + 1.01f) * VIS_ZONE_Y_SCALE_SPEED, 0.0f, VIS_ZONE_MAX_SIZE);
         selected.at(0)->center.y = selected.at(0)->halfExtents.y;
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


void makeui() {
   const ImGuiViewport* viewport = ImGui::GetMainViewport();


   ImGui::Begin("Scene", nullptr, ImGuiTableFlags_NoSavedSettings);
   ImGui::SetWindowPos(ImVec2(17, 17), ImGuiCond_Once);
   ImGui::SetWindowSize(ImVec2(250, 200), ImGuiCond_Once);
   VisUI_SliderFloatResettable("Pitch", &VIS_CAMERA.angles.pitch, VIS_CAMERA_PITCH_MIN, VIS_CAMERA_PITCH_MAX);
   VisUI_SliderFloatResettable("Yaw", &VIS_CAMERA.angles.yaw, -180.0f, 180.0f);
   VisUI_SliderFloatResettable("Roll", &VIS_CAMERA.angles.roll, -180.0f, 180.0f);
   ImGui::SliderInt("Projection", &VIS_CAMERA.projection_mode, 0, 1);
   if (ImGui::Button("Reset View"))
      Vis_Camera_Reset();
   ImGui::End();
   
   
   ImGui::Begin("Map", nullptr, ImGuiTableFlags_NoSavedSettings);
   ImGui::SetWindowPos(ImVec2(930, 20), ImGuiCond_Once);
   ImGui::SetWindowSize(ImVec2(250, 200), ImGuiCond_Once);
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

   if (ImGui::Button("Export zones to CSV")) {
      auto path = "./zones.csv";
      if (Vis_ExportZonesCSV(path, zones))
         printf("Exported %zu zones to %s\n", zones.size(), path);
      else
         printf("Failed to write %s\n", path);
   }

   ImGui::End();


   if (selectionState != EMPTY && selected.size() > 0) {
      ImGui::Begin("Selection Properties", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse | ImGuiTableFlags_NoSavedSettings);
      ImGui::SetWindowPos(ImVec2(770, 450), ImGuiCond_Once);
      ImGui::SetWindowSize(ImVec2(410, 300), ImGuiCond_Once);


      ModifiedSelection<Vis_Zone> &sel = selected.at(0);

      ImGui::InputText("Zone Label", sel->label, SIZE(sel->label));
      ImGui::InputText("Zone ID", sel->id, SIZE(sel->id), ImGuiInputTextFlags_ReadOnly);

      
      ImGui::DragFloat3("RGB", &sel->color.x, 0.005f, 0.0f, 1.0f, "%.3f");
      ImGui::SameLine();
      if (ImGui::ColorButton("##preview", ImVec4(sel->color.x, sel->color.y, sel->color.z, 1.0f),
         ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker,
         ImVec2(24, 24))) {

         static const glm::vec3 PALETTE[] = {
            { 0.750f, 0.000f, 0.000f },   // red
            { 0.000f, 0.750f, 0.000f },   // green
            { 0.221f, 0.221f, 1.000f },   // blue (see note)
            { 0.750f, 0.000f, 0.750f },   // magenta
            { 0.000f, 0.750f, 0.750f },   // cyan
            { 0.750f, 0.750f, 0.000f },   // yellow
         };
         constexpr int N = SIZE(PALETTE);

         int next = 0;   // custom color (not in palette) -> start from red
         for (int i = 0; i < N; i++) {
            if (sel->color == PALETTE[i]) { next = (i + 1) % N; break; }
         }
         sel->color = PALETTE[next];
      }

      ImGui::Spacing(); ImGui::Spacing();

      sel->halfExtents *= 2;
      ImGui::DragFloat3("Zone size XYZ", &sel->halfExtents.x, 0.05f);
      sel->halfExtents /= 2;

      ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();

      ImGui::DragFloat3("Zone XYZ", &sel->center.x, 0.05f);
      auto og_center = cmap->original(sel->center);
      ImGui::BeginDisabled();
      ImGui::DragFloat3("Zone map XYZ", &og_center.x, 0, 0, 0, "%.3f", ImGuiInputTextFlags_ReadOnly);
      ImGui::EndDisabled();
      // TODO: make ^^^^^^^^ read-write

      ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();

      glm::quat q = sel->rot;
      if (ImGui::DragFloat4("Zone rotation XYZW", &q.x, 0.005f, -1.0f, 1.0f, "%.4f")) {
         const float len = glm::length(q);
         sel->rot = (len > 1e-6f) ? q / len : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      }
      glm::quat og = cmap->original(sel->rot);
      ImGui::BeginDisabled();
      ImGui::DragFloat4("Zone map rotation XYZW", &og.x, 0.0f, 0.0f, 0.0f, "%.4f");
      ImGui::EndDisabled();
      // TODO: make ^^^^^^^^ read-write

      ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();

      if (ImGui::Button("Ok", ImVec2(50, 20))) {
         reset_selection();
      }

      ImGui::End();
   }


   ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x, viewport->Pos.y + viewport->Size.y - ImGui::GetFrameHeight()));
   ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, ImGui::GetFrameHeight()));

   // Add menu bar flag and disable everything else
   ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse |
      ImGuiWindowFlags_NoSavedSettings |
      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoBackground |
      ImGuiWindowFlags_MenuBar;

   ImGui::Begin("StatusBar", nullptr, flags);
   ImGui::BeginMenuBar();
   {// DRAWING INFO
      if (drawState != UNINIT) {
         ImGui::Text("(DRAWING) ");
      }
   }
   {// ZONE INFO
      if (selectionState != EMPTY) {
         switch (selectionState) {
         case SINGLE_DRAG:
            ImGui::Text("(DRAG/ROTATE) ");
            break;
         case SINGLE_SCALE:
            ImGui::Text("(RESIZE) ");
            break;
         default:
            ImGui::Text("(SELECTED) ");
         }
      }
   }
   { // CURSOR COORDINATES
      glm::vec3 rayO, rayD; Vis_GetMouseRay(rayO, rayD);
      glm::vec3 target;
      if (Vis_RayPlaneIntersect(rayO, rayD, { 0, 0, 0 }, VIS_SCENE_WORLD_UP_NORM, target)) {
         ImGui::Text("GROUND[% 6.3f, % 6.3f] ", target.x, target.z);
         auto o = cmap->original(target);
         ImGui::Text("MAP[% 6.3f, % 6.3f, % 6.3f] ", o.x, o.y, o.z);
      }
      Vis_Zone* zone;
      if (Vis_GetZoneFromRay(rayO, rayD, zone)) {
         ImGui::Text("Zone{ '%s', '%s' } ", zone->label, zone->id + 30);
      }
   }
   ImGui::EndMenuBar();
   ImGui::End();
}



// =========================================================================
// THE MAIN LOOP
//
// =========================================================================

void mainloop() {
   Vis_FrameStart();

   Vis_UpdateInput();
   Vis_UpdateWorld();

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
   for (Vis_Zone& zone : ZONES) {
      glColor3f(zone.color.r, zone.color.g, zone.color.b);
      Vis_DrawZone(zone);
   }

   glColor3f(0.75f, 0.0f, 0.0f);
   if (drawState == START_POINT) {
      Vis_DrawWireCube(drawing_cuboid.p0, drawing_cuboid.p1, drawing_cuboid.rot);
   }

   // DRAW SELECTED ZONE
   if (!selected.empty()) {
      glColor3f(1.0f, 1.0f, 1.0f);
      if (selectionState == SINGLE_DRAG || selectionState == SINGLE_SCALE)
         glColor3f(1.0f, 0.85f, 0.0f);
      Vis_DrawZone(selected.at(0).mod);
   }

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
   cmap = new CloudMap(pointCloud);

   Eigen::Affine3f groundTransform;
   Vis_FloorToGroundAlignment(pointCloud, groundTransform);
   cmap->transformMap(groundTransform);

   Vis_ImportZonesCSV("./zones.csv", zones);

   /* -------------------------------- end setup ------------------------------- */

   Vis_MainLoop(mainloop);

   Vis_End();
   return 0;
}