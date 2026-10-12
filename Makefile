CXX      := g++
CC       := gcc
FLAGS := \
       -Iexternal/imgui -Iexternal/imgui/backends \
       -Iexternal/nfd/src -Iexternal/nfd/src/include \
       -I/usr/include/pcl-1.12 -I/usr/include/eigen3 -I/usr/include/ni -I/usr/include/openni2 \
       -I/usr/include/eigen3
LDFLAGS  := -lglfw -lGL -ldl -L/usr/lib64 -lGLEW -lX11 -lGLU \
       -lpcl_io -lOpenNI -lOpenNI2 -lpcl_octree -lpcl_common \
       -lpcl_segmentation -lpcl_features -lpcl_filters -lpcl_sample_consensus -lpcl_search -lpcl_kdtree -llz4 -lflann -lflann_cpp -llz4 -lpcl_octree -lpcl_ml -lpcl_common

SRC := main.cpp \
       \
       vis/core.cpp \
       vis/drawing.cpp \
       vis/world.cpp \
       vis/shaders.cpp \
       vis/maps.cpp \
       vis/algo.cpp \
       \
       external/imgui/imgui.cpp \
       external/imgui/imgui_draw.cpp \
       external/imgui/imgui_tables.cpp \
       external/imgui/imgui_widgets.cpp \
       external/imgui/imgui_demo.cpp \
       external/imgui/backends/imgui_impl_glfw.cpp \
       external/imgui/backends/imgui_impl_opengl2.cpp \
       \
       external/nfd/src/nfd_common.c \
       external/nfd/src/nfd_zenity.c

OBJDIR := build
OBJ := $(patsubst %.cpp,$(OBJDIR)/%.o,$(filter %.cpp,$(SRC))) \
       $(patsubst %.c,$(OBJDIR)/%.o,$(filter %.c,$(SRC)))

app: $(OBJ)
	$(CXX) $(OBJ) -o main $(LDFLAGS)


define compile_cxx_o
	@mkdir -p $(dir $@)
	$(CXX) -std=c++20 $(FLAGS) -c $< -o $@
endef

define compile_c_o
	@mkdir -p $(dir $@)
	$(CC) -std=c11 $(FLAGS) -c $< -o $@
endef

$(OBJDIR)/%.o: %.cpp
	$(call compile_cxx_o)

$(OBJDIR)/%.o: %.c
	$(call compile_c_o)