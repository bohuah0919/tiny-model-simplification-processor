#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <filesystem>
#include"solver.h"

class Engine {
public:
	Engine();
	Engine(Eigen::MatrixXd vertices, Eigen::MatrixXi faces);
	void setupImgui();
	void launch();
private:
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	igl::opengl::glfw::Viewer viewer;
	igl::opengl::glfw::imgui::ImGuiPlugin plugin;
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	SimplificationSolver ss;
	bool simplifyStart;
	void setupDefaultCommandsList();
	void setupConsole();
};
