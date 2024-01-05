#include"engine.h"

Engine::Engine() {
    V = Eigen::MatrixXd::Zero(1, 3);
    F = Eigen::MatrixXi::Zero(1, 3);
    simplifyStart = false;
    ss = SimplificationSolver(V, F);
    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);
}

Engine::Engine(Eigen::MatrixXd vertices, Eigen::MatrixXi faces) : V(vertices), F(faces){

    simplifyStart = false;
    ss = SimplificationSolver(V, F);
    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);
}

void Engine::launch() {
    viewer.core().is_animating = true;
    viewer.data().set_mesh(V, F);
    viewer.data().set_face_based(true);
    viewer.launch(false, "Model Simplification Processor");
}

void Engine::setupImgui() {

	setupDefaultCommandsList();
	setupConsole();
}
void Engine::setupDefaultCommandsList() {
	menu.callback_draw_viewer_menu = [&]()
    {
        if (ImGui::CollapsingHeader("Default Libigl Commands", ImGuiTreeNodeFlags_DefaultOpen))
        {

            ImGui::Text("[drag]  Rotate scene");
            ImGui::Text("A, a    Toggle animation(tight draw loop)");
            ImGui::Text("D, d    Toggle double sided lighting");
            ImGui::Text("F, f    Toggle face based");
            ImGui::Text("I, i    Toggle invert normals");
            ImGui::Text("L, l    Toggle wireframe");
            ImGui::Text("O, o    Toggle orthographic / perspective projection");
            ImGui::Text("S, s    Toggle shadows");
            ImGui::Text("T, t    Toggle filled faces");
            ImGui::Text("Z       Snap to canonical view");
            ImGui::Text("[, ]    Toggle between rotation control types(trackball, two - axis");
            ImGui::Text("   valuator with fixed up, 2D mode with no rotation))");
            ImGui::Text(";       Toggle vertex labels");
            ImGui::Text(":       Toggle face labels");
        }
    };
}
void Engine::setupConsole() {
    menu.callback_draw_custom_window = [&]()
    {
        ImGui::SetNextWindowPos(ImVec2(350.f * menu.menu_scaling(), 0), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(480, 240), ImGuiCond_FirstUseEver);
        ImGui::Begin(
            "Console", nullptr,
            ImGuiWindowFlags_NoSavedSettings
        );

        static float progress = 0.0f;
        const std::string path = "../../model/";
        static std::string loadedName = "File Name (.obj/off/ply)";
        static std::string savedName = "Saved File Name (.obj/off/ply)";
        static int numIter = 0;

        if (ImGui::CollapsingHeader("Upload", ImGuiTreeNodeFlags_DefaultOpen))
        {

            ImGui::InputText("Model File Name", loadedName);
            if (ImGui::Button("Load Model", ImVec2(-1, 0)))
            {

                std::string fileType = "";
                if (loadedName.size() > 4) fileType = loadedName.substr(loadedName.size() - 4, 4);

                bool load = false;
                if (fileType == ".obj") {
                    load = igl::readOBJ(path + loadedName, V, F);
                }
                else if (fileType == ".off") {
                    load = igl::readOFF(path + loadedName, V, F);
                }
                else if (fileType == ".ply") {
                    load = igl::readPLY(path + loadedName, V, F);
                }



                if (!load) {
                    loadedName = "No Found. Input New Name (.obj/off/ply)";
                }
                else {
                    simplifyStart = false;
                    progress = 0.0f;
                    numIter = 0;
                    viewer.data().clear();
                    viewer.data().set_mesh(V, F);
                    viewer.core().align_camera_center(V, F);
                    ss = SimplificationSolver(V, F);
                }
            }
        }

        if (ImGui::CollapsingHeader("Simplication", ImGuiTreeNodeFlags_DefaultOpen))
        {

            if (ImGui::Button("Process", ImVec2(-1, 0)))
            {
                if (ss.hasOpenBoundary()) {
                    loadedName = "Models with boundary are not supported";
                }
                else simplifyStart = true;
            }
            if (simplifyStart)
            {
                ss.simplify(progress);
            }
            ImGui::ProgressBar(progress, ImVec2(-1.0f, 0.0f));

            if (progress >= 1.0f) {
                Eigen::MatrixXd V1;
                Eigen::MatrixXi F1;
                ss.setIterationState(numIter, V1, F1);
                viewer.data().clear();
                viewer.data().set_mesh(V1, F1);
                viewer.core().align_camera_center(V1, F1);
                if (ImGui::Button("-", ImVec2(25.0f, 0.0f)) && numIter > 0) {
                    numIter -= 1;
                }
                ImGui::SameLine();
                if (ImGui::Button("+", ImVec2(25.0f, 0.0f)) && numIter < ss.getMaxIter()) {
                    numIter += 1;
                }
                ImGui::SameLine();
                ImGui::SliderInt("Iterations", &numIter, 0, ss.getMaxIter());
            }

        }

        if (progress >= 1.0f)
        {
            if (ImGui::CollapsingHeader("Save Current Model", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::InputText("Saved File Name", savedName);
                if (ImGui::Button("Save", ImVec2(-1, 0)))
                {
                    Eigen::MatrixXd V2;
                    Eigen::MatrixXi F2;

                    ss.getCleanedData(V2, F2);

                    int repeatCount = 1;
                    std::string fileType = "";
                    if (savedName.size() > 4) fileType = savedName.substr(savedName.size() - 4, 4);

                    if (fileType == ".obj") {
                        if (std::filesystem::exists(path + savedName)) {

                            std::string newPath = path + savedName.substr(0, savedName.size() - 4) + "-" + std::to_string(repeatCount) + ".obj";

                            while (std::filesystem::exists(newPath)) {
                                repeatCount++;
                                newPath = path + savedName.substr(0, savedName.size() - 4) + "-" + std::to_string(repeatCount) + ".obj";
                            }

                            if (igl::writeOBJ(newPath, V2, F2))
                                savedName = "Successfully Saved. (.obj/off)";
                            else
                                savedName = "Failed to Save. (.obj/off)";
                        }
                        else {
                            if (igl::writeOBJ(path + savedName, V2, F2))
                                savedName = "Successfully Saved. (.obj/off)";
                            else
                                savedName = "Failed to Save. (.obj/off)";
                        }
                    }
                    else if (fileType == ".off") {
                        if (std::filesystem::exists(path + savedName)) {
                            std::string newPath = path + savedName.substr(0, savedName.size() - 4) + "-" + std::to_string(repeatCount) + ".off";

                            while (std::filesystem::exists(newPath)) {
                                repeatCount++;
                                newPath = path + savedName.substr(0, savedName.size() - 4) + "-" + std::to_string(repeatCount) + ".off";
                            }

                            if (igl::writeOFF(newPath, V2, F2))
                                savedName = "Successfully Save. (.obj/off)";
                            else
                                savedName = "Failed to Save. (.obj/off)";
                        }
                        else {
                            if (igl::writeOFF(path + savedName, V2, F2))
                                savedName = "Successfully Save. (.obj/off)";
                            else
                                savedName = "Failed to Save. (.obj/off)";
                        }
                    }
                    else {
                        savedName = "Only support .obj and .off file";
                    }
                }
            }
        }

        ImGui::End();
    };
}