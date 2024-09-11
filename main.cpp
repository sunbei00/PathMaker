#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare" 
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "ImguiHelper.hpp"
#include "pcl-1.12/pcl/io/pcd_io.h"
#include "pcl-1.12/pcl/point_types.h"
#include "GLHelper.hpp"
#include "FileIO.h"
#include <vector>
#include <cstdlib>
#include <ctime>


static std::vector<glm::vec3> sel;
static std::vector<glm::vec3> lineVertices;
static std::vector<std::pair<glm::vec3,glm::vec3>> poses;
static std::vector<std::pair<glm::vec3,float>> posWithYaw;
static std::string saveFolderPath = ".";



#define MODE_1        // sampling method

int main(int, char**)
{
    srand(time(nullptr));
    GLFWwindow* window = imguiInit();
    if(window == nullptr)
        return -1;

    glMap map;
    fillMap(map);
    glMap sels;
    glMap lines;
    //fillMap(sels);
    glMap test;

    GLuint shaderProgram = returnProgram();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if(map.size == -1){
            std::string path = ShowFileDialog();
            if(path != "none"){
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
                {
                    PCL_ERROR("Couldn't read the file \n");
                    return -1;
                }
                glBindVertexArray(map.vao);
                glBufferData(GL_ARRAY_BUFFER, cloud->points.size() * sizeof(pcl::PointXYZ), &cloud->points[0], GL_STATIC_DRAW);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZ), (GLvoid*)0);
                glEnableVertexAttribArray(0);
                glBindVertexArray(0);
                map.size = cloud->points.size();
            }
        } else {

            static int capture = 300;


            //ImGui::SetNextWindowSize(ImVec2(200, 100));
            //ImGui::SetNextWindowPos(ImVec2(0,0));

            ImGui::Begin("Path Maker");
#ifdef MODE_1
            ImGui::SliderInt("Capture amount", &capture, 10, 500);
#endif

            ImGuiFileDialog* fileDialog = ImGuiFileDialog::Instance();


            if(ImGui::Button("make Path") && sel.size() >= 2){

                poses.clear();
                posWithYaw.clear();

#ifdef MODE_1
                int posePerLine = capture / sel.size();

                glm::vec3 center = glm::vec3(0,0,0);
                for(auto& it : sel){
                    center.x += (it.x / sel.size());
                    center.y += (it.y / sel.size());
                    center.z += (it.z / sel.size());
                }


                for(size_t i = 0; i < sel.size(); i++){
                    glm::vec3& srcPose = sel[i];
                    glm::vec3& targetPose = sel[(i+1)%sel.size()];

                    for(size_t j =0; j < posePerLine; j++){
                        glm::vec3 samplingPose;
                        samplingPose.x = srcPose.x * (1 - 1.0/posePerLine*(float)j) + targetPose.x * (1.0/posePerLine*(float)j);
                        samplingPose.y = srcPose.y * (1 - 1.0/posePerLine*(float)j) + targetPose.y * (1.0/posePerLine*(float)j);
                        samplingPose.z = srcPose.z * (1 - 1.0/posePerLine*(float)j) + targetPose.z * (1.0/posePerLine*(float)j);

                        glm::vec3 noise = center - samplingPose;
                        noise.x *= 0.7 * ((float)rand()/RAND_MAX);
                        glm::vec3 centerWithNoise = noise + center;

                        glm::vec3 direction = glm::normalize(centerWithNoise - samplingPose);
                        float yaw = atan2(direction.y, direction.x);
                        //yaw = glm::degrees(yaw);

                        posWithYaw.emplace_back(samplingPose, yaw);
                        poses.push_back({samplingPose, samplingPose + direction});
                    }
                }
#endif



                saveToTextFile(posWithYaw, saveFolderPath + "/path.txt");

                if(test.size != -1)
                    removeMap(test);

                fillMap(test);
                glBindVertexArray(test.vao);
                glBufferData(GL_ARRAY_BUFFER, poses.size()* sizeof(glm::vec3)*2, &poses[0].first, GL_STATIC_DRAW);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
                glEnableVertexAttribArray(0);
                glBindVertexArray(0);
                test.size = poses.size()*2;


            }

            if (ImGui::Button("Select Save Location")) {
                fileDialog->OpenDialog("SelectFolderDlgKey", "Select Folder to Save", nullptr);
            }

            if (ImGuiFileDialog::Instance()->Display("SelectFolderDlgKey"))
            {
                if (fileDialog->IsOk()) {
                    saveFolderPath = ImGuiFileDialog::Instance()->GetCurrentPath();
                }
                ImGuiFileDialog::Instance()->Close();
            }

            ImGui::Text("Save Folder Path: %s", saveFolderPath.c_str());

            ImGui::End();


            if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                int display_w, display_h;
                glfwGetFramebufferSize(window, &display_w, &display_h);
                ImVec2 mousePose = ImGui::GetMousePos();
                glm::vec3 ray_direction = raycast(mousePose.x, mousePose.y, ImVec2(display_w,display_h));
                glm::vec3 ray_origin = glm::vec3(cam.cameraX, cam.cameraY, cam.cameraZoom);
                glm::vec3 intersection_point = getIntersectionPointWithZPlane(ray_direction, ray_origin, 0.5f);

                sel.push_back(intersection_point);

                if(sels.size != -1){
                    removeMap(sels);
                    removeMap(lines);
                }


                fillMap(sels);
                glBindVertexArray(sels.vao);
                glBufferData(GL_ARRAY_BUFFER, sel.size()* sizeof(glm::vec3), &sel[0], GL_STATIC_DRAW);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
                glEnableVertexAttribArray(0);
                glBindVertexArray(0);
                sels.size = sel.size();

                lineVertices.clear();

#if defined(MODE_1)
                for(int i=0; i < sel.size(); i++){
#else
                for(int i=0; i < sel.size()-1; i++){
#endif

                    lineVertices.push_back(sel[i]);
                    lineVertices.push_back(sel[(i+1) % sel.size()]);
                }

                fillMap(lines);
                glBindVertexArray(lines.vao);
                glBufferData(GL_ARRAY_BUFFER, lineVertices.size()* sizeof(glm::vec3), &lineVertices[0], GL_STATIC_DRAW);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
                glEnableVertexAttribArray(0);
                glBindVertexArray(0);
                lines.size = lineVertices.size();


            }else if(ImGui::IsMouseClicked(ImGuiMouseButton_Middle)){
                if(sels.size != -1){
                    removeMap(sels);
                    removeMap(lines);
                }
                if(sel.size() > 0){
                    sel.pop_back();

                    fillMap(sels);
                    glBindVertexArray(sels.vao);
                    glBufferData(GL_ARRAY_BUFFER, sel.size()* sizeof(glm::vec3), &sel[0], GL_STATIC_DRAW);
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
                    glEnableVertexAttribArray(0);
                    glBindVertexArray(0);
                    sels.size = sel.size();

                    lineVertices.clear();
                    for(int i=0; i < sel.size(); i++){
                        lineVertices.push_back(sel[i]);
                        lineVertices.push_back(sel[(i+1) % sel.size()]);
                    }

                    fillMap(lines);
                    glBindVertexArray(lines.vao);
                    glBufferData(GL_ARRAY_BUFFER, lineVertices.size()* sizeof(glm::vec3), &lineVertices[0], GL_STATIC_DRAW);
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
                    glEnableVertexAttribArray(0);
                    glBindVertexArray(0);
                    lines.size = lineVertices.size();
                }
            }
        }

        // Handle mouse dragging
        static bool dragging = false;
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            static ImVec2 drag_start_pos;
            if (!dragging) {
                drag_start_pos = ImGui::GetMousePos();
                dragging = true;
            }
            ImVec2 drag_current_pos = ImGui::GetMousePos();
            ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
            cam.cameraX -= drag_delta.x * (cam.cameraZoom/500.0);
            cam.cameraY += drag_delta.y * (cam.cameraZoom/500.0);
            ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);
        } else {
            dragging = false;
        }

        ImGuiIO& io = ImGui::GetIO();
        float wheel = io.MouseWheel;
        if (wheel != 0.0f) {
            cam.cameraZoom -= wheel * 3.0f; // Adjust the factor as needed
            if(cam.cameraZoom <= 1)
                cam.cameraZoom = 5.0f;
        }



        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        //glClearColor(1.0f, 1.0f, 1.0f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view;
        glm::mat4 projection;
        getMatrix(view,projection);

        glUseProgram(shaderProgram);
        GLuint projectionLoc = glGetUniformLocation(shaderProgram, "projection");
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
        GLuint viewLoc = glGetUniformLocation(shaderProgram, "view");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        GLuint pointLoc = glGetUniformLocation(shaderProgram, "pointSize");

        float pointSize = 1.0;
        glUniform1f(pointLoc, pointSize);
        glBindVertexArray(map.vao);
        glDrawArrays(GL_POINTS, 0, map.size);
        glBindVertexArray(0);

        pointSize = 10.0;
        glUniform1f(pointLoc, pointSize);
        glBindVertexArray(sels.vao);
        glDrawArrays(GL_POINTS, 0, sels.size);
        glBindVertexArray(0);

        pointSize = 3.0;
        glUniform1f(pointLoc, pointSize);
        glBindVertexArray(lines.vao);
        glDrawArrays(GL_LINES, 0, lines.size);
        glBindVertexArray(0);

        pointSize = 1.0;
        glUniform1f(pointLoc, pointSize);
        glBindVertexArray(test.vao);
        glDrawArrays(GL_LINES, 0, test.size);
        glBindVertexArray(0);


        glUseProgram(0);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    removeMap(map);
    imguiTerminate(window);

    return 0;
}

#pragma GCC diagnostic pop