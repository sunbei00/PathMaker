#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare" 
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wformat-security"

#include "ImguiHelper.hpp"
#include "pcl-1.12/pcl/io/pcd_io.h"
#include "pcl-1.12/pcl/point_types.h"
#include "GLHelper.hpp"
#include "FileIO.h"
#include <vector>
#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"


rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

static std::vector<glm::vec3> sel;
static std::vector<glm::vec3> lineVertices;
static std::vector<glm::vec3> poses;
static std::string saveFolderPath = ".";


glm::vec3 pos = {0.0,0.0,1.0};
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto odom_position = msg->pose.pose.position;
    pos = {odom_position.x, odom_position.y, odom_position.z};
}

glm::vec3 linearInterpolate(const glm::vec3& start, const glm::vec3& end, float t) {
    return start + t * (end - start);
}

std::vector<glm::vec3> interpolateBetweenPoints(const glm::vec3& start, const glm::vec3& end, float intervalCm) {
    std::vector<glm::vec3> interpolatedPositions;

    float distance = glm::distance(start, end);  

    float distanceCm = distance * 100.0f; 

    int numIntervals = static_cast<int>(distanceCm / intervalCm);

    for (int i = 0; i <= numIntervals; ++i) {
        float t = static_cast<float>(i) / numIntervals;  
        glm::vec3 interpolatedPos = linearInterpolate(start, end, t);
        interpolatedPositions.push_back(interpolatedPos); 
    }

    return interpolatedPositions;
}


void publish_path(const std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> &path_publisher, const std::vector<glm::vec3> &positions, rclcpp::Time time_now)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = time_now;
    path_msg.header.frame_id = "map";
    for (const auto &pos : positions)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = time_now;
        pose.header.frame_id = "map";
        pose.pose.position.x = pos.x;
        pose.pose.position.y = pos.y;
        pose.pose.position.z = pos.z;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }       
    path_publisher->publish(path_msg);
}
std::string status = "not yet";
bool bstatus = false;

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("path_maker");
    
    auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10), std::bind(&odom_callback, std::placeholders::_1));
    path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("path", 10);


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
        rclcpp::spin_some(node); 
        if(test.size != -1) 
            removeMap(test);
        fillMap(test);
        glBindVertexArray(test.vao);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3), &pos.x, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
        test.size = 1;

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

            static int capture = 4.0;


            //ImGui::SetNextWindowSize(ImVec2(200, 100));
            //ImGui::SetNextWindowPos(ImVec2(0,0));

            ImGui::Begin("Path Maker");
            ImGui::SliderInt("split cm", &capture, 0.0, 100.0);

            // ImGuiFileDialog* fileDialog = ImGuiFileDialog::Instance();


            if(ImGui::Button("Send Path") && sel.size() >= 1){
                poses.clear();

                auto interpolated = interpolateBetweenPoints(pos, sel[0], capture);
                for(auto inter_pos : interpolated)
                    poses.push_back(inter_pos);
                    
                for(size_t i = 1; i < sel.size(); i++){
                    glm::vec3& srcPose = sel[i-1];
                    glm::vec3& targetPose = sel[i];

                    auto interpolated = interpolateBetweenPoints(srcPose, targetPose, capture);
                    for(auto inter_pos : interpolated)
                        poses.push_back(inter_pos);
                }

                rclcpp::Time now = node->now();
                publish_path(path_publisher_, poses, now);
                rclcpp::spin_some(node);
                status = "sent";
                bstatus = true;
            }

            ImGui::Text(status.c_str());
            // if (ImGui::Button("Select Save Location")) {
            //     fileDialog->OpenDialog("SelectFolderDlgKey", "Select Folder to Save", nullptr);
            // }

            // if (ImGuiFileDialog::Instance()->Display("SelectFolderDlgKey"))
            // {
            //     if (fileDialog->IsOk()) {
            //         saveFolderPath = ImGuiFileDialog::Instance()->GetCurrentPath();
            //     }
            //     ImGuiFileDialog::Instance()->Close();
            // }

            // ImGui::Text("Save Folder Path: %s", saveFolderPath.c_str());

            if (ImGui::Button("reset send")){
                if(bstatus){
                    bstatus = false;
                    status = "not yet";
                    sel.clear();
                    removeMap(sels);
                    removeMap(lines);

                    fillMap(sels);
                    fillMap(lines);
                }
            }

            ImGui::End();


            if(bstatus)
                goto selection;

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

                if (sel.size() != 0){
                    lineVertices.push_back(pos);
                    lineVertices.push_back(sel[0]);
                }

                for(int i=0; i < sel.size()-1; i++){
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
                if(sel.size() != 0){
                    removeMap(sels);
                    removeMap(lines);
                }

                if(sel.size() > 0){
                    sel.pop_back();


                    fillMap(sels);
                    glBindVertexArray(sels.vao);
                    glBufferData(GL_ARRAY_BUFFER, sel.size()* sizeof(glm::vec3), sel.data(), GL_STATIC_DRAW);
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
                    glEnableVertexAttribArray(0);
                    glBindVertexArray(0);
                    sels.size = sel.size();

                    lineVertices.clear();
                    if (sel.size() != 0){
                        lineVertices.push_back(pos);
                        lineVertices.push_back(sel[0]);
                    }
                    if(sel.size() >= 2)
                        for(int i=0; i < sel.size()-1; i++){
                            lineVertices.push_back(sel[i]);
                            lineVertices.push_back(sel[(i+1) % sel.size()]);
                        }
                    fillMap(lines);
                    glBindVertexArray(lines.vao);
                    glBufferData(GL_ARRAY_BUFFER, lineVertices.size()* sizeof(glm::vec3), lineVertices.data(), GL_STATIC_DRAW);
                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
                    glEnableVertexAttribArray(0);
                    glBindVertexArray(0);
                    lines.size = lineVertices.size();
                }
            }
        }

selection:



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

        pointSize = 8.0;
        glUniform1f(pointLoc, pointSize);
        glBindVertexArray(sels.vao);
        glDrawArrays(GL_POINTS, 0, sels.size);
        glBindVertexArray(0);

        pointSize = 3.0;
        glUniform1f(pointLoc, pointSize);
        glBindVertexArray(lines.vao);
        glDrawArrays(GL_LINES, 0, lines.size);
        glBindVertexArray(0);

        pointSize = 15.0;
        glUniform1f(pointLoc, pointSize);
        glBindVertexArray(test.vao);
        glDrawArrays(GL_POINTS, 0, test.size);
        glBindVertexArray(0);


        glUseProgram(0);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    removeMap(map);
    imguiTerminate(window);
    rclcpp::shutdown();

    return 0;
}

#pragma GCC diagnostic pop