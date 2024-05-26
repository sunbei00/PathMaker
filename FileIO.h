//
// Created by root on 24. 5. 26.
//

#ifndef PATHMAKER_FILEIO_H
#define PATHMAKER_FILEIO_H
#include <vector>
#include <utility>
#include <fstream>
#include <iostream>
#include <glm/vec3.hpp>

void saveToTextFile(const std::vector<std::pair<glm::vec3, float>>& posWithYaw, const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile) {
        throw std::runtime_error("Failed to open file for writing");
    }

    for (const auto& element : posWithYaw) {
        outFile << element.first.x << ' '
                << element.first.y << ' '
                << element.first.z << ' '
                << element.second << '\n';
    }

    outFile.close();
}

std::vector<std::pair<glm::vec3, float>> loadFromTextFile(const std::string& filename) {
    std::ifstream inFile(filename);
    if (!inFile) {
        throw std::runtime_error("Failed to open file for reading");
    }

    std::vector<std::pair<glm::vec3, float>> posWithYaw;
    glm::vec3 position;
    float yaw;

    while (inFile >> position.x >> position.y >> position.z >> yaw) {
        posWithYaw.emplace_back(position, yaw);
    }

    inFile.close();
    return posWithYaw;
}


#endif //PATHMAKER_FILEIO_H
