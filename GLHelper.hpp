#ifndef GL_HELPER
#define GL_HELPER

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec3 iPos;


out float oPos;

uniform float pointSize;
uniform mat4 projection;
uniform mat4 view;

void main()
{
    oPos = iPos.z;
    if(pointSize >= 1.1)
        oPos = 100.0;
    gl_Position = projection * view * vec4(iPos, 1.0);
    gl_PointSize = pointSize;
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
in float oPos;
out vec4 FragColor;

vec3 hsv2rgb(float h, float s, float v) {
    float c = v * s;
    float x = c * (1.0 - abs(mod(h * 6.0, 2.0) - 1.0));
    float m = v - c;
    vec3 rgb;

    if (0.0 <= h && h < 1.0 / 6.0) {
        rgb = vec3(c, x, 0.0);
    } else if (1.0 / 6.0 <= h && h < 2.0 / 6.0) {
        rgb = vec3(x, c, 0.0);
    } else if (2.0 / 6.0 <= h && h < 3.0 / 6.0) {
        rgb = vec3(0.0, c, x);
    } else if (3.0 / 6.0 <= h && h < 4.0 / 6.0) {
        rgb = vec3(0.0, x, c);
    } else if (4.0 / 6.0 <= h && h < 5.0 / 6.0) {
        rgb = vec3(x, 0.0, c);
    } else {
        rgb = vec3(c, 0.0, x);
    }

    rgb += vec3(m, m, m);
    return rgb;
}

void main()
{
    float color = oPos;
    if(color < 0.1)
        color = 0.0;
    if(color < 1.6)
        color = 0.25;
    else
        color = color/50.0 + 0.6;

    vec3 rgb = hsv2rgb(color, 1.0, 1.0);
    if(oPos < 0.1)
        rgb = vec3(0.45f, 0.55f, 0.60f);
    if(oPos >= 99.0)
        rgb = vec3(1.0,1.0,1.0);
    FragColor = vec4(rgb, 1.0);
}

)";



typedef struct map{
    GLuint vbo;
    GLuint vao;
    GLuint size = -1;
} glMap;

typedef struct camera {
    float cameraX = 0.0f;
    float cameraY = 0.0f;
    float cameraZoom = 100;
} Camera;

static Camera cam;

void getMatrix(glm::mat4& view, glm::mat4& projection){
    view = glm::lookAt(
            glm::vec3(cam.cameraX, cam.cameraY, cam.cameraZoom), // camera
            glm::vec3(cam.cameraX, cam.cameraY, 0.0f), // look
            glm::vec3(0.0f, 1.0f, 0.0f)
    );
    projection = glm::perspective(glm::radians(45.0f), (float)640 / (float)480, 0.1f, 500.0f);
}

void fillMap(glMap& map){
    glGenVertexArrays(1, &map.vao);
    glBindVertexArray(map.vao);
    glGenBuffers(1, &map.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, map.vbo);
    glBindVertexArray(0);
}


void removeMap(glMap& map){
    glDeleteBuffers(1, &map.vbo);
    glDeleteVertexArrays(1, &map.vao);
}

GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    return shader;
}

GLuint returnProgram(){
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    GLint success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return shaderProgram;
}

glm::vec3 raycast(float mouseX, float mouseY, ImVec2 windowSize) {
    glm::mat4 view;
    glm::mat4 projection;
    getMatrix(view,projection);

    // Normalized Device Coordinates (NDC)
    float x = (2.0f * mouseX) / windowSize.x - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / windowSize.y;
    float z = 1.0f;
    glm::vec3 ray_nds = glm::vec3(x, y, z);

    // Clip Coordinates
    glm::vec4 ray_clip = glm::vec4(ray_nds, 1.0);

    // View Coordinates
    glm::vec4 ray_eye = glm::inverse(projection) * ray_clip;
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);

    // World Coordinates
    glm::vec3 ray_wor = glm::vec3(glm::inverse(view) * ray_eye);
    ray_wor = glm::normalize(ray_wor);

    return ray_wor;
}

glm::vec3 getIntersectionPointWithZPlane(glm::vec3 ray_direction, glm::vec3 ray_origin, float z_plane) {
    float t = (z_plane - ray_origin.z) / ray_direction.z;
    glm::vec3 intersection = ray_origin + t * ray_direction;
    return intersection;
}

#endif