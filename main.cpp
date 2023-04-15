// MIT License
//
// Copyright (c) 2022 Robin Lind
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <filesystem>
#include "argh/argh.h"
#include "aixlog.hpp"
#include "math/vector.h"
#include "math/utils.h"
#include "camera.h"
#include "framebuffer.h"
#include "parallel_for.h"
#include "filesystem.h"
#include "scene.h"
#include "sampler.h"
#include "renderer.h"

static void error_callback(int error, const char *description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void GLAPIENTRY
GL_message(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
    if (type == GL_DEBUG_TYPE_ERROR)
        LOG(ERROR) << "GL: " << message << "\n";
    // else
    //     LOG(INFO) << "GL: " << message << "\n";
}

int main(int argc, char *argv[])
{
    auto sink_cout = std::make_shared<AixLog::SinkCout>(AixLog::Severity::trace);
    auto sink_file = std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace, "logfile.log");
    AixLog::Log::init({ sink_cout, sink_file });
    const argh::parser cmdl(argc, argv);

    luc::settings s;
    if (!(cmdl("width") >> s.width))
        LOG(WARNING) << "No image width provided! Defaulting to: " << s.width << " (-w=N)!\n";
    if (!(cmdl("height") >> s.height))
        LOG(WARNING) << "No image height provided! Defaulting to: " << s.height << " (-h=N)!\n";
    if ((cmdl("imagescale") >> s.image_scale)) {
        s.width *= s.image_scale;
        s.height *= s.image_scale;
    }
    if (!(cmdl("spp") >> s.samples_per_pixel))
        LOG(WARNING) << "No samples per pixel provided! Defaulting to: " << s.samples_per_pixel << " (-s=N)!\n";
    s.samples_per_pixel = std::max(s.samples_per_pixel, 1);
    cmdl("spl") >> s.samples_per_loop;
    s.samples_per_loop = std::max(std::min(s.samples_per_pixel, s.samples_per_loop), std::min(s.samples_per_pixel, 16));
    if (!(cmdl("camera") >> s.camera_id))
        LOG(WARNING) << "No camera selected! Defaulting to: " << s.camera_id << " (-c=N)!\n";
    if (!(cmdl("depth") >> s.max_depth))
        LOG(WARNING) << "No max depth selected! Defaulting to: " << s.max_depth << " (-d=N)!\n";

    const std::vector<std::string> positional(std::begin(cmdl.pos_args()) + 1, std::end(cmdl.pos_args()));
    if (positional.empty()) {
        LOG(ERROR) << "No input files!\n";
        return 1;
    }
    std::vector<std::string> input_files;
    input_files.reserve(positional.size());
    const auto working_directory = std::filesystem::current_path();
    for (const auto& pos : positional) {
        const auto path = working_directory / std::filesystem::path(pos);
        if (std::filesystem::exists(path))
            input_files.emplace_back(path);
        else
            LOG(ERROR) << "File does not exist! (" << path << ")\n";
    }
    if (!(cmdl("output") >> s.image_name))
        s.image_name = input_files.front();

    luc::scene scene;
    for (auto& file_path : input_files)
        scene.append_model(luc::load_file(file_path));
    scene.commit();

    if (scene.cameras.empty()) {
        LOG(ERROR) << "No cameras in scene!\n";
        const float fov_x = 43.f * 0.0174532793f;
        const math::float3 root_max(scene.accelerator.get_root().get_bbox().max.values);
        const math::float3 eye = root_max * 2.f;
        const math::float3 up(0.f, 1.f, 0.f);
        scene.cameras.emplace_back(eye, -eye, up, (float)s.width / (float)s.height, fov_x);
    }
    if (s.camera_id < 0 || s.camera_id >= scene.cameras.size()) {
        LOG(ERROR) << "Invalid camera index! (" << s.camera_id << ") setting index to 0\n";
        s.camera_id = 0;
    }

    luc::framebuffer<float> fb(s.width, s.height);
    abort_token aborter;
    auto render_worker = [&]() {
        luc::render(s, fb, aborter, scene);
    };
    std::thread render_thread(render_worker);

    glfwSetErrorCallback(error_callback);
    if (!glfwInit()) {
        LOG(ERROR) << "glfwInit failed\n";
        return 1;
    }
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    auto *window = glfwCreateWindow(s.width, s.height, "lucray", NULL, NULL);
    if (!window) {
        LOG(ERROR) << "glfwCreateWindow failed\n";
        glfwTerminate();
        return 1;
    }
    glfwSetKeyCallback(window, key_callback);
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        if (!gladLoadGL()) {
            LOG(ERROR) << "LoadGL failed\n";
            return 1;
        }
    }
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(GL_message, nullptr);
    glfwSwapInterval(0);
    const char *vertex_shader_text =
      "#version 330 core\n"
      "const vec2 vertices[3] = vec2[3](vec2(-1., 1.), vec2(3., 1.), vec2(-1., -3.));\n"
      "const vec2 uvs[3] = vec2[3](vec2(0., 0.), vec2(2., 0.), vec2(0., 2.));\n"
      "out vec2 frag_uv;\n"
      "void main()\n"
      "{\n"
      "    frag_uv = uvs[gl_VertexID];\n"
      "    gl_Position = vec4(vertices[gl_VertexID], 0., 1.);\n"
      "}\n";
    const char *fragment_shader_text =
      "#version 330 core\n"
      "in vec2 frag_uv;\n"
      "uniform sampler2D texture_r;\n"
      "uniform sampler2D texture_g;\n"
      "uniform sampler2D texture_b;\n"
      "uniform vec3 color_mul;\n"
      "uniform vec3 color_lum;\n"
      "out vec4 output_color;\n"
      "float sRGB_gamma(float n)\n"
      "{\n"
      "	return n < 0.0031308f ? (n * 12.92f) : ((1.055f * pow(n, 1.0f / 2.4f)) - 0.055f);\n"
      "}\n"
      "vec3 sRGB_gamma3f(vec3 n)\n"
      "{\n"
      "	return vec3(\n"
      "	 sRGB_gamma(n.x),\n"
      "	 sRGB_gamma(n.y),\n"
      "	 sRGB_gamma(n.z));\n"
      "}\n"
      "void main()\n"
      "{\n"
      "    float red = texture(texture_r, frag_uv).r;\n"
      "    float green = texture(texture_g, frag_uv).r;\n"
      "    float blue = texture(texture_b, frag_uv).r;\n"
      "    vec3 rgb_image = vec3(red, green, blue) * color_mul;\n"
      "    vec3 lum_image = vec3(red, red, red) * color_lum;\n"
      "    vec3 color = rgb_image + lum_image;\n"
      "    vec3 srgb = sRGB_gamma3f(color);\n"
      "    output_color = vec4(srgb, 1.);\n"
      "}\n";
    auto vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, nullptr);
    glCompileShader(vertex_shader);
    auto fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, nullptr);
    glCompileShader(fragment_shader);
    auto program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    math::float3 color_mul(1);
    math::float3 color_lum(0, 0, 0);
    const auto color_mul_loc = glGetUniformLocation(program, "color_mul");
    const auto color_lum_loc = glGetUniformLocation(program, "color_lum");
    const auto texture_r_loc = glGetUniformLocation(program, "texture_r");
    const auto texture_g_loc = glGetUniformLocation(program, "texture_g");
    const auto texture_b_loc = glGetUniformLocation(program, "texture_b");
    auto gl_create_texture = [&]() {
        GLuint id;
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, s.width, s.height, 0, GL_RED, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        return id;
    };
    const auto texture_r = gl_create_texture();
    const auto texture_g = gl_create_texture();
    const auto texture_b = gl_create_texture();
    GLuint dummy_vertex_array_id;
    glGenVertexArrays(1, &dummy_vertex_array_id);
    while (!glfwWindowShouldClose(window) && !aborter.aborted) {
        int f_width, f_height;
        glfwGetWindowSize(window, &f_width, &f_height);
        glViewport(0, 0, f_width, f_height);
        glClear(GL_COLOR_BUFFER_BIT);
        glBindTexture(GL_TEXTURE_2D, texture_r);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, s.width, s.height, 0, GL_RED, GL_FLOAT, fb.combined.channels[0].pixels.data());
        glBindTexture(GL_TEXTURE_2D, texture_g);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, s.width, s.height, 0, GL_RED, GL_FLOAT, fb.combined.channels[1].pixels.data());
        glBindTexture(GL_TEXTURE_2D, texture_b);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, s.width, s.height, 0, GL_RED, GL_FLOAT, fb.combined.channels[2].pixels.data());
        glUseProgram(program);
        glUniform3fv(color_mul_loc, 1, color_mul.values.data());
        glUniform3fv(color_lum_loc, 1, color_lum.values.data());
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_r);
        glUniform1i(texture_r_loc, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texture_g);
        glUniform1i(texture_g_loc, 1);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, texture_b);
        glUniform1i(texture_b_loc, 2);
        glBindVertexArray(dummy_vertex_array_id);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    aborter.abort();
    render_thread.join();
    return 0;
}
