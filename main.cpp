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

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <stdlib.h>
#include <stdio.h>

#include "argh/argh.h"
#include "aixlog.hpp"
#include "math/vector.h"
#include "camera.h"
#include "framebuffer.h"
#include "parallel_for.h"
#include "filesystem.h"
#include "scene.h"
#include "sampler.h"

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
MessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
    if (type == GL_DEBUG_TYPE_ERROR)
        LOG(ERROR) << "GL: " << message << "\n";
    else
        LOG(INFO) << "GL: " << message << "\n";
    return;
}

namespace luc {
math::float3 scene_light(const luc::scene& scene, sampler<float>& rng, const math::float3& ray_org, const math::float3& ray_dir);
} // namespace luc

int main(int argc, char *argv[])
{
    auto sink_cout = std::make_shared<AixLog::SinkCout>(AixLog::Severity::trace);
    auto sink_file = std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace, "logfile.log");
    AixLog::Log::init({ sink_cout, sink_file });

    const argh::parser cmdl(argc, argv);

    int width = 1, height = 1, spp = 1, camera_id = 0;
    if (!(cmdl("w") >> width))
        LOG(WARNING) << "No image width provided! Defaulting to: " << width << " (-w=N)!\n";
    if (!(cmdl("h") >> height))
        LOG(WARNING) << "No image height provided! Defaulting to: " << height << " (-h=N)!\n";
    if (!(cmdl("s") >> spp))
        LOG(WARNING) << "No samples per pixel provided! Defaulting to: " << spp << " (-s=N)!\n";
    spp = std::max(spp, 1);
    if (!(cmdl("c") >> camera_id))
        LOG(WARNING) << "No camera selected! Defaulting to: " << camera_id << " (-c=N)!\n";

    const std::vector<std::string> positional(std::begin(cmdl.pos_args()) + 1, std::end(cmdl.pos_args()));
    if (positional.empty()) {
        LOG(ERROR) << "No input files!\n";
        return 1;
    }
    std::vector<std::string> input_files;
    input_files.reserve(positional.size());
    auto working_directory = std::filesystem::current_path();
    for (const auto& pos : positional)
        input_files.emplace_back(working_directory / std::filesystem::path(pos));

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
        scene.cameras.emplace_back(eye, -eye, up, (float)width / (float)height, fov_x);
    }

    luc::framebuffer<float> framebuffer(width, height);
    bool done = false;
    abort_token aborter;
    std::mutex preview_mutex;
    std::vector<work_range<int> *> active_ranges;
    auto render_worker = [&]() {
        int frame = 0;
        while (!aborter.aborted) {
            std::random_device rd_outer;
            std::mt19937 rng_outer(rd_outer());
            const auto sample_count = spp;
            const auto samples_sqrt = (int)std::ceil(std::sqrt(sample_count));
            const auto sample_count_true = samples_sqrt * samples_sqrt;
            std::vector<math::float4> samples;
            for (size_t v = 0; v < samples_sqrt; v++)
                for (size_t u = 0; u < samples_sqrt; u++)
                    samples.emplace_back(
                      (((float)u + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_sqrt) - .5f,
                      (((float)v + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)samples_sqrt) - .5f,
                      1.f / (float)sample_count_true,
                      ((float)samples.size() + std::uniform_real_distribution<float>(0.f, 1.f)(rng_outer)) / (float)sample_count_true);
            auto start = std::chrono::steady_clock::now();
            LOG(INFO) << "frame(" << frame << ")";
            auto tile_func = [&](const work_block<int>& block) {
                std::random_device rd_inner;
                std::mt19937 rng_inner(rd_inner());
                static thread_local work_range<int> *active_range = nullptr;
                if (!active_range) {
                    const std::scoped_lock preview_lock(preview_mutex);
                    active_range = new work_range<int>(block.tile);
                    active_ranges.push_back(active_range);
                }
                else {
                    *active_range = block.tile;
                }
                const float current_spp(frame + 1);
                const float inv_next_spp(1. / double(frame + 2));
                auto item_func = [&](const int x, const int y, auto&& transform) {
                    luc::sampler<float> rng((int)std::sqrt(samples.size()) + 1, rng_inner);
                    math::vector<float, 3> combined = framebuffer.combined.get(x, y) * current_spp;
                    math::vector<float, 3> albedo = framebuffer.albedo.get(x, y) * current_spp;
                    math::vector<float, 3> shading_normal = framebuffer.shading_normal.get(x, y) * current_spp;
                    math::vector<float, 3> geometry_normal = framebuffer.geometry_normal.get(x, y) * current_spp;
                    math::vector<float, 3> position = framebuffer.position.get(x, y) * current_spp;
                    math::vector<float, 3> emission = framebuffer.emission.get(x, y) * current_spp;
                    math::vector<float, 3> specular = framebuffer.specular.get(x, y) * current_spp;
                    math::vector<float, 1> metallic = framebuffer.metallic.get(x, y) * current_spp;
                    math::vector<float, 1> roughness = framebuffer.roughness.get(x, y) * current_spp;
                    math::vector<float, 1> ior = framebuffer.ior.get(x, y) * current_spp;
                    math::vector<float, 1> transmission = framebuffer.transmission.get(x, y) * current_spp;
                    for (auto& sample : samples) {
                        math::float3 ray_org, ray_dir;
                        std::tie(ray_org, ray_dir) = scene.cameras[camera_id].ray(transform(sample.uv));
                        const auto light = luc::scene_light(scene, rng, ray_org, ray_dir);
                        if (const auto hit = scene.intersect(ray_org, ray_dir)) {
                            combined += light * sample.z;
                            albedo += hit->albedo * sample.z;
                            shading_normal += hit->normal_s * sample.z;
                            geometry_normal += hit->normal_g * sample.z;
                            position += hit->position * sample.z;
                            if (hit->emission.has_value())
                                emission += *hit->emission * sample.z;
                            specular += *hit->specular * sample.z;
                            metallic += *hit->metallic * sample.z;
                            roughness += *hit->roughness * sample.z;
                            ior += *hit->ior * sample.z;
                            transmission += *hit->transmission * sample.z;
                        }
                    }
                    framebuffer.combined.set(x, y, combined * inv_next_spp);
                    framebuffer.albedo.set(x, y, albedo * inv_next_spp);
                    framebuffer.shading_normal.set(x, y, shading_normal * inv_next_spp);
                    framebuffer.geometry_normal.set(x, y, geometry_normal * inv_next_spp);
                    framebuffer.position.set(x, y, position * inv_next_spp);
                    framebuffer.emission.set(x, y, emission * inv_next_spp);
                    framebuffer.specular.set(x, y, specular * inv_next_spp);
                    framebuffer.metallic.set(x, y, metallic * inv_next_spp);
                    framebuffer.roughness.set(x, y, roughness * inv_next_spp);
                    framebuffer.ior.set(x, y, ior * inv_next_spp);
                    framebuffer.transmission.set(x, y, transmission * inv_next_spp);
                };
                iterate_over_tile(block, aborter, item_func);
            };
            const auto domain = generate_parallel_for_domain_rows(0, width, 0, height);
            parallel_for<int, true>(domain, tile_func, aborter);
            frame++;
            const auto end = std::chrono::steady_clock::now();
            const std::chrono::duration<double> elapsed_seconds = end - start;
            LOG(INFO) << " in: " << elapsed_seconds.count() << "s\n";
            for (auto *range : active_ranges)
                delete range;
            active_ranges.clear();
            luc::save_framebuffer_exr(framebuffer, input_files.front());
        }
        done = true;
    };
    const std::thread render_thread(render_worker);

    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) {
        LOG(ERROR) << "glfwInit failed\n";
        return 1;
    }

    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    auto *window = glfwCreateWindow(width, height, "lucray", NULL, NULL);
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
    glDebugMessageCallback(MessageCallback, nullptr);
    glfwSwapInterval(1);

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
    auto color_mul_loc = glGetUniformLocation(program, "color_mul");
    auto color_lum_loc = glGetUniformLocation(program, "color_lum");
    auto texture_r_loc = glGetUniformLocation(program, "texture_r");
    auto texture_g_loc = glGetUniformLocation(program, "texture_g");
    auto texture_b_loc = glGetUniformLocation(program, "texture_b");

    auto gl_create_texture = [&]() {
        GLuint id;
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        return id;
    };
    GLuint texture_r = gl_create_texture();
    GLuint texture_g = gl_create_texture();
    GLuint texture_b = gl_create_texture();

    GLuint dummy_vertex_array_id;
    glGenVertexArrays(1, &dummy_vertex_array_id);
    while (!glfwWindowShouldClose(window)) {
        int f_width, f_height;
        glfwGetWindowSize(window, &f_width, &f_height);
        glViewport(0, 0, f_width, f_height);
        glClear(GL_COLOR_BUFFER_BIT);

        glBindTexture(GL_TEXTURE_2D, texture_r);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, framebuffer.combined.channels[0].pixels.data());
        glBindTexture(GL_TEXTURE_2D, texture_g);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, framebuffer.combined.channels[1].pixels.data());
        glBindTexture(GL_TEXTURE_2D, texture_b);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, framebuffer.combined.channels[2].pixels.data());

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
    return 0;
}
