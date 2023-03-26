#pragma once
#include "tinyobjloader/tiny_obj_loader.h"
#include <tuple>
#include <vector>
#include <string>
#include "aixlog.hpp"
#include "math/vector.h"
#include <filesystem>

auto load_model(const std::string& inputfile)
{
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = std::filesystem::path(inputfile).parent_path();

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(inputfile, reader_config))
    {
        if (!reader.Error().empty())
            LOG(ERROR) << "TinyObjReader: " << reader.Error() << std::endl;
        else
            LOG(ERROR) << "TinyObjReader: Unknown parse error!" << std::endl;
    }
    if (!reader.Warning().empty())
        LOG(WARNING) << "TinyObjReader: " << reader.Warning() << std::endl;
    LOG(INFO) << "Loaded model: " << inputfile << std::endl;
    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();
    const auto& materials = reader.GetMaterials();

    std::vector<math::float3> vertices;
    vertices.reserve(attrib.vertices.size() / 3);
    for (size_t i = 0; i < attrib.vertices.size(); i += 3)
    {
        const auto& a = attrib.vertices[i + 0];
        const auto& b = attrib.vertices[i + 1];
        const auto& c = attrib.vertices[i + 2];
        vertices.emplace_back(a, b, c);
    }
    std::vector<math::float3> normals;
    normals.reserve(attrib.normals.size() / 3);
    for (size_t i = 0; i < attrib.normals.size(); i += 3)
    {
        const auto& a = attrib.normals[i + 0];
        const auto& b = attrib.normals[i + 1];
        const auto& c = attrib.normals[i + 2];
        normals.emplace_back(a, b, c);
    }
    std::vector<std::tuple<int, int, int>> triangles;
    for (const auto& shape : shapes)
    {
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
        {
            const auto& a = shape.mesh.indices[f * 3 + 0].vertex_index;
            const auto& b = shape.mesh.indices[f * 3 + 1].vertex_index;
            const auto& c = shape.mesh.indices[f * 3 + 2].vertex_index;
            triangles.emplace_back(a, b, c);
        }
    }
    return std::make_tuple(triangles, vertices, normals);
}