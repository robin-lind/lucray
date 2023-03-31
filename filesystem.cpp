// MIT License
//
// Copyright (c) 2023 Robin Lind
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

#include "filesystem.h"
#include "model.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "math/bounds.h"
#include "aixlog.hpp"
#include "tinygltf/tiny_gltf.h"
#include <cstdint>
#include <vector>

namespace luc {

namespace inner {
luc::model load_obj(std::filesystem::path& path)
{
    return luc::model();
}

luc::model load_gltf(std::filesystem::path& path)
{
    LOG(INFO) << path << "\n";
    ;

    tinygltf::Model gmodel;
    tinygltf::TinyGLTF ctx;
    ctx.SetStoreOriginalJSONForExtrasAndExtensions(true);

    std::string error, warn;
    bool ret = ctx.LoadBinaryFromFile(&gmodel, &error, &warn, path.c_str());
    if (!error.empty())
        LOG(ERROR) << error.c_str() << "\n";
    ;
    if (!warn.empty())
        LOG(WARNING) << warn.c_str() << "\n";
    ;

    luc::model model;
    if (!ret)
        return model;
    auto indices_from_prim = [&](const tinygltf::Primitive& gprim) {
        const auto& indices_accessor = gmodel.accessors[gprim.indices];
        const auto& buffer_view = gmodel.bufferViews[indices_accessor.bufferView];
        const auto& buffer = gmodel.buffers[buffer_view.buffer];
        const auto *data_address = buffer.data.data() + buffer_view.byteOffset + indices_accessor.byteOffset;
        const auto count = indices_accessor.count;
        std::vector<uint32_t> indices(count);
        switch (indices_accessor.componentType) {
            case TINYGLTF_COMPONENT_TYPE_BYTE: {
                const auto *data = (int8_t *)data_address;
                for (size_t i = 0; i < count; i++)
                    indices[i] = (uint8_t)data[i];
            } break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
                const auto *data = (uint8_t *)data_address;
                for (size_t i = 0; i < count; i++)
                    indices[i] = data[i];
            } break;
            case TINYGLTF_COMPONENT_TYPE_SHORT: {
                const auto *data = (int16_t *)data_address;
                for (size_t i = 0; i < count; i++)
                    indices[i] = data[i];
            } break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                const auto *data = (uint16_t *)data_address;
                for (size_t i = 0; i < count; i++)
                    indices[i] = data[i];
            } break;
            case TINYGLTF_COMPONENT_TYPE_INT: {
                const auto *data = (int32_t *)data_address;
                for (size_t i = 0; i < count; i++)
                    indices[i] = data[i];
            } break;
            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
                const auto *data = (uint32_t *)data_address;
                for (size_t i = 0; i < count; i++)
                    indices[i] = data[i];
            } break;
            default:
                break;
        }
        return indices;
    };
    auto vertices_from_attributes = [&](const int accessor_key) {
        const auto attrib_accessor = gmodel.accessors[accessor_key];
        const auto& buffer_view = gmodel.bufferViews[attrib_accessor.bufferView];
        const auto& buffer = gmodel.buffers[buffer_view.buffer];
        const auto *data_address = buffer.data.data() + buffer_view.byteOffset + attrib_accessor.byteOffset;
        const auto count = attrib_accessor.count;
        LOG(INFO) << "Position count: " << count << "\n";
        ;
        std::vector<math::float3> vertices(count);
        switch (attrib_accessor.type) {
            case TINYGLTF_TYPE_VEC3: {
                switch (attrib_accessor.componentType) {
                    case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                        LOG(INFO) << "Type is FLOAT\n";

                        const auto *data = (math::float3 *)data_address;
                        for (size_t i = 0; i < count; i++)
                            vertices[i] = data[i];
                    } break;
                    case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                        LOG(INFO) << "Type is DOUBLE\n";

                        const auto *data = (math::double3 *)data_address;
                        for (size_t i = 0; i < count; i++) {
                            const auto& v = data[i];
                            vertices[i] = math::float3(v.x, v.y, v.z);
                        }
                    } break;
                    default:
                        LOG(ERROR) << "Vertex position not float or double\n";
                        break;
                }
            } break;
            default:
                LOG(ERROR) << "Vertex position not 3D\n";
                break;
        }
        return vertices;
    };
    auto normals_from_attributes = [&](const int accessor_key, std::vector<uint32_t>& indices) {
        const auto attrib_accessor = gmodel.accessors[accessor_key];
        const auto& buffer_view = gmodel.bufferViews[attrib_accessor.bufferView];
        const auto& buffer = gmodel.buffers[buffer_view.buffer];
        const auto *data_address = buffer.data.data() + buffer_view.byteOffset + attrib_accessor.byteOffset;
        const auto count = attrib_accessor.count;
        LOG(INFO) << "Normal count: " << count << "\n";
        ;
        std::vector<math::float3> normals(count);
        switch (attrib_accessor.type) {
            case TINYGLTF_TYPE_VEC3: {
                switch (attrib_accessor.componentType) {
                    case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                        LOG(INFO) << "Type is FLOAT\n";

                        const auto *data = (math::float3 *)data_address;
                        const auto t_count = indices.size() / 3;
                        for (size_t i = 0; i < t_count; ++i) {
                            const auto f0 = indices[3 * i + 0];
                            const auto f1 = indices[3 * i + 1];
                            const auto f2 = indices[3 * i + 2];

                            const auto n0 = data[f0];
                            const auto n1 = data[f1];
                            const auto n2 = data[f2];

                            normals[i + 0] = math::float3(n0.x, n0.y, n0.z);
                            normals[i + 1] = math::float3(n1.x, n1.y, n1.z);
                            normals[i + 2] = math::float3(n2.x, n2.y, n2.z);
                        }
                    } break;
                    case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                        LOG(INFO) << "Type is DOUBLE\n";

                        const auto *data = (math::double3 *)data_address;
                        const auto t_count = indices.size() / 3;
                        for (size_t i = 0; i < t_count; ++i) {
                            const auto f0 = indices[3 * i + 0];
                            const auto f1 = indices[3 * i + 1];
                            const auto f2 = indices[3 * i + 2];

                            const auto n0 = data[f0];
                            const auto n1 = data[f1];
                            const auto n2 = data[f2];

                            normals[i + 0] = math::float3(n0.x, n0.y, n0.z);
                            normals[i + 1] = math::float3(n1.x, n1.y, n1.z);
                            normals[i + 2] = math::float3(n2.x, n2.y, n2.z);
                        }
                    } break;
                    default:
                        LOG(ERROR) << "Vertex normals not float or double\n";
                        break;
                }
            } break;
            default:
                LOG(ERROR) << "Vertex normals not 3D\n";
                break;
        }
        return normals;
    };
    auto texcoords_from_attributes = [&](const int accessor_key, std::vector<uint32_t>& indices) {
        const auto attrib_accessor = gmodel.accessors[accessor_key];
        const auto& buffer_view = gmodel.bufferViews[attrib_accessor.bufferView];
        const auto& buffer = gmodel.buffers[buffer_view.buffer];
        const auto *data_address = buffer.data.data() + buffer_view.byteOffset + attrib_accessor.byteOffset;
        const auto count = attrib_accessor.count;
        LOG(INFO) << "Texture coordinate count: " << count << "\n";
        ;
        std::vector<math::float2> texcoords(count);
        switch (attrib_accessor.type) {
            case TINYGLTF_TYPE_VEC2: {
                switch (attrib_accessor.componentType) {
                    case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                        LOG(INFO) << "Type is FLOAT\n";

                        const auto *data = (math::float2 *)data_address;
                        const auto t_count = indices.size() / 3;
                        for (size_t i = 0; i < t_count; ++i) {
                            const auto f0 = indices[3 * i + 0];
                            const auto f1 = indices[3 * i + 1];
                            const auto f2 = indices[3 * i + 2];

                            const auto uv0 = data[f0];
                            const auto uv1 = data[f1];
                            const auto uv2 = data[f2];

                            texcoords[i + 0] = math::float2(uv0.x, uv0.y);
                            texcoords[i + 1] = math::float2(uv1.x, uv1.y);
                            texcoords[i + 2] = math::float2(uv2.x, uv2.y);
                        }
                    } break;
                    case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                        LOG(INFO) << "Type is DOUBLE\n";

                        const auto *data = (math::double2 *)data_address;
                        const auto t_count = indices.size() / 3;
                        for (size_t i = 0; i < t_count; ++i) {
                            const auto f0 = indices[3 * i + 0];
                            const auto f1 = indices[3 * i + 1];
                            const auto f2 = indices[3 * i + 2];

                            const auto uv0 = data[f0];
                            const auto uv1 = data[f1];
                            const auto uv2 = data[f2];

                            texcoords[i + 0] = math::float2(uv0.x, uv0.y);
                            texcoords[i + 1] = math::float2(uv1.x, uv1.y);
                            texcoords[i + 2] = math::float2(uv2.x, uv2.y);
                        }
                    } break;
                    default:
                        LOG(ERROR) << "Vertex UVs not float or double\n";
                        break;
                }
            } break;
            default:
                LOG(ERROR) << "Vertex UVs not 2D\n";
                break;
        }
        return texcoords;
    };
    auto process_mesh = [&](const tinygltf::Mesh& gmesh) {
        LOG(INFO) << gmesh.name << "\n";
        luc::model::mesh mesh;
        for (const auto& gprim : gmesh.primitives) {
            luc::model::mesh::submesh smesh;
            smesh.material = gprim.material;
            smesh.indices = indices_from_prim(gprim);
            switch (gprim.mode) {
                // case TINYGLTF_MODE_TRIANGLE_FAN: {
                //     LOG(INFO) << "TRIANGLE_FAN\n";
                //     auto fan = std::move(indices);
                //     indices.clear();
                //     for (size_t i{ 2 }; i < fan.size(); ++i) {
                //         indices.push_back(fan[0]);
                //         indices.push_back(fan[i - 1]);
                //         indices.push_back(fan[i]);
                //     }
                // }
                // case TINYGLTF_MODE_TRIANGLE_STRIP: {
                //     LOG(INFO) << "TRIANGLE_STRIP\n";
                //     auto fan = std::move(indices);
                //     indices.clear();
                //     for (size_t i{ 2 }; i < fan.size(); ++i) {
                //         indices.push_back(fan[i - 2]);
                //         indices.push_back(fan[i - 1]);
                //         indices.push_back(fan[i]);
                //     }
                // }
                case TINYGLTF_MODE_TRIANGLES: {
                    LOG(INFO) << "TRIANGLES\n";

                    for (const auto& attribute : gprim.attributes) {
                        if (attribute.first == "POSITION")
                            smesh.vertices = vertices_from_attributes(attribute.second);
                        if (attribute.first == "NORMAL")
                            smesh.normals = normals_from_attributes(attribute.second, smesh.indices);
                        if (attribute.first == "TEXCOORD_0")
                            smesh.texcoords = texcoords_from_attributes(attribute.second, smesh.indices);
                    }
                } break;
                case TINYGLTF_MODE_POINTS:
                case TINYGLTF_MODE_LINE:
                case TINYGLTF_MODE_LINE_LOOP:
                    LOG(ERROR) << "primitive is not triangle based, ignoring\n";
                    break;
                default:
                    LOG(ERROR) << "primitive mode not implemented\n";
                    break;
            }
            mesh.meshes.push_back(smesh);
        }
        return mesh;
    };
    model.meshes.reserve(gmodel.meshes.size());
    for (const auto& gmesh : gmodel.meshes)
        model.meshes.push_back(process_mesh(gmesh));

    model.instances.reserve(gmodel.nodes.size());
    for (const auto& gnode : gmodel.nodes) {
        LOG(INFO) << "Instance: " << gnode.name << "\n";
        luc::model::instance instance;
        if (gnode.mesh > -1) {
            instance.type = luc::model::instance_type::mesh_instance;
            instance.id = gnode.mesh;
        }
        else if (gnode.camera > -1) {
            instance.type = luc::model::instance_type::camera_instance;
            instance.id = gnode.camera;
        }
        else {
            LOG(ERROR) << "unknown instance type\n";
        }
        if (!gnode.translation.empty()) {
            const auto translation = math::translation(math::float3(gnode.translation[0], gnode.translation[1], gnode.translation[2]));
            instance.transform = math::mul(instance.transform, translation);
        }
        if (!gnode.scale.empty()) {
            const auto scale = math::scale<float,4,4>(math::float3(gnode.scale[0], gnode.scale[1], gnode.scale[2]));
            instance.transform = math::mul(instance.transform, scale);
        }
        if (!gnode.rotation.empty()) {
            const auto rot = math::quat_to_matrix(math::float4(gnode.rotation[0], gnode.rotation[1], gnode.rotation[2], gnode.rotation[3]));
            instance.transform = math::mul(instance.transform, rot);
        }
        model.instances.push_back(instance);
    }

    model.materials.reserve(gmodel.materials.size());
    for (const auto& gmat : gmodel.materials) {
        LOG(INFO) << "Material: " << gmat.name << "\n";
        luc::model::material material;
        for (const auto& value : gmat.values)
            if (value.first == "baseColorFactor")
                material.albedo.c = math::float3(value.second.number_array[0], value.second.number_array[1], value.second.number_array[2]);
            else if (value.first == "metallicFactor")
                material.metallic.c = (float)value.second.number_value;
            else if (value.first == "roughnessFactor")
                material.roughness.c = (float)value.second.number_value;
            else
                LOG(ERROR) << "unknown material parameter: " << value.first << "\n";
        for (const auto& value : gmat.additionalValues)
            if (value.first == "emissiveFactor")
                material.emission.c = math::float3(value.second.number_array[0], value.second.number_array[1], value.second.number_array[2]);
            else if (value.first == "doubleSided")
                LOG(INFO) << "doubleSided is ignored\n";
            else
                LOG(ERROR) << "unknown material parameter: " << value.first << "\n";
        for (const auto& value : gmat.extensions)
            if (value.first == "KHR_materials_emissive_strength")
                material.emissive_strength = (float)value.second.Get("emissiveStrength").GetNumberAsDouble();
            else if (value.first == "KHR_materials_ior")
                material.ior.c = (float)value.second.Get("ior").GetNumberAsDouble();
            else if (value.first == "KHR_materials_transmission")
                material.transmission.c = (float)value.second.Get("transmissionFactor").GetNumberAsDouble();
            else if (value.first == "KHR_materials_specular") {
                const auto vspec = value.second.Get("specularColorFactor");
                material.specular.c = math::float3(vspec.Get(0).GetNumberAsDouble(), vspec.Get(1).GetNumberAsDouble(), vspec.Get(2).GetNumberAsDouble());
            }
            else
                LOG(ERROR) << "unknown material extension: " << value.first << "\n";
        model.materials.push_back(material);
    }

    return model;
}
} // namespace inner

luc::model load_file(std::filesystem::path path)
{
    const auto ext = path.extension();
    luc::model model;
    if (ext == ".obj")
        model = inner::load_obj(path);
    else if (ext == ".glb" || ext == ".gltf")
        model = inner::load_gltf(path);
    return model;
}
} // namespace luc
