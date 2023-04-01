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

#include "gltf.h"
#include "aixlog.hpp"
#include "image.h"
#include "math/vector.h"

namespace luc::inner {
std::vector<uint32_t> indices_from_prim(const tinygltf::Model& gmodel, const tinygltf::Primitive& gprim)
{
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
}

std::vector<math::float3> vertices_from_attributes(const tinygltf::Model& gmodel, const int accessor_key)
{
    const auto attrib_accessor = gmodel.accessors[accessor_key];
    const auto& buffer_view = gmodel.bufferViews[attrib_accessor.bufferView];
    const auto& buffer = gmodel.buffers[buffer_view.buffer];
    const auto *data_address = buffer.data.data() + buffer_view.byteOffset + attrib_accessor.byteOffset;
    const auto count = attrib_accessor.count;
    LOG(INFO) << "Position count: " << count << "\n";
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
                        vertices[i] = math::float3((float)v.x, (float)v.y, (float)v.z);
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
}

std::vector<math::float3> normals_from_attributes(const tinygltf::Model& gmodel, const int accessor_key, std::vector<uint32_t>& indices)
{
    const auto attrib_accessor = gmodel.accessors[accessor_key];
    const auto& buffer_view = gmodel.bufferViews[attrib_accessor.bufferView];
    const auto& buffer = gmodel.buffers[buffer_view.buffer];
    const auto *data_address = buffer.data.data() + buffer_view.byteOffset + attrib_accessor.byteOffset;
    const auto count = attrib_accessor.count;
    LOG(INFO) << "Normal count: " << count << "\n";
    std::vector<math::float3> normals(count);
    switch (attrib_accessor.type) {
        case TINYGLTF_TYPE_VEC3: {
            switch (attrib_accessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                    LOG(INFO) << "Type is FLOAT\n";

                    const auto *data = (math::float3 *)data_address;
                    for (size_t i = 0; i < count; i++)
                        normals[i] = data[i];
                } break;
                case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                    LOG(INFO) << "Type is DOUBLE\n";

                    const auto *data = (math::double3 *)data_address;
                    for (size_t i = 0; i < count; i++) {
                        const auto& v = data[i];
                        normals[i] = math::float3((float)v.x, (float)v.y, (float)v.z);
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
}

std::vector<math::float2> texcoords_from_attributes(const tinygltf::Model& gmodel, const int accessor_key, std::vector<uint32_t>& indices)
{
    const auto attrib_accessor = gmodel.accessors[accessor_key];
    const auto& buffer_view = gmodel.bufferViews[attrib_accessor.bufferView];
    const auto& buffer = gmodel.buffers[buffer_view.buffer];
    const auto *data_address = buffer.data.data() + buffer_view.byteOffset + attrib_accessor.byteOffset;
    const auto count = attrib_accessor.count;
    LOG(INFO) << "Texture coordinate count: " << count << "\n";
    std::vector<math::float2> texcoords(count);
    switch (attrib_accessor.type) {
        case TINYGLTF_TYPE_VEC2: {
            switch (attrib_accessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                    LOG(INFO) << "Type is FLOAT\n";

                    const auto *data = (math::float2 *)data_address;
                    for (size_t i = 0; i < count; i++)
                        texcoords[i] = data[i];
                } break;
                case TINYGLTF_COMPONENT_TYPE_DOUBLE: {
                    LOG(INFO) << "Type is DOUBLE\n";

                    const auto *data = (math::double2 *)data_address;
                    for (size_t i = 0; i < count; i++) {
                        const auto& v = data[i];
                        texcoords[i] = math::float2((float)v.x, (float)v.y);
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
}

luc::model::mesh process_mesh(const tinygltf::Model& gmodel, const tinygltf::Mesh& gmesh)
{
    LOG(INFO) << gmesh.name << "\n";
    luc::model::mesh mesh;
    for (const auto& gprim : gmesh.primitives) {
        luc::model::mesh::submesh smesh;
        smesh.material = gprim.material;
        smesh.indices = indices_from_prim(gmodel, gprim);
        switch (gprim.mode) {
            case TINYGLTF_MODE_TRIANGLES: {
                LOG(INFO) << "TRIANGLES\n";

                for (const auto& attribute : gprim.attributes) {
                    if (attribute.first == "POSITION")
                        smesh.vertices = std::move(vertices_from_attributes(gmodel, attribute.second));
                    if (attribute.first == "NORMAL")
                        smesh.normals = std::move(normals_from_attributes(gmodel, attribute.second, smesh.indices));
                    if (attribute.first == "TEXCOORD_0")
                        smesh.texcoords = std::move(texcoords_from_attributes(gmodel, attribute.second, smesh.indices));
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
}

luc::model load_gltf(std::filesystem::path& path)
{
    LOG(INFO) << path << "\n";

    tinygltf::Model gmodel;
    tinygltf::TinyGLTF ctx;
    ctx.SetStoreOriginalJSONForExtrasAndExtensions(true);

    std::string error, warn;
    bool ret = ctx.LoadBinaryFromFile(&gmodel, &error, &warn, path.c_str());
    if (!error.empty())
        LOG(ERROR) << error.c_str() << "\n";

    if (!warn.empty())
        LOG(WARNING) << warn.c_str() << "\n";

    luc::model model;
    if (!ret)
        return model;

    model.meshes.reserve(gmodel.meshes.size());
    for (const auto& gmesh : gmodel.meshes) {
        auto mesh = process_mesh(gmodel, gmesh);
        model.meshes.push_back(std::move(mesh));
    }

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
            const auto translation = math::translation(math::float3((float)gnode.translation[0], (float)gnode.translation[1], (float)gnode.translation[2]));
            instance.transform = math::mul(instance.transform, translation);
        }
        if (!gnode.scale.empty()) {
            const auto scale = math::scale<float, 4, 4>(math::float3((float)gnode.scale[0], (float)gnode.scale[1], (float)gnode.scale[2]));
            instance.transform = math::mul(instance.transform, scale);
        }
        if (!gnode.rotation.empty()) {
            const auto rot = math::quat_to_matrix(math::float4((float)gnode.rotation[0], (float)gnode.rotation[1], (float)gnode.rotation[2], (float)gnode.rotation[3]));
            instance.transform = math::mul(instance.transform, rot);
        }
        model.instances.push_back(instance);
    }
    for (const auto& tex : gmodel.textures) {
        if (tex.source > -1) {
            const auto& gimage = gmodel.images[tex.source];
            luc::texture<float, 3> image(gimage.width, gimage.height);
            switch (gimage.pixel_type) {
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
                    load_raw_into_image<uint8_t>(image, gimage.component, (void *)gimage.image.data());
                } break;
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                    load_raw_into_image<uint16_t>(image, gimage.component, (void *)gimage.image.data());
                } break;
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
                    load_raw_into_image<uint32_t>(image, gimage.component, (void *)gimage.image.data());
                } break;
                case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                    load_raw_into_image<float>(image, gimage.component, (void *)gimage.image.data());
                } break;
                default:
                    LOG(ERROR) << "unknown pixel type: " << gimage.mimeType << "(" << gimage.component << "x" << gimage.bits << "bits)\n";
                    break;
            }
            model.textures.push_back(std::move(image));
            LOG(INFO) << "texture!!!!\n";
        }
    }

    model.materials.reserve(gmodel.materials.size());
    for (const auto& gmat : gmodel.materials) {
        LOG(INFO) << "Material: " << gmat.name << "\n";
        luc::model::material material;
        if (gmat.emissiveTexture.index > -1)
            material.emission.texture = gmat.emissiveTexture.index;
        if (gmat.pbrMetallicRoughness.baseColorTexture.index > -1)
            material.albedo.texture = gmat.pbrMetallicRoughness.baseColorTexture.index;
        for (const auto& value : gmat.values)
            if (value.first == "baseColorFactor")
                material.albedo.c = math::float3((float)value.second.number_array[0], (float)value.second.number_array[1], (float)value.second.number_array[2]);
            else if (value.first == "baseColorTexture")
                LOG(INFO) << "baseColorTexture!!!!\n";
            else if (value.first == "metallicFactor")
                material.metallic.c = (float)value.second.number_value;
            else if (value.first == "roughnessFactor")
                material.roughness.c = (float)value.second.number_value;
            else
                LOG(ERROR) << "unknown material parameter: " << value.first << "\n";
        for (const auto& value : gmat.additionalValues)
            if (value.first == "emissiveFactor")
                material.emission.c = math::float3((float)value.second.number_array[0], (float)value.second.number_array[1], (float)value.second.number_array[2]);
            else if (value.first == "emissiveTexture")
                LOG(INFO) << "emissiveTexture!!!!\n";
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
                material.specular.c = math::float3((float)vspec.Get(0).GetNumberAsDouble(), (float)vspec.Get(1).GetNumberAsDouble(), (float)vspec.Get(2).GetNumberAsDouble());
            }
            else
                LOG(ERROR) << "unknown material extension: " << value.first << "\n";
        model.materials.push_back(material);
    }

    return model;
}
} // namespace luc::inner