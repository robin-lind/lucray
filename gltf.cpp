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
#include "tinygltf/tiny_gltf.h"

namespace luc::inner {
template<typename T, typename S, size_t N>
auto get_component_list(const tinygltf::Model& gmodel, const tinygltf::Accessor& accessor)
{
    if constexpr (N == 1) {
        const auto& buffer_view = gmodel.bufferViews[accessor.bufferView];
        const auto& buffer = gmodel.buffers[buffer_view.buffer];
        const auto *ptr = buffer.data.data() + buffer_view.byteOffset + accessor.byteOffset;
        std::vector<T> values(accessor.count);
        const auto *data = (S *)ptr;
        for (size_t i = 0; i < accessor.count; i++)
            values[i] = static_cast<T>(data[i]);
        return values;
    }
    else {
        const auto& buffer_view = gmodel.bufferViews[accessor.bufferView];
        const auto& buffer = gmodel.buffers[buffer_view.buffer];
        const auto *ptr = buffer.data.data() + buffer_view.byteOffset + accessor.byteOffset;
        std::vector<math::vector<T, N>> values(accessor.count);
        const auto *data = (math::vector<S, N> *)ptr;
        for (size_t i = 0; i < accessor.count; i++) {
            values[i] = [&]<std::size_t... I>(std::index_sequence<I...>)
            {
                return math::vector<T, N>(static_cast<T>(std::get<I>(data[i].values))...);
            }
            (std::make_index_sequence<N>{});
        }
        return values;
    }
}

template<typename T, size_t N>
auto get_component(const tinygltf::Model& gmodel, int accessor_key)
{
    const auto& accessor = gmodel.accessors[accessor_key];
    switch (accessor.componentType) {
        case TINYGLTF_COMPONENT_TYPE_BYTE:
            return get_component_list<T, int8_t, N>(gmodel, accessor);
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
            return get_component_list<T, uint8_t, N>(gmodel, accessor);
        case TINYGLTF_COMPONENT_TYPE_SHORT:
            return get_component_list<T, int16_t, N>(gmodel, accessor);
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
            return get_component_list<T, uint16_t, N>(gmodel, accessor);
        case TINYGLTF_COMPONENT_TYPE_INT:
            return get_component_list<T, int32_t, N>(gmodel, accessor);
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
            return get_component_list<T, uint32_t, N>(gmodel, accessor);
        case TINYGLTF_COMPONENT_TYPE_FLOAT:
            return get_component_list<T, float, N>(gmodel, accessor);
        case TINYGLTF_COMPONENT_TYPE_DOUBLE:
            return get_component_list<T, double, N>(gmodel, accessor);
    }
    LOG(ERROR) << "unknown component type!\n";
    if constexpr (N == 1) {
        std::vector<T> values(accessor.count);
        return values;
    }
    else {
        std::vector<math::vector<T, N>> values(accessor.count);
        return values;
    }
}

auto process_meshes(const tinygltf::Model& gmodel)
{
    std::vector<luc::model::mesh> meshes;
    meshes.reserve(gmodel.nodes.size());
    for (const auto& gmesh : gmodel.meshes) {
        LOG(INFO) << gmesh.name << "\n";
        luc::model::mesh mesh;
        for (const auto& gprim : gmesh.primitives) {
            luc::model::mesh::submesh smesh;
            smesh.material = gprim.material;
            smesh.indices = get_component<uint32_t, 1>(gmodel, gprim.indices);
            switch (gprim.mode) {
                case TINYGLTF_MODE_TRIANGLES: {
                    LOG(INFO) << "TRIANGLES\n";
                    for (const auto& attribute : gprim.attributes) {
                        if (attribute.first == "POSITION")
                            smesh.vertices = get_component<float, 3>(gmodel, attribute.second);
                        if (attribute.first == "NORMAL")
                            smesh.normals = get_component<float, 3>(gmodel, attribute.second);
                        if (attribute.first == "TEXCOORD_0")
                            smesh.texcoords = get_component<float, 2>(gmodel, attribute.second);
                    }
                } break;
                default:
                    LOG(ERROR) << "primitive mode not implemented\n";
                    break;
            }
            mesh.meshes.push_back(std::move(smesh));
        }
        meshes.push_back(std::move(mesh));
    }
    return meshes;
}

auto process_instances(const tinygltf::Model& gmodel)
{
    std::vector<luc::model::instance> instances;
    instances.reserve(gmodel.nodes.size());
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
        instances.push_back(instance);
    }
    return instances;
}

auto process_materials(const tinygltf::Model& gmodel)
{
    std::vector<luc::model::material> materials;
    materials.reserve(gmodel.materials.size());
    for (const auto& gmat : gmodel.materials) {
        LOG(INFO) << "Material: " << gmat.name << "\n";
        luc::model::material material;
        const auto& albedo = gmat.pbrMetallicRoughness.baseColorFactor;
        material.albedo.c = math::float3((float)albedo[0], (float)albedo[1], (float)albedo[2]);
        material.metallic.c = (float)gmat.pbrMetallicRoughness.metallicFactor;
        material.roughness.c = (float)gmat.pbrMetallicRoughness.roughnessFactor;
        if (gmat.emissiveTexture.index > -1)
            material.emission.texture = gmat.emissiveTexture.index;
        if (gmat.pbrMetallicRoughness.baseColorTexture.index > -1)
            material.albedo.texture = gmat.pbrMetallicRoughness.baseColorTexture.index;
        for (const auto& value : gmat.extensions)
            if (value.first == "KHR_materials_emissive_strength")
                material.emissive_strength = (float)value.second.Get("emissiveStrength").GetNumberAsDouble();
            else if (value.first == "KHR_materials_ior")
                material.ior.c = (float)value.second.Get("ior").GetNumberAsDouble();
            else if (value.first == "KHR_materials_transmission")
                material.transmission.c = (float)value.second.Get("transmissionFactor").GetNumberAsDouble();
            else if (value.first == "KHR_materials_specular") {
                const auto specular = value.second.Get("specularColorFactor");
                material.specular.c.r = (float)specular.Get(0).GetNumberAsDouble();
                material.specular.c.g = (float)specular.Get(1).GetNumberAsDouble();
                material.specular.c.b = (float)specular.Get(2).GetNumberAsDouble();
            }
            else
                LOG(ERROR) << "unknown material extension: " << value.first << "\n";
        materials.push_back(material);
    }
    return materials;
}

auto process_textures(const tinygltf::Model& gmodel)
{
    std::vector<luc::texture<float, 3>> textures;
    textures.reserve(gmodel.textures.size());
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
            textures.push_back(std::move(image));
        }
    }
    return textures;
}

luc::model load_gltf(std::filesystem::path& path)
{
    LOG(INFO) << path << "\n";
    tinygltf::Model gmodel;
    tinygltf::TinyGLTF ctx;
    ctx.SetStoreOriginalJSONForExtrasAndExtensions(true);
    std::string error, warn;
    const auto ret = ctx.LoadBinaryFromFile(&gmodel, &error, &warn, path.c_str());
    if (!error.empty())
        LOG(ERROR) << error.c_str() << "\n";
    if (!warn.empty())
        LOG(WARNING) << warn.c_str() << "\n";
    luc::model model;
    if (!ret)
        return model;
    model.meshes = process_meshes(gmodel);
    model.instances = process_instances(gmodel);
    model.materials = process_materials(gmodel);
    model.textures = process_textures(gmodel);
    return model;
}
} // namespace luc::inner
