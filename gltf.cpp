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
#include "camera.h"
#include "image.h"
#include "math/math.h"
#include "tinygltf/tiny_gltf.h"
#include <memory>

namespace luc::inner {
struct gltf_ctx {
    tinygltf::Model model;
    std::vector<std::shared_ptr<luc::texture<float, 3>>> textures;
};

template<typename T, typename S, size_t N>
auto get_component_list(const gltf_ctx& gltf, const tinygltf::Accessor& accessor)
{
    if constexpr (N == 1) {
        const auto& buffer_view = gltf.model.bufferViews[accessor.bufferView];
        const auto& buffer = gltf.model.buffers[buffer_view.buffer];
        const auto *ptr = buffer.data.data() + buffer_view.byteOffset + accessor.byteOffset;
        std::vector<T> values(accessor.count);
        const auto *data = (S *)ptr;
        for (size_t i = 0; i < accessor.count; i++)
            values[i] = static_cast<T>(data[i]);
        return values;
    }
    else {
        const auto& buffer_view = gltf.model.bufferViews[accessor.bufferView];
        const auto& buffer = gltf.model.buffers[buffer_view.buffer];
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
auto get_component(const gltf_ctx& gltf, int accessor_key)
{
    const auto& accessor = gltf.model.accessors[accessor_key];
    switch (accessor.componentType) {
        case TINYGLTF_COMPONENT_TYPE_BYTE:
            return get_component_list<T, int8_t, N>(gltf, accessor);
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
            return get_component_list<T, uint8_t, N>(gltf, accessor);
        case TINYGLTF_COMPONENT_TYPE_SHORT:
            return get_component_list<T, int16_t, N>(gltf, accessor);
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
            return get_component_list<T, uint16_t, N>(gltf, accessor);
        case TINYGLTF_COMPONENT_TYPE_INT:
            return get_component_list<T, int32_t, N>(gltf, accessor);
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
            return get_component_list<T, uint32_t, N>(gltf, accessor);
        case TINYGLTF_COMPONENT_TYPE_FLOAT:
            return get_component_list<T, float, N>(gltf, accessor);
        case TINYGLTF_COMPONENT_TYPE_DOUBLE:
            return get_component_list<T, double, N>(gltf, accessor);
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

void process_meshes(luc::model& model, const gltf_ctx& gltf)
{
    model.meshes.reserve(gltf.model.nodes.size());
    for (const auto& gmesh : gltf.model.meshes) {
        LOG(INFO) << gmesh.name << "\n";
        luc::model::mesh mesh;
        for (const auto& gprim : gmesh.primitives) {
            luc::model::mesh::submesh smesh;
            smesh.material = gprim.material;
            smesh.indices = get_component<uint32_t, 1>(gltf, gprim.indices);
            switch (gprim.mode) {
                case TINYGLTF_MODE_TRIANGLES: {
                    LOG(INFO) << "TRIANGLES\n";
                    for (const auto& attribute : gprim.attributes) {
                        if (attribute.first == "POSITION")
                            smesh.vertices = get_component<float, 3>(gltf, attribute.second);
                        if (attribute.first == "NORMAL")
                            smesh.normals = get_component<float, 3>(gltf, attribute.second);
                        if (attribute.first == "TEXCOORD_0")
                            smesh.texcoords = get_component<float, 2>(gltf, attribute.second);
                    }
                } break;
                default:
                    LOG(ERROR) << "primitive mode not implemented\n";
                    break;
            }
            mesh.meshes.push_back(std::move(smesh));
        }
        model.meshes.push_back(std::move(mesh));
    }
}

void process_instances(luc::model& model, const gltf_ctx& gltf)
{
    model.instances.reserve(gltf.model.nodes.size());
    for (const auto& gnode : gltf.model.nodes) {
        LOG(INFO) << "Instance: " << gnode.name << "\n";
        luc::model::instance instance;
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
        if (gnode.mesh > -1) {
            instance.id = gnode.mesh;
            model.instances.push_back(instance);
        }
        else if (gnode.camera > -1) {
            const auto cam = gltf.model.cameras[gnode.camera];
            const auto pos = instance.transform.w.xyz;
            const auto up = math::mul(instance.transform, math::float4(0.f, 1.f, 0.f, 0.f)).xyz;
            const auto dir = math::mul(instance.transform, math::float4(0.f, 0.f, -1.f, 0.f)).xyz;
            const auto target = pos + dir;
            const auto aspect = (float)cam.perspective.aspectRatio;
            const auto yfov = (float)cam.perspective.yfov;
            const ray_camera<float> camera(pos, target, up, aspect, yfov);
            model.cameras.push_back(camera);
        }
        else {
            LOG(ERROR) << "unknown instance type\n";
        }
    }
}

void process_materials(luc::model& model, const gltf_ctx& gltf)
{
    model.materials.reserve(gltf.model.materials.size());
    for (const auto& gmat : gltf.model.materials) {
        LOG(INFO) << "Material: " << gmat.name << "\n";
        luc::model::material material;
        const auto& albedo = gmat.pbrMetallicRoughness.baseColorFactor;
        material.albedo.value = math::float3((float)albedo[0], (float)albedo[1], (float)albedo[2]);
        material.metallic.value.t = (float)gmat.pbrMetallicRoughness.metallicFactor;
        material.roughness.value.t = (float)gmat.pbrMetallicRoughness.roughnessFactor;
        if (gmat.emissiveTexture.index > -1)
            material.emission.texture = gltf.textures[gmat.emissiveTexture.index];
        if (gmat.pbrMetallicRoughness.baseColorTexture.index > -1)
            material.albedo.texture = gltf.textures[gmat.pbrMetallicRoughness.baseColorTexture.index];
        for (const auto& value : gmat.extensions)
            if (value.first == "KHR_materials_emissive_strength")
                material.emissive_strength = (float)value.second.Get("emissiveStrength").GetNumberAsDouble();
            else if (value.first == "KHR_materials_ior")
                material.ior.value.t = (float)value.second.Get("ior").GetNumberAsDouble();
            else if (value.first == "KHR_materials_transmission")
                material.transmission.value.t = (float)value.second.Get("transmissionFactor").GetNumberAsDouble();
            else if (value.first == "KHR_materials_specular") {
                const auto specular = value.second.Get("specularColorFactor");
                material.specular.value.r = (float)specular.Get(0).GetNumberAsDouble();
                material.specular.value.g = (float)specular.Get(1).GetNumberAsDouble();
                material.specular.value.b = (float)specular.Get(2).GetNumberAsDouble();
            }
            else
                LOG(ERROR) << "unknown material extension: " << value.first << "\n";
        model.materials.push_back(material);
    }
}

void process_textures(gltf_ctx& gltf)
{
    gltf.textures.reserve(gltf.model.textures.size());
    for (const auto& tex : gltf.model.textures) {
        if (tex.source > -1) {
            const auto& gimage = gltf.model.images[tex.source];
            auto image = std::make_shared<luc::texture<float, 3>>(gimage.width, gimage.height);
            switch (gimage.pixel_type) {
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
                    load_raw_into_image<uint8_t>(image->buffer, gimage.component, (void *)gimage.image.data());
                } break;
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                    load_raw_into_image<uint16_t>(image->buffer, gimage.component, (void *)gimage.image.data());
                } break;
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
                    load_raw_into_image<uint32_t>(image->buffer, gimage.component, (void *)gimage.image.data());
                } break;
                case TINYGLTF_COMPONENT_TYPE_FLOAT: {
                    load_raw_into_image<float>(image->buffer, gimage.component, (void *)gimage.image.data());
                } break;
                default:
                    LOG(ERROR) << "unknown pixel type: " << gimage.mimeType << "(" << gimage.component << "x" << gimage.bits << "bits)\n";
                    break;
            }
            gltf.textures.push_back(image);
        }
    }
}

luc::model load_gltf(std::filesystem::path& path)
{
    LOG(INFO) << path << "\n";
    tinygltf::TinyGLTF ctx;
    ctx.SetStoreOriginalJSONForExtrasAndExtensions(true);
    std::string error, warn;
    gltf_ctx gltf;
    const auto ret = ctx.LoadBinaryFromFile(&gltf.model, &error, &warn, path.c_str());
    if (!error.empty())
        LOG(ERROR) << error.c_str() << "\n";
    if (!warn.empty())
        LOG(WARNING) << warn.c_str() << "\n";
    luc::model model;
    if (!ret)
        return model;
    process_textures(gltf);
    process_meshes(model, gltf);
    process_instances(model, gltf);
    process_materials(model, gltf);
    return model;
}
} // namespace luc::inner
