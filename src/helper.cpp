#include "helper.hpp"

#include "cgtub/geometry.hpp"
#include "imgui.h"
#include "json.hpp"
#include "tiny_obj_loader.h"

#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <numbers>

namespace ex6
{
GuiChanges gui(
    bool&              show_skeleton,
    bool&              show_elephant,
    AnimationName&     animation_name,
    bool&              play_animation,
    bool&              show_simulation,
    SystemName&        system_name,
    IntegrationMethod& integration_method,
    int&               spring_subdivision,
    float&             simulation_speed,
    float&             mass,
    float&             stiffness,
    float&             drag,
    bool&              update_simulation,
    bool&              reset_simulation)
{
    GuiChanges changes{0};

    ImGui::Begin("CG1 - Exercise 6");
    ImGuiIO& io = ImGui::GetIO();
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);

    ImGui::SeparatorText("Skeletal Animation");
    if (ImGui::Checkbox("Show Skeleton", &show_skeleton))
        changes |= 1 << 0;
    if (ImGui::Checkbox("Show Elephant", &show_elephant))
        changes |= 1 << 1;
    if (ImGui::Combo("Animation Name", (int*)&animation_name, "Swing Dance\0Jump\0Swimming\0"))
        changes |= 1 << 2;
    if (ImGui::Checkbox("Play Animation", &play_animation))
        changes |= 1 << 3;

    ImGui::SeparatorText("Cloth Simulation");
    if (ImGui::Checkbox("Show Simulation", &show_simulation))
        changes |= 1 << 4;
    if (ImGui::Combo("System Name", (int*)&system_name, "Pendulum\0Curtain\0"))
        changes |= 1 << 5;
    if (ImGui::Combo("Integration Method", (int*)&integration_method, "Euler\0Trapezoid\0"))
        changes |= 1 << 6;
    if (ImGui::InputInt("Subdivisions", &spring_subdivision))
        changes |= 1 << 7;
    if (ImGui::InputFloat("Step", &simulation_speed))
        changes |= 1 << 8;
    if (ImGui::InputFloat("Mass", &mass))
        changes |= 1 << 9;
    if (ImGui::InputFloat("Stiffness", &stiffness))
        changes |= 1 << 10;
    if (ImGui::InputFloat("Drag", &drag))
        changes |= 1 << 11;
    if (ImGui::Checkbox("Run Simulation", &update_simulation))
        changes |= 1 << 12;
    reset_simulation = ImGui::Button("Reset Simulation");
    if (reset_simulation)
        changes |= 1 << 13;

    ImGui::End();

    return changes;
}

bool has_gui_changed_parameter(GuiChanges gui_changes, uint32_t parameter_index)
{
    if (parameter_index >= 32)
        return false;

    return static_cast<bool>(gui_changes & (1 << parameter_index));
}

int const* Skeleton::bones(int vertexIndex) const
{
    return &m_cols[m_rows[vertexIndex]];
}

float const* Skeleton::weights(int vertexIndex) const
{
    return &m_weights[m_rows[vertexIndex]];
}

int Skeleton::count(int vertexIndex) const
{
    return m_rows[vertexIndex + 1] - m_rows[vertexIndex];
}

int SkeletalAnimation::bones() const
{
    return m_transforms.size() / m_frames;
}

glm::mat4x4 const* SkeletalAnimation::frame(int index) const
{
    if (index >= m_frames)
        return nullptr;

    return &m_transforms[bones() * index];
}

bool load_obj(std::string const& path, std::vector<glm::vec3>* vertices, std::vector<glm::u32vec3>* faces, glm::mat4x4* transform)
{
    // Load the file
    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(path))
    {
        return false;
    }

    auto& shapes = reader.GetShapes();
    auto& attrib = reader.GetAttrib();

    // Copy vertex positions
    size_t numVertices = attrib.vertices.size() / 3;
    vertices->resize(numVertices);
    std::memcpy(vertices->data(), attrib.vertices.data(), sizeof(glm::vec3) * numVertices);

    // Scale and center the mesh
    glm::vec3 center(0);
    glm::vec3 min(std::numeric_limits<float>::max());
    glm::vec3 max(-std::numeric_limits<float>::max());
    for (glm::vec3 const& v : *vertices)
    {
        center += v / float(vertices->size());
        min = glm::min(min, v);
        max = glm::max(max, v);
    }
    float scale = 0.5f * glm::length(max - min);

    glm::mat4x4 T(1);
    T[3] = glm::vec4(-center, 1.f);
    glm::mat4x4 S(1 / scale);
    S[3][3] = 1.f;

    *transform = S * T;
    for (glm::vec3& v : *vertices)
    {
        v = *transform * glm::vec4(v, 1.f);
    }

    // Loop over faces(polygon)
    size_t index_offset = 0;
    faces->clear();
    for (size_t f = 0; f < shapes[0].mesh.num_face_vertices.size(); f++)
    {
        size_t fv = size_t(shapes[0].mesh.num_face_vertices[f]);
        if (fv != 3)
        {
            std::cerr << "Only triangle meshes are supported" << std::endl;
            return EXIT_FAILURE;
        }

        // Loop over vertices in the face.
        glm::u32vec3 face;
        for (size_t v = 0; v < fv; v++)
        {
            // access to vertex
            tinyobj::index_t idx = shapes[0].mesh.indices[index_offset + v];
            tinyobj::real_t  vx  = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
            tinyobj::real_t  vy  = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
            tinyobj::real_t  vz  = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

            face[v] = idx.vertex_index;
        }
        faces->push_back(face);
        index_offset += fv;
    }

    return true;
}

bool load_skeleton(std::string const& path_indices, std::string const& path_weights, Skeleton* skeleton)
{
    skeleton->m_cols.clear();
    skeleton->m_rows.clear();
    skeleton->m_weights.clear();

    std::ifstream  file_index(path_indices);
    nlohmann::json data_index = nlohmann::json::parse(file_index);
    if (!data_index.is_array())
        return false;

    // Build sparse bone weight matrix in CSR format
    // Vertices are rows, bones are columns, values are the weights:
    //    _____B_______
    //   |w1  0  0 w2
    // V | 0 w3  0  0
    //   | 0  0 w4 w5
    //   |
    //
    for (nlohmann::json::iterator it = data_index.begin(); it != data_index.end(); ++it)
    {
        size_t num_cols = skeleton->m_cols.size();
        skeleton->m_rows.push_back(num_cols);

        if (!it->is_array())
            return false;

        for (nlohmann::json::iterator bit = it->begin(); bit != it->end(); ++bit)
        {
            if (!bit->is_number_unsigned())
                return false;

            size_t bone_idx = (*bit).template get<size_t>();
            skeleton->m_cols.push_back(bone_idx);
        }
    }
    skeleton->m_rows.push_back(skeleton->m_cols.size());

    std::ifstream  file_weight(path_weights);
    nlohmann::json data_weight = nlohmann::json::parse(file_weight);
    if (!data_weight.is_array())
        return false;

    for (nlohmann::json::iterator it = data_weight.begin(); it != data_weight.end(); ++it)
    {
        if (!it->is_array())
            return false;

        for (nlohmann::json::iterator bit = it->begin(); bit != it->end(); ++bit)
        {
            if (!bit->is_number())
                return false;

            float bone_weight = (*bit).template get<float>();
            skeleton->m_weights.push_back(bone_weight);
        }
    }

    return true;
}

bool load_elephant(std::vector<glm::vec3>* vertices, std::vector<glm::u32vec3>* faces, Skeleton* skeleton, glm::mat4* transform)
{
    if (!load_obj(ASSETS_DIRECTORY + std::string("/elephant.obj"), vertices, faces, transform))
    {
        std::cerr << "Failed to read elephant mesh" << std::endl;
        return false;
    }
    if (!load_skeleton(ASSETS_DIRECTORY + std::string("/indices.json"), ASSETS_DIRECTORY + std::string("/weights.json"), skeleton))
    {
        std::cerr << "Failed to read elephant mesh" << std::endl;
        return false;
    }
    return true;
}

bool load_animation(AnimationName animation_name, glm::mat4x4 const& transform, SkeletalAnimation* rest, SkeletalAnimation* anim)
{
    std::string path;
    if (animation_name == AnimationName::Swing_Dance)
        path = ASSETS_DIRECTORY + std::string("/animation_swing_dance.json");
    else if (animation_name == AnimationName::Jump)
        path = ASSETS_DIRECTORY + std::string("/animation_jump.json");
    else if (animation_name == AnimationName::Swimming)
        path = ASSETS_DIRECTORY + std::string("/animation_swimming.json");
    else
        return false;

    std::ifstream  file(path);
    nlohmann::json data = nlohmann::json::parse(file);
    if (!data.is_object())
        return false;

    // Parse the rest pose
    auto& data_restpose = data["restpose"];
    if (!data_restpose.is_array())
        return false;

    rest->m_frames = 1;
    rest->m_transforms.clear();
    for (nlohmann::json::iterator it = data_restpose.begin(); it != data_restpose.end(); ++it)
    {
        if (!it->is_array())
            return false;

        glm::mat4x4  bone_mat;
        unsigned int index = 0;
        for (nlohmann::json::iterator mit = it->begin(); mit != it->end(); ++mit)
        {
            if (!mit->is_number())
                return false;

            bone_mat[index / 4][index % 4] = (*mit).template get<float>();
            ++index;
        }
        rest->m_transforms.push_back(transform * bone_mat);
    }

    auto& data_frames = data["frames"];
    if (!data_frames.is_array())
        return false;

    anim->m_frames = 0;
    anim->m_transforms.clear();
    for (nlohmann::json::iterator it = data_frames.begin(); it != data_frames.end(); ++it)
    {
        if (!it->is_array())
            return false;

        for (nlohmann::json::iterator fit = it->begin(); fit != it->end(); ++fit)
        {
            if (!fit->is_array())
                return false;

            glm::mat4x4  bone_mat;
            unsigned int index = 0;
            for (nlohmann::json::iterator mit = fit->begin(); mit != fit->end(); ++mit)
            {
                if (!mit->is_number())
                    return false;

                bone_mat[index / 4][index % 4] = (*mit).template get<float>();
                ++index;
            }
            anim->m_transforms.push_back(transform * bone_mat);
        }
        ++(anim->m_frames);
    }

    return true;
}

void create_pendulum_chain(int n_springs, float size, SpringSystem* spring_system)
{
    spring_system->system_name = SystemName::Pendulum;

    // Clear all data
    spring_system->positions.clear();
    spring_system->face_indices.clear();
    spring_system->velocities.clear();
    spring_system->spring_indices.clear();
    spring_system->rest_lengths.clear();
    spring_system->pinned.clear();

    // Preallocate memory
    const int n_vertices = n_springs + 1;
    spring_system->positions.reserve(n_vertices);
    spring_system->velocities.reserve(n_vertices);
    spring_system->spring_indices.reserve(n_springs);
    spring_system->rest_lengths.reserve(n_springs);

    // Setup chain of strings
    spring_system->pinned.push_back(0);
    for (int i = 0; i < n_vertices; i++)
    {
        spring_system->positions.emplace_back(0., 0., -i * size / n_springs);
        spring_system->velocities.emplace_back(0., 0., 0.);
    }
    for (int i = 0; i < n_springs; i++)
    {
        spring_system->spring_indices.emplace_back(i, i + 1);
        spring_system->rest_lengths.push_back(size / n_springs);
    }
    spring_system->init_positions = spring_system->positions;

    // Set constants
    spring_system->mass      = 1.;
    spring_system->stiffness = 75.f;
    spring_system->drag      = 0.1f;
}

void create_curtain(int n_subdivision, float size, glm::vec3& position, SpringSystem* spring_system)
{
    spring_system->system_name = SystemName::Curtain;

    // Clear all data
    spring_system->positions.clear();
    spring_system->face_indices.clear();
    spring_system->velocities.clear();
    spring_system->spring_indices.clear();
    spring_system->rest_lengths.clear();
    spring_system->pinned.clear();

    // create mesh
    int   nv_x   = 3 * (n_subdivision + 1) + 1;
    int   nv_y   = 2 * (n_subdivision + 1) + 1;
    float step_x = size * 1.5 / nv_x;
    float step_y = size / nv_y;
    spring_system->positions.reserve(nv_x * nv_y);
    spring_system->face_indices.reserve(2 * (nv_x - 1) * (nv_y - 1));
    for (int j = 0; j < nv_y; j++)
    {
        for (int i = 0; i < nv_x; i++)
        {
            // vertices
            float z_pos = 0.1 * std::sin(i * 4 * std::numbers::pi_v<float> / nv_x);
            spring_system->positions.push_back(glm::vec3(i * step_x, -j * step_y, z_pos) + position);

            // faces
            if (i < nv_x - 1 and j < nv_y - 1)
            {
                int v_idx = i + nv_x * j;
                spring_system->face_indices.emplace_back(v_idx, v_idx + nv_x, v_idx + 1);
                spring_system->face_indices.emplace_back(v_idx + 1, v_idx + nv_x, v_idx + nv_x + 1);
            }
        }
    }
    spring_system->init_positions = spring_system->positions;
    spring_system->velocities     = std::vector<glm::vec3>(nv_x * nv_y, glm::vec3(0.));

    // create springs
    int   n_structural  = nv_x * (nv_y - 1) + (nv_x - 1) * nv_y;
    int   n_shear       = 2 * (nv_x - 1) * (nv_y - 1);
    int   n_flex        = nv_x * (nv_y - 2) + (nv_x - 2) * nv_y;
    float len_strutural = size / nv_y;
    float len_shear     = std::sqrt(2 * len_strutural * len_strutural);
    float len_flex      = 2 * len_strutural;

    spring_system->spring_indices.reserve(n_structural + n_shear + n_flex);
    spring_system->rest_lengths.reserve(n_structural + n_shear + n_flex);
    for (int j = 0; j < nv_y; j++)
    {
        for (int i = 0; i < nv_x; i++)
        {
            int v_idx = i + nv_x * j;
            // structural springs
            if (i < nv_x - 1)
            {
                spring_system->spring_indices.emplace_back(v_idx, v_idx + 1);
                spring_system->rest_lengths.push_back(len_strutural);
            }
            if (j < nv_y - 1)
            {
                spring_system->spring_indices.emplace_back(v_idx, v_idx + nv_x);
                spring_system->rest_lengths.push_back(len_strutural);
            }

            // shear springs
            if (i < nv_x - 1 and j < nv_y - 1)
            {
                spring_system->spring_indices.emplace_back(v_idx, v_idx + nv_x + 1);
                spring_system->spring_indices.emplace_back(v_idx + 1, v_idx + nv_x);
                spring_system->rest_lengths.push_back(len_shear);
                spring_system->rest_lengths.push_back(len_shear);
            }

            // flex springs
            if (i < nv_x - 2)
            {
                spring_system->spring_indices.emplace_back(v_idx, v_idx + 2);
                spring_system->rest_lengths.push_back(len_flex);
            }
            if (j < nv_y - 2)
            {
                spring_system->spring_indices.emplace_back(v_idx, v_idx + 2 * nv_x);
                spring_system->rest_lengths.push_back(len_flex);
            }
        }
    }

    // pin 4 vertices at the top
    spring_system->pinned.reserve(4);
    for (int i = 0; i < 4; i++)
    {
        spring_system->pinned.push_back(i * (n_subdivision + 1));
    }

    // Set constants
    spring_system->mass      = 1.;
    spring_system->stiffness = 100.f;
    spring_system->drag      = 1.0f;
}

void render_sphere(cgtub::SimpleRenderer& renderer, glm::vec3& position, glm::vec3& color)
{
    static std::vector<glm::vec3>    sphere_vertices;
    static std::vector<glm::u32vec3> sphere_indices;
    cgtub::create_sphere_geometry(8, 8, glm::vec3(0.02f), &sphere_vertices, &sphere_indices);

    std::vector<glm::vec3> new_sphere_vertices = sphere_vertices;
    for (auto& v : new_sphere_vertices)
    {
        v += position;
    }
    renderer.render_mesh(new_sphere_vertices, sphere_indices, color);
}

void render_spring_system(cgtub::SimpleRenderer& renderer, SpringSystem& system)
{
    if (system.system_name == SystemName::Pendulum)
    {
        glm::vec3 sphere_color{glm::vec3(1.f, 0.1, 0.f)};
        for (auto& p : system.positions)
        {
            render_sphere(renderer, p, sphere_color);
        }

        int                    n = system.spring_indices.size();
        std::vector<glm::vec3> lines;
        std::vector<glm::vec3> line_colors(n, glm::vec3(1., 1., 1.));
        lines.reserve(n);
        for (auto& s : system.spring_indices)
        {
            lines.push_back(system.positions[s[0]]);
            lines.push_back(system.positions[s[1]]);
        }
        renderer.render_lines(lines, line_colors);
    }
    else if (system.system_name == ex6::SystemName::Curtain)
    {
        renderer.render_mesh(system.positions, system.face_indices, glm::vec3(0.8, 0.1, 0.1));
    }
}
} // namespace ex6
