#include <functional>
#include <vector>

#define GLFW_INCLUDE_NONE

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <iostream>

#include "cgtub/canvas.hpp"
#include "cgtub/event_dispatcher.hpp"
#include "cgtub/gl_wrap.hpp"
#include "cgtub/primitives.hpp"
#include "cgtub/simple_renderer.hpp"
#include "helper.hpp"

static void compute_derivative(const ex6::SpringSystem&      sys, const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& velocities,
    std::vector<glm::vec3>&       dpos_dt, std::vector<glm::vec3>&       dvel_dt)
{
    const int n = (int)positions.size();
    dpos_dt.assign(n, glm::vec3(0.0f));
    dvel_dt.assign(n, glm::vec3(0.0f));

    std::vector<char> is_pinned(n, 0);
    for (int idx : sys.pinned)
        if (0 <= idx && idx < n)
            is_pinned[idx] = 1;

    for (int i = 0; i < n; ++i)
        if (!is_pinned[i])
            dpos_dt[i] = velocities[i];

    std::vector<glm::vec3> F(n, glm::vec3(0.0f));

    const glm::vec3 g(0.0f, -9.81f, 0.0f);
    for (int i = 0; i < n; ++i)
        if (!is_pinned[i])
            F[i] += sys.mass * g;

    // Drag
    for (int i = 0; i < n; ++i)
        if (!is_pinned[i])
            F[i] += -sys.drag * velocities[i];

    // Springs
    for (size_t s = 0; s < sys.spring_indices.size(); ++s)
    {
        glm::u32vec2 ij = sys.spring_indices[s];
        int          i  = (int)ij.x;
        int          j  = (int)ij.y;

        glm::vec3 d   = positions[i] - positions[j];
        float     len = glm::length(d);

        if (len > 1e-6f)
        {
            glm::vec3 dir = d / len;
            float     r   = sys.rest_lengths[s];
            glm::vec3 Fs  = -sys.stiffness * (len - r) * dir;

            if (!is_pinned[i])
                F[i] += Fs;
            if (!is_pinned[j])
                F[j] -= Fs;
        }
    }

    for (int i = 0; i < n; ++i)
        if (!is_pinned[i])
            dvel_dt[i] = F[i] / sys.mass;
}



int main(int argc, char** argv)
{
    // Create a GLFW window and an OpenGL context
    // An event dispatcher records incoming events for a window
    // (e.g. change of size or mouse cursor movement)
    GLFWwindow*             window     = nullptr;
    cgtub::EventDispatcher* dispatcher = nullptr;
    if (!cgtub::init(800, 600, "CG1", &window, &dispatcher))
    {
        std::cerr << "Failed to initialize OpenGL window" << std::endl;
        return EXIT_FAILURE;
    }

    // A canvas is a (logical) subregion of a window.
    cgtub::Canvas         canvas(window, cgtub::Extent{.x = 0, .y = 0, .width = 1.f, .height = 1.f});
    cgtub::SimpleRenderer renderer(canvas);

    // State
    ex6::AnimationName     animation_name     = ex6::AnimationName::Swing_Dance;
    bool                   show_skeleton      = true;
    bool                   show_elephant      = true;
    bool                   play_animation     = false;
    ex6::SystemName        system_name        = ex6::SystemName::Pendulum;
    ex6::IntegrationMethod integration_method = ex6::IntegrationMethod::Euler;
    bool                   show_simulation    = true;
    int                    spring_subdivision = 1;
    float                  simulation_speed   = 0.2f;
    bool                   update_simulation  = false;
    bool                   reset_simulation   = false;

    unsigned int frame = 0;
    float        time  = static_cast<float>(glfwGetTime());

    // imports for linear blend skinning
    std::vector<glm::vec3>    elephant_vertices;
    std::vector<glm::u32vec3> elephant_faces;
    ex6::Skeleton             skeleton;
    glm::mat4x4               transform;
    ex6::load_elephant(&elephant_vertices, &elephant_faces, &skeleton, &transform);

    ex6::SkeletalAnimation rest;
    ex6::SkeletalAnimation anim;
    ex6::load_animation(animation_name, transform, &rest, &anim);

    // Skin 
    std::vector<glm::vec3> skinned_vertices = elephant_vertices;
    std::vector<glm::mat4> inv_rest;
    inv_rest.resize(rest.bones());

    glm::mat4x4 const* Gbar = rest.frame(0);
    int                m    = rest.bones();

    for (int j = 0; j < m; ++j)
        inv_rest[j] = glm::inverse(Gbar[j]);



    // setup for spring simulation
    ex6::SpringSystem spring_system;
    glm::vec3         curtain_position(-1.5, 1.5, -1.0);
    ex6::create_pendulum_chain(spring_subdivision, 0.5, &spring_system);

    if (system_name == ex6::SystemName::Pendulum)
        ex6::create_pendulum_chain(spring_subdivision, 0.5f, &spring_system);
    else
        ex6::create_curtain(spring_subdivision, 0.8f, curtain_position, &spring_system);


    while (!glfwWindowShouldClose(window))
    {
        cgtub::begin_frame(window);

        // Track current time and elapsed time between frames (dt)
        float now = static_cast<float>(glfwGetTime());
        float dt  = now - time;
        time      = now;

        // Poll window and record window events (resizing, key inputs, etc.)
        dispatcher->poll_window_events();

        // Handle resize and input events
        canvas.update(dt, dispatcher);
        renderer.update(dt, dispatcher);

        ex6::GuiChanges gui_changes = ex6::gui(
            show_skeleton,
            show_elephant,
            animation_name,
            play_animation,
            show_simulation,
            system_name,
            integration_method,
            spring_subdivision,
            simulation_speed,
            spring_system.mass,
            spring_system.stiffness,
            spring_system.drag,
            update_simulation,
            reset_simulation);

        // == Skeletal Animation ==
        if (ex6::has_gui_changed_parameter(gui_changes, 2)) // animation changed
        {
            ex6::load_animation(animation_name, transform, &rest, &anim);
            frame = 0;

            inv_rest.resize(rest.bones());
            glm::mat4 const* Gbar = rest.frame(0);
            int              m    = rest.bones();
            for (int j = 0; j < m; ++j)
                inv_rest[j] = glm::inverse(Gbar[j]);
        }


        // == Spring Simulation ==
        if (ex6::has_gui_changed_parameter(gui_changes, 5) || ex6::has_gui_changed_parameter(gui_changes, 7))
        {
            if (system_name == ex6::SystemName::Pendulum)
                ex6::create_pendulum_chain(spring_subdivision, 0.5f, &spring_system);
            else
                ex6::create_curtain(spring_subdivision, 0.8f, curtain_position, &spring_system);
        }


        if (reset_simulation)
        {
            spring_system.positions = spring_system.init_positions;
            spring_system.velocities.assign(spring_system.positions.size(), glm::vec3(0.0f));
        }

        if (play_animation && anim.m_frames > 0)
        {
            frame++;
            if (frame >= anim.m_frames)
                frame = 0;
        }
        else
        {
            frame = 0;
        }

        //Linear Blend Skinning
        if (show_elephant)
        {
            if (!play_animation)
            {
                // rest pose
                skinned_vertices = elephant_vertices;
            }
            else
            {
                glm::mat4 const* G = anim.frame(frame);
                int              m = anim.bones();

                skinned_vertices.resize(elephant_vertices.size());

                for (size_t vi = 0; vi < elephant_vertices.size(); ++vi)
                {
                    glm::vec4 p(elephant_vertices[vi], 1.f);
                    glm::vec4 p_new(0.f);

                    int          n        = skeleton.count((int)vi);
                    int const*   bone_ids = skeleton.bones((int)vi);
                    float const* w        = skeleton.weights((int)vi);

                    for (int k = 0; k < n; ++k)
                    {
                        int   j   = bone_ids[k];
                        float wij = w[k];

                        glm::mat4 M = G[j] * inv_rest[j];

                        p_new += wij * (M * p);
                    }

                    skinned_vertices[vi] = glm::vec3(p_new);
                }
            }
        }

        // Euler and Trapezoid integration
        if (show_simulation && update_simulation)
        {
            const int n = (int)spring_system.positions.size();
            if (n > 0)
            {
                float h = simulation_speed * dt;

                std::vector<glm::vec3> dp0, dv0;
                compute_derivative(spring_system,spring_system.positions, spring_system.velocities, dp0, dv0);

                if (integration_method == ex6::IntegrationMethod::Euler)
                {
                    for (int i = 0; i < n; ++i)
                    {
                        spring_system.positions[i] += h * dp0[i];
                        spring_system.velocities[i] += h * dv0[i];
                    }
                }
                else // Trapezoid
                {
                    std::vector<glm::vec3> pos_e(n), vel_e(n);
                    for (int i = 0; i < n; ++i)
                    {
                        pos_e[i] = spring_system.positions[i] + h * dp0[i];
                        vel_e[i] = spring_system.velocities[i] + h * dv0[i];
                    }

                    std::vector<glm::vec3> dp1, dv1;
                    compute_derivative(spring_system, pos_e, vel_e, dp1, dv1);
                    float half_h = 0.5f * h;
                    for (int i = 0; i < n; ++i)
                    {
                        spring_system.positions[i] += half_h * (dp0[i] + dp1[i]);
                        spring_system.velocities[i] += half_h * (dv0[i] + dv1[i]);
                    }
                }
            }
        }





        // == Render ==
        cgtub::clear(window, 0.05f, 0.05f, 0.05f, 1);
        canvas.clear();

        // skeleton
        if (show_skeleton)
        {
            glm::mat4x4 const* G          = nullptr;
            int                bone_count = 0;

            if (play_animation)
            {
                G          = anim.frame(frame);
                bone_count = anim.bones();
            }
            else
            {
                G          = rest.frame(0);
                bone_count = rest.bones();
            }

            if (G)
            {
                for (int j = 0; j < bone_count; ++j)
                {
                    glm::vec4 joint_world = G[j] * glm::vec4(0.f, 0.f, 0.f, 1.f);
                    glm::vec3 pos(joint_world);
                    glm::vec3 color(0.2f, 0.9f, 0.2f);
                    ex6::render_sphere(renderer, pos, color);
                }
            }
        }

        // elephant
        if (show_elephant)
        {
            renderer.render_mesh(skinned_vertices, elephant_faces, glm::vec3(0.2f, 0.9f, 0.2f));
        }

        // simulation
        if (show_simulation)
        {
            ex6::render_spring_system(renderer, spring_system);
        }

        cgtub::end_frame(window);

    }

    cgtub::uninit(window, dispatcher);

    return EXIT_SUCCESS;
}