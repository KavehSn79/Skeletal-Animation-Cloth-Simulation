#include <fstream>
#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "cgtub/simple_renderer.hpp"

namespace ex6
{

using GuiChanges = uint32_t;

enum class AnimationName
{
    Swing_Dance = 0,
    Jump,
    Swimming
};

enum class SystemName
{
    Pendulum = 0,
    Curtain
};

enum class IntegrationMethod
{
    Euler = 0,
    Trapezoid
};

struct Skeleton
{
    /**
     * Returns an array (pointer to array start) of bone indices that influence a vertex `vertexIndex` in the elephant mesh.
     */
    int const* bones(int vertexIndex) const;

    /**
     * Returns an array (pointer to array start) of scalar bone weights for a vertex `vertexIndex` in the elephant mesh.
     */
    float const* weights(int vertexIndex) const;

    /**
     * Returns the number of bones that influence a given vertex with index `vertexIndex` in the elephant mesh.
     */
    int count(int vertexIndex) const;

    std::vector<int>   m_rows;
    std::vector<int>   m_cols;
    std::vector<float> m_weights;
};

struct SkeletalAnimation
{
    /**
     * Returns the number of bones in the skeletal animation.
     */
    int bones() const;

    /**
     * Returns an array (pointer to array start) of bone transformation matrices in a given frame (=index).
     */
    glm::mat4x4 const* frame(int index) const;

    unsigned int             m_frames{0u};
    std::vector<glm::mat4x4> m_transforms;
};

struct SpringSystem
{
    SystemName                system_name;    // pendulum or curtain
    std::vector<glm::vec3>    positions;      // particle positions p
    std::vector<glm::vec3>    init_positions; // initial particle positions for reset
    std::vector<glm::u32vec3> face_indices;   // face indices (only for curtain)
    std::vector<glm::vec3>    velocities;     // velocities v
    std::vector<glm::u32vec2> spring_indices; // springs (indices into positions)
    std::vector<float>        rest_lengths;   // rest length r for each spring
    std::vector<int>          pinned;         // fixed vertices (indices into positions)
    float                     mass;           // m
    float                     stiffness;      // k_s
    float                     drag;           // K_d
};

/**
 * \brief Query if an interaction with the GUI has changed a parameter value.
 *
 * Example usage:
 * \code{.cpp}
 * int   foo = ...;
 * float bar = ...;
 *
 * GuiChanges gui_changes = gui(&foo, &bar);
 *
 * if(has_gui_changed_parameter(gui_changes, 0))
 * {
 *     // Parameter `foo` was changed
 * }
 *
 * if(has_gui_changed_parameter(gui_changes, 1))
 * {
 *     // Parameter `bar` was changed
 * }
 * \endcode
 *
 * \param[in] changes         The \c GuiChanges returned by a call to \c gui(...)
 * \param[in] parameter_index The 0-based index of the parameter to query for changes
 */
bool has_gui_changed_parameter(GuiChanges gui_changes, uint32_t parameter_index);

/**
 * \brief Update the Graphical User Interface (GUI) and retrieve new values for the parameters.
 *
 * All non-const parameters are input/output, meaning their value will be used to display the GUI and
 * they will be set to the new value, as implied by user interaction with the GUI (in the previous frame).
 *
 * \return Object that tracks changes to the parameters.
 */
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
    bool&              reset_simulation);

/**
 * \brief Loads vertices, face indices, skeleton and a transformation of the elephant.
 *
 * Loads the vertices and face indices into given vectors. Also populates the entires of a skeleton struct for the elephant.
 * The elephant mesh will be centered and scaled to the unit box. This transformation will be stored into the transformation matrix.
 *
 * \param[out] vertices        The vertices will be inserted into this vector.
 * \param[out] faces           Indices for the faces will be inserted into this vector.
 * \param[out] skeleton        A struct that stores which vertex is affected by a bone (index) and how strong the influence is.
 * \param[out] transformation  This will be the transformation that centered and scaled the vertices to the unit box.
 *
 * \return Boolean if loading was successful.
 */
bool load_elephant(std::vector<glm::vec3>* vertices, std::vector<glm::u32vec3>* faces, Skeleton* skeleton, glm::mat4* transform);

/**
 * \brief Loads an animation for the elephant mesh.
 *
 * Loads the animation and rest pose for the animation specified in animation_name and stores the animation and rest pose in the
 * rest and anim structs. The animation will be transformed using the given transformation matrix.
 *
 * \param[in]  animation_name  This is the name of the animation to load.
 * \param[in]  transform       A transformation that is premultiplied to all bone transformations of the rest pose and animation.
 * \param[out] rest            All bone transformations for every frame are stored in this struct.
 * \param[out] anim            All bone transformation for the rest pose are stored in this struct (in frame 0).
 *
 * \return Boolean if loading was successful.
 */
bool load_animation(AnimationName animation_name, glm::mat4x4 const& transform, SkeletalAnimation* rest, SkeletalAnimation* anim);

/**
 * \brief Creates a pendulum chain.
 *
 * This will chain n_springs pendulums together. For n_spring == 1 this will be a single mass pendulum.
 * The pinned mass will be at (0, 0, 0)^T and the last mass will be at (0, 0, -size)^T.
 * Values for the constants used for the force calculation are also initialized.
 *
 * \param[in]  n_springs       How many springs should be attached to each other.
 * \param[in]  size            Size of the pendulum chain in world coordinates.
 * \param[out] spring_system   Vertices, spring indices, constants, etc. are stored in this struct.
 */
void create_pendulum_chain(int n_springs, float size, SpringSystem* spring_system);

/**
 * \brief Creates a curtain using springs.
 *
 * This will create a curtain mesh and structural, shear and flex springs between the vertices.
 * The curtain mesh contains at least 2 by 3 quadrilaterals (each out of 2 triangles) when n_subdivision == 0.
 * Values for the constants used for the force calculation are also initialized.
 *
 * \param[in]  n_subdivision   How often the 2 by 3 quadrilaterals are subdivided. The number of triangles and springs grow exponentially with this number.
 * \param[in]  size            Size of the curtain chain in world coordinates.
 * \param[in]  position        Position of the top left corner of the curtain in world coordinates.
 * \param[out] spring_system   Vertices, face indices, spring indices, constants, etc. are stored in this struct.
 */
void create_curtain(int n_subdivision, float size, glm::vec3& position, SpringSystem* spring_system);

/**
 * \brief Renders a small sphere at the given position.
 *
 * \param[in]  render      This is the renderer object that is used to render the sphere.
 * \param[in]  position    Position of the sphere center in world coordinates.
 * \param[in]  color       Color of the sphere.
 */
void render_sphere(cgtub::SimpleRenderer& renderer, glm::vec3& position, glm::vec3& color);

/**
 * \brief Renders a given spring system according to its name property.
 *
 * For a pendulum, this will render small spheres for the masses and white lines for the springs.
 * For a curtain, this will render a red curtain mesh. The vertices correspond to the positions of the masses.
 *
 * \param[in]  render      This is the renderer object that is used to render the spring system.
 * \param[in]  system      This is the spring system to render.
 */
void render_spring_system(cgtub::SimpleRenderer& renderer, SpringSystem& system);

} // namespace ex6
