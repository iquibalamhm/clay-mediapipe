#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <SDL.h>
#include <SDL_opengl.h>

//#include "renderer.h"

// Particle parameters
constexpr float PARTICLE_RADIUS = 2.0f;
constexpr float PARTICLE_MASS = 1.0f;
constexpr float PARTICLE_DENSITY = 1.0f;

// Probe parameters
constexpr float PROBE_RADIUS = 3.0f;
constexpr float PROBE_FORCE = 100.0f;

// Simulation parameters
constexpr float DT = 1.0f / 60.0f;
constexpr float EPSILON = 1e-6f;
constexpr int ITERATIONS = 10;

// Particle structure
struct Particle {
    glm::vec2 pos = glm::vec2(0.0f, 0.0f);
    glm::vec2 vel = glm::vec2(0.0f, 0.0f);
};

// Probe structure
struct Probe {
    glm::vec2 pos = glm::vec2(0.0f, 0.0f);
    glm::vec2 target = glm::vec2(0.0f, 0.0f);
    bool active = true;
};
std::tuple<glm::vec2, glm::mat2> calculate_centroid_and_rotation_matrix(const std::vector<Particle>& particles) {
    // Calculate centroid
    glm::vec2 centroid(0.0f, 0.0f);
    for (const auto& particle : particles) {
        centroid += particle.pos;
    }
    centroid /= particles.size();

    // Calculate covariance matrix
    float c11 = 0.0f, c12 = 0.0f, c21 = 0.0f, c22 = 0.0f;
    for (const auto& particle : particles) {
        float x = particle.pos.x - centroid.x;
        float y = particle.pos.y - centroid.y;
        c11 += x * x;
        c12 += x * y;
        c21 += y * x;
        c22 += y * y;
    }
    glm::mat2 covariance(c11, c12, c21, c22);

    // Compute SVD of covariance matrix
    glm::mat2 rotation, singular_values;
    glm::vec2 scale;
    glm::mat2::decompose(covariance, scale, rotation, singular_values);

    return std::make_tuple(centroid, rotation);
}
glm::vec2 calculate_centroid(const std::vector<Particle>& particles) {
    glm::vec2 sum_pos(0.0f, 0.0f);
    for (const auto& particle : particles) {
        sum_pos += particle.pos;
    }
    return sum_pos / static_cast<float>(particles.size());
}

glm::mat2 calculate_rotation_matrix(const std::vector<Particle>& particles, const glm::vec2& centroid) {
    glm::mat2 covariance(0.0f);
    for (const auto& particle : particles) {
        glm::vec2 pos = particle.pos - centroid;
        covariance[0][0] += pos.x * pos.x;
        covariance[0][1] += pos.x * pos.y;
        covariance[1][0] += pos.y * pos.x;
        covariance[1][1] += pos.y * pos.y;
    }
    glm::vec2 eigenvalues;
    glm::mat2 eigenvectors;
    glm::eigen(covariance, eigenvalues, eigenvectors);
    return eigenvectors;
}

class Renderer {
public:
    Renderer() {
        // Compile the shader program
        const char* vertex_shader_source =
            "#version 150\n"
            "in vec2 position;"
            "uniform mat4 transform;"
            "void main() {"
            "   gl_Position = transform * vec4(position, 0.0, 1.0);"
            "}";
        const char* fragment_shader_source =
            "#version 150\n"
            "out vec4 out_color;"
            "void main() {"
            "   out_color = vec4(1.0, 1.0, 1.0, 1.0);"
            "}";
        GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex_shader, 1, &vertex_shader_source, nullptr);
        glCompileShader(vertex_shader);
        GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment_shader, 1, &fragment_shader_source, nullptr);
        glCompileShader(fragment_shader);
        shader_program_ = glCreateProgram();
        glAttachShader(shader_program_, vertex_shader);
        glAttachShader(shader_program_, fragment_shader);
        glBindFragDataLocation(shader_program_, 0, "out_color");
        glLinkProgram(shader_program_);
        glUseProgram(shader_program_);

        // Create the particle vertex buffer
        glGenBuffers(1, &particle_vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, particle_vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2), nullptr, GL_DYNAMIC_DRAW);
        GLint position_attrib = glGetAttribLocation(shader_program_, "position");
        glEnableVertexAttribArray(position_attrib);
        glVertexAttribPointer(position_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), 0);

        // Create the probe vertex buffer
        glGenBuffers(1, &probe_vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, probe_vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2), nullptr, GL_DYNAMIC_DRAW);
    }

    ~Renderer() {
        glDeleteProgram(shader_program_);
        glDeleteBuffers(1, &particle_vbo_);
        glDeleteBuffers(1, &probe_vbo_);
    }

    void render_particles(const std::vector<Particle>& particles) {
        glBindBuffer(GL_ARRAY_BUFFER, particle_vbo_);
        glBufferData(GL_ARRAY_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_DYNAMIC_DRAW);
        GLint transform_uniform = glGetUniformLocation(shader_program_, "transform");
        glm::mat4 identity(1.0f);
        glUniformMatrix4fv(transform_uniform, 1, GL_FALSE, &identity[0][0]);
        glPointSize(PARTICLE_RADIUS * 2);
    glDrawArrays(GL_POINTS, 0, particles.size());
}
void render_probes(const std::vector<Probe>& probes) {
    glBindBuffer(GL_ARRAY_BUFFER, probe_vbo_);
    glBufferData(GL_ARRAY_BUFFER, probes.size() * sizeof(Probe), probes.data(), GL_DYNAMIC_DRAW);
    GLint transform_uniform = glGetUniformLocation(shader_program_, "transform");
    for (const Probe& probe : probes) {
        if (probe.active) {
            glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(probe.pos, 0.0f)) * glm::scale(glm::mat4(1.0f), glm::vec3(PROBE_RADIUS * 2, PROBE_RADIUS * 2, 1.0f));
            glUniformMatrix4fv(transform_uniform, 1, GL_FALSE, &transform[0][0]);
            glDrawArrays(GL_TRIANGLE_FAN, 0, 8);
        }
    }
}
private:
GLuint shader_program_;
GLuint particle_vbo_;
GLuint probe_vbo_;
};

void render_probes(const std::vector<Probe>& probes) {
    glBindBuffer(GL_ARRAY_BUFFER, probe_vbo_);
    glBufferData(GL_ARRAY_BUFFER, probes.size() * sizeof(Probe), probes.data(), GL_DYNAMIC_DRAW);
    GLint transform_uniform = glGetUniformLocation(shader_program_, "transform");
    for (const Probe& probe : probes) {
        if (probe.active) {
            glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(probe.pos, 0.0f)) * glm::scale(glm::mat4(1.0f), glm::vec3(PROBE_RADIUS * 2, PROBE_RADIUS * 2, 1.0f));
            glUniformMatrix4fv(transform_uniform, 1, GL_FALSE, &transform[0][0]);
            glDrawArrays(GL_TRIANGLE_FAN, 0, 8);
        }
    }
}
// Apply a force from a probe to a particle
void apply_probe_force(std::vector<Particle>& particles, const Probe& probe) {
    for (auto& particle : particles) {
        if (glm::distance(particle.pos, probe.pos) < PARTICLE_RADIUS + PROBE_RADIUS) {
            glm::vec2 direction = glm::normalize(particle.pos - probe.pos);
            glm::vec2 force = direction * PROBE_FORCE;
            particle.vel += force / PARTICLE_MASS;
        }
    }
}
// Calculate the center of mass of a set of particles
glm::vec2 center_of_mass(const std::vector<Particle>& particles) {
    glm::vec2 sum = glm::vec2(0.0f, 0.0f);
    float total_mass = 0.0f;
    for (const auto& particle : particles) {
        sum += particle.pos * PARTICLE_MASS;
        total_mass += PARTICLE_MASS;
    }
    return sum / total_mass;
}

// Calculate the moment of inertia of a set of particles
float moment_of_inertia(const std::vector<Particle>& particles) {
    float sum = 0.0f;
    for (const auto& particle : particles) {
        sum += PARTICLE_MASS * glm::dot(particle.pos, particle.pos);
    }
    return sum;
}

// Calculate the rotation matrix for a set of particles
glm::mat2 rotation_matrix(const std::vector<Particle>& particles) {
    glm::vec2 center = center_of_mass(particles);
    float I = moment_of_inertia(particles);
    glm::mat2 A = glm::mat2(0.0f);
    for (const auto& particle : particles) {
        glm::vec2 r = particle.pos - center;
        A[0][0] += r.y * r.y;
        A[0][1] -= r.x * r.y;
        A[1][0] -= r.x * r.y;
        A[1][1] += r.x * r.x;
    }
    glm::mat2 U, S, V;
    glm::mat2 B = glm::transpose(A) * A;
    glm::vec2 eigenvalues = glm::eigenvalues(B[0][0], B[0][1],
 B[1][0], B[1][1]);
glm::vec2 eigenvector1, eigenvector2;
glm::eigenSVD(B, eigenvector1, eigenvector2);
if (glm::dot(eigenvector1, glm::vec2(-A[0][1], A[0][0])) < 0.0f) {
    eigenvector1 = -eigenvector1;
}
glm::mat2 R = glm::mat2(eigenvector1, eigenvector2);
return R;
}

void apply_transform(const glm::vec2& centroid, const glm::mat2& rotation, std::vector<Particle>& particles, std::vector<Probe> probes) {
    // Apply transformation to particles
    for (auto& particle : particles) {
        particle.pos = rotation * (particle.pos - centroid) + centroid;
    }

    // Apply transformation to probes
    for (auto& probe : probes) {
        probe.pos = rotation * (probe.pos - centroid) + centroid;
        probe.target = rotation * (probe.target - centroid) + centroid;
    }
}

int main(int argc, char* argv[]) {
// Initialize SDL
if (SDL_Init(SDL_INIT_VIDEO) != 0) {
std::cerr << "SDL_Init error: " << SDL_GetError() << std::endl;
return 1;
}
// Create a window and OpenGL context
SDL_Window* window = SDL_CreateWindow("Particle Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_OPENGL);
if (!window) {
    std::cerr << "SDL_CreateWindow error: " << SDL_GetError() << std::endl;
    SDL_Quit();
    return 1;
}
SDL_GLContext gl_context = SDL_GL_CreateContext(window);
if (!gl_context) {
    std::cerr << "SDL_GL_CreateContext error: " << SDL_GetError() << std::endl;
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
}

// Initialize OpenGL
if (SDL_GL_LoadLibrary(NULL) < 0) {
    std::cerr << "SDL_GL_LoadLibrary error: " << SDL_GetError() << std::endl;
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
}
SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
glMatrixMode(GL_PROJECTION);
glLoadIdentity();
glOrtho(0.0f, 800.0f, 0.0f, 600.0f, -1.0f, 1.0f);
glMatrixMode(GL_MODELVIEW);
glLoadIdentity();

// Initialize the renderer
Renderer renderer;

// Create the particles
std::vector<Particle> particles;
for (int i = 0; i < 800; i++) {
    float x = (i % 80) * (PARTICLE_RADIUS * 2.0f + 1.0f);
    float y = (i / 80) * (PARTICLE_RADIUS * 2.0f + 1.0f);
    particles.push_back(Particle{ glm::vec2(x,y), glm::vec2(0.0f, 0.0f) });
}

// Create the probes
std::vector<Probe> probes;
probes.push_back(Probe{ glm::vec2(200.0f, 300.0f), glm::vec2(200.0f, 300.0f), true });
probes.push_back(Probe{ glm::vec2(600.0f, 300.0f), glm::vec2(600.0f, 300.0f), true });

// Simulation loop
bool quit = false;
while (!quit) {
    // Handle events
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
        case SDL_QUIT:
            quit = true;
            break;
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym) {
            case SDLK_w:
                probes[0].target.y += 10.0f;
                break;
            case SDLK_a:
                probes[0].target.x -= 10.0f;
                break;
            case SDLK_s:
                probes[0].target.y -= 10.0f;
                break;
            case SDLK_d:
                probes[0].target.x += 10.0f;
                break;
            case SDLK_UP:
                probes[1].target.y += 10.0f;
                break;
            case SDLK_LEFT:
                probes[1].target.x -= 10.0f;
                break;
            case SDLK_DOWN:
                probes[1].target.y -= 10.0f;
                break;
            case SDLK_RIGHT:
                probes[1].target.x += 10.0f;
                break;
            }
            break;
        }
    }

    // Apply forces to the particles
    for (auto& particle : particles) {
        // Gravity
        particle.vel.y -= 0.1f;

        // Particle-probe interaction
        for (auto& probe : probes) {
            if (probe.active) {
                float distance = glm::distance(particle.pos, probe.pos);
                if (distance < PARTICLE_RADIUS + PROBE_RADIUS) {
                    glm::vec2 direction = glm::normalize(particle.pos - probe.pos);
                    particle.vel += direction * 0.2f;
                }
            }
        }
    }

    // Calculate centroid and rotation matrix
    auto [centroid, rotation] = calculate_centroid_and_rotation_matrix(particles);

    // Apply transformation
    apply_transform(centroid, rotation, particles, probes);

    // Draw particles and probes
    renderer.clear();
    draw_particles(renderer, particles);
    draw_probes(renderer, probes);
    renderer.present();
    // Apply Least-Squares Rigid Motion Using SVD
    glm::vec2 centroid = calculate_centroid(particles);
    glm::mat2 R = calculate_rotation_matrix(particles, centroid);
    glm::mat3 transform = glm::mat3(glm::vec3(R[0], 0.0f), glm::vec3(R[1], 0.0f), glm::vec3(centroid, 1.0f));
    apply_transform(particles, transform);
    apply_transform(probes, transform);

    // Update the probes' positions
    for (auto& probe : probes) {
        probe.pos = probe.pos + (probe.target - probe.pos) * 0.1f;
    }

    // Render the particles and probes
    glClear(GL_COLOR_BUFFER_BIT);
    renderer.render_particles(particles);
    renderer.render_probes(probes);
    SDL_GL_SwapWindow(window);

    // Delay to control the frame rate
    SDL_Delay(10);
}

// Clean up
SDL_GL_DeleteContext(gl_context);
SDL_DestroyWindow(window);
SDL_Quit();
return 0;
}