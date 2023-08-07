#include <iostream>
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>
// Define the Particle structure with position and radius
struct Particle {
    
    glm::vec2 pos = glm::vec2(0.0f, 0.0f); // Position of the center of the particle
    glm::vec2 vel = glm::vec2(0.0f, 0.0f); // Velocity of the particle

    // Particle(const glm::vec2& pos, const glm::vec2& vel) : pos(pos), vel(vel) {}
};
struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        auto h1 = std::hash<T1>{}(pair.first);
        auto h2 = std::hash<T2>{}(pair.second);
        return h1 ^ h2;
    }
};
// Define the SpatialGrid class for collision detection
class SpatialGrid {
private:
    double binWidth, binHeight;
    std::unordered_map<std::pair<int, int>, std::vector<Particle*>, PairHash> grid;
public:
    SpatialGrid(double width, double height, double binSize) : binWidth(binSize), binHeight(binSize) {
        // Calculate the number of bins along each axis
        int numBinsX = static_cast<int>(width / binSize) + 1;
        int numBinsY = static_cast<int>(height / binSize) + 1;

        // Initialize the grid with empty vectors in each cell
        for (int i = 0; i < numBinsX; ++i) {
            for (int j = 0; j < numBinsY; ++j) {
                grid[std::make_pair(i, j)] = std::vector<Particle*>();
            }
        }
    }
    
    // Move a particle to a new bin based on its updated position
    void moveParticle(Particle* particle, int oldBinX, int oldBinY) {
        int newBinX = static_cast<int>(particle->pos.x / binWidth);
        int newBinY = static_cast<int>(particle->pos.y / binHeight);

        if (oldBinX != newBinX || oldBinY != newBinY) {
            // Remove the particle from its old bin
            auto& oldBin = grid[std::make_pair(oldBinX, oldBinY)];
            oldBin.erase(std::remove(oldBin.begin(), oldBin.end(), particle), oldBin.end());

            // Add the particle to its new bin
            grid[std::make_pair(newBinX, newBinY)].push_back(particle);
        }
    }

    // Add a particle to the spatial grid
    void addParticle(Particle* particle) {
        // Calculate the bin indices for the particle's position
        int binX = static_cast<int>(particle->pos.x / binWidth);
        int binY = static_cast<int>(particle->pos.y / binHeight);

        // Add the particle to the corresponding bin
        grid[std::make_pair(binX, binY)].push_back(particle);
    }

    // Get particles in a given bin
    const std::vector<Particle*>& getParticlesInBin(int binX, int binY) const {
        return grid.at(std::make_pair(binX, binY));
    }

    // Get the neighboring bins for a given bin
    std::vector<Particle*> getParticlesInNeighboringBins(int binX, int binY) const {
        std::vector<Particle*> neighboringParticles;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                int neighborX = binX + dx;
                int neighborY = binY + dy;

                auto it = grid.find(std::make_pair(neighborX, neighborY));
                if (it != grid.end()) {
                    const std::vector<Particle*>& particles = it->second;
                    neighboringParticles.insert(neighboringParticles.end(), particles.begin(), particles.end());
                }
            }
        }
        return neighboringParticles;
    }
    
    // Clear the spatial grid
    void clear() {
        for (auto& bin : grid) {
            bin.second.clear();
        }
    }
};
