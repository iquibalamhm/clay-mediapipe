#include "Mode.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>
#include <string>

struct PlayMode : Mode {
	PlayMode();
	virtual ~PlayMode();

	//functions called by main loop:
	virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
	virtual void update(float elapsed) override;
	virtual void draw(glm::uvec2 const &drawable_size) override;
	virtual void polyfit(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &coeff,float &err,int order);

	std::string fitted_text = " x ";


	glm::vec2 mouse_at = glm::vec2(std::numeric_limits< float >::quiet_NaN()); //in [-1,1]^2 coords
	
	//First hand coordinates
	glm::vec2 index_1_at = glm::vec2(std::numeric_limits< float >::quiet_NaN()); //in [-1,1]^2 coords
	glm::vec2 thumb_1_at = glm::vec2(std::numeric_limits< float >::quiet_NaN()); //in [-1,1]^2 coords

	//Second hand coordinates
	glm::vec2 index_2_at = glm::vec2(std::numeric_limits< float >::quiet_NaN()); //in [-1,1]^2 coords
	glm::vec2 thumb_2_at = glm::vec2(std::numeric_limits< float >::quiet_NaN()); //in [-1,1]^2 coords
	
	bool do_pinch = false;
	//stored on each draw, used on each update:
	glm::mat4 world_to_clip = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,  
										0.0f, 1.0f, 0.0f, 0.0f,  
										0.0f, 0.0f, 1.0f, 0.0f,  
										0.0f, 0.0f, 0.0f, 1.0f);

	float time_acc = 0.0f;
	struct Particle {
		glm::vec2 pos = glm::vec2(0.0f, 0.0f);
		glm::vec2 vel = glm::vec2(0.0f, 0.0f);
		//uint32_t neighbors_begin = -1U, neighbors_end = -1U; //into neighbors array
	};

	/*
	struct Neighbor {
		uint32_t index = -1U;
		glm::vec2 offset = glm::vec2(0.0f, 0.0f);
	};
	*/

	std::vector< Particle > particles;
	//std::vector< Neighbor > neighbors;

	struct Probe {
		glm::vec2 pos = glm::vec2(0.0f, 0.0f);
		glm::vec2 target = glm::vec2(0.0f, 0.0f);
		bool active = true;
	};

	std::vector< Probe > probes;
	float probe_rot = 0.0f;
	float probe_pinch = 0.0f;
	bool do_rotation_left = false;
	bool do_rotation_right = false;
	bool do_hand_movement = false;

	bool rigid = false;
	int num_hands = 1;
	bool hand_1_closed = false;
	bool hand_2_closed = false;
	bool hand_1_active = false;
	bool hand_2_active = false;


    // Calculate the center of mass and moment of inertia
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    double m = 0.0;
    double Ixx = 0.0;
    double Iyy = 0.0;
    double Izz = 0.0;
	double I = 0.0;

	//float particle_radius = 0.01f;
	float particle_radius = 0.007f;
	float probe_radius = 0.08f;
	//float neighbor_radius = 0.2f;

	glm::vec2 box_min = glm::vec2(0.0f, 0.0f);
	glm::vec2 box_max = glm::vec2(1.5f, 1.0f);

	glm::vec2 axis_min = glm::vec2(0.1f, 0.1f);
	glm::vec2 axis_max = glm::vec2(1.4f, 0.9f);

	void reset_clay();
	inline static constexpr float ClayTick = 0.001f;
	void tick_clay();
	std::vector<double> coeff  = {0.0f,1.0f};
	uint32_t parabola_step = 10;
	uint32_t poly_order = 2;
	uint32_t fit_step = 10;
	//ad-hoc performance measurement:


	uint32_t ticks_acc = 0;
	double duration_acc = 0.0f;
};
