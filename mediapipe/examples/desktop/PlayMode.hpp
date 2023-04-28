#include "Mode.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>
#include <string>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>

struct PlayMode : Mode {
	PlayMode();
	virtual ~PlayMode();

	//functions called by main loop:
	virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
	virtual void update(float elapsed) override;
	virtual void draw(glm::uvec2 const &drawable_size) override;
	virtual void init_serial(std::string port_name) override;
	virtual void close_serial() override;
	virtual void init_function(std::string function) override;
	virtual void reset_motor();

	bool areVectorsApproximatelyEqual(const std::vector<double>& v1, const std::vector<double>& v2, double tolerance);
	void polyfit(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &coeff,float &err,uint8_t order);

	//called to create menu for current scene:
	void enter_scene(float elapsed);

	std::string serial_port_name = "None";
	struct function{
		std::string name;
		std::vector<double> coeff;
		std::vector<double> coeff_offset;
		uint8_t order;
	};
	function to_match;
	function fitted;

    LibSerial::SerialPort serial_port;
	
	enum {
		mainmenu,
		website,
		credit,
		gamescene,
		donemode
	} location = mainmenu;

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
	};
		//uint32_t neighbors_begin = -1U, neighbors_end = -1U; //into neighbors array

	/*
	struct Neighbor {
		uint32_t index = -1U;
		glm::vec2 offset = glm::vec2(0.0f, 0.0f);
	};
	*/
	bool interaction = false;
	const float PARTICLE_MASS = 1.0f;
	bool touching = false;
	bool isActive = false;
	std::vector< Particle > particles;
	//std::vector< Neighbor > neighbors;

	const bool filter_signal  = false;

	struct Probe {
		glm::vec2 pos = glm::vec2(0.0f, 0.0f);
		glm::vec2 target = glm::vec2(0.0f, 0.0f);
		bool active = true;
	};
		//glm::vec2 force = glm::vec2(0.0f, 0.0f);
	std::vector< Probe > probes;
	float probe_rot = 0.0f;
	float probe_pinch = 0.0f;
	bool do_rotation_left = false;
	bool do_rotation_right = false;
	bool do_hand_movement = false;

	float err_1 = 0.0f;
	float err_2 =0.0f;
	bool rigid = false;
	bool mod_1 = true;
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
	float particle_radius = 0.008f;
	float probe_radius = 0.08f;
	//float neighbor_radius = 0.2f;

	glm::vec2 box_min = glm::vec2(0.0f, 0.0f);
	glm::vec2 box_max = glm::vec2(1.5f, 1.0f);

	glm::vec2 axis_min = glm::vec2(0.1f, 0.1f);
	glm::vec2 axis_max = glm::vec2(1.4f, 0.9f);

	void reset_clay();
	void tick_clay();
	double final_error = -1.0;
	const float viscosity_radius = 4.0f * particle_radius;
	const float wall_bounce = 0.01f;
	// const float alpha = 0.9f; //controls particle squish
	const float alpha = 0.99f; //controls particle squish

	inline static constexpr float ClayTick = 0.001f;
	

	glm::vec2 view_min = glm::vec2(0,0);
	glm::vec2 view_max = glm::vec2(1024, 768);

	uint32_t parabola_step = 15;
	uint32_t fit_step = 10;
	//ad-hoc performance measurement:
	uint32_t ticks_acc = 0;
	uint32_t ticks_acc_filter = 0;
	std::chrono::_V2::system_clock::time_point start_time;
	double duration_acc = 0.0f;
	bool show_function_name = false;
	bool show_function_line = false;
	bool time_fixed = true;
	inline static constexpr float TIME_LIMIT = 5.0f; //given in seconds
};
