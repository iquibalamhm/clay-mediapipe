#include "Mode.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>
#include <string>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>

#include <algorithm> // For std::remove_if

#include <iostream> //For std::cout
#include <sstream>  // For std::ostringstream
#include <iomanip>  // For std::setprecision

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
	virtual void init_file(std::string input_file_name) override;

	virtual void reset_motor();

	bool areVectorsApproximatelyEqual(const std::vector<double>& v1, const std::vector<double>& v2, double tolerance);
	void polyfit(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &coeff,float &err,uint8_t order);
	void set_flags(std::string mode);
	//called to create menu for current scene:
	void enter_scene(float elapsed);
	void append_data(std::string input_file_name, std::vector<std::string> data);
	std::string serial_port_name = "None";

	struct function{
		std::string name;
		std::vector<double> coeff;
		std::vector<double> real_coeff = {0.0,0.0,0.0}; // ax^2 + bx + c
		uint8_t order;
		glm::vec2 start;
		glm::vec2 end;
		std::vector<double> vector_from_name(std::string name_string) {
			std::vector<double> coefficients = {0.0,0.0,0.0};
			name_string.erase(remove_if(name_string.begin(), name_string.end(), ::isspace), name_string.end());
			
			if (name_string == "None") {
				name = name_string;
				return coefficients;
			}
			// Remove whitespaces from the equation name

			std::cout<<name_string.size()<<" "<<name_string[0]<<" "<<name_string[name_string.size() - 3]<<" "<<name_string[name_string.size() - 2]<<std::endl;
			// Check if the equation is a parabola of the form "(x+k)^2"
			if (name_string.size() >= 6 && name_string[0] == '(' && name_string[name_string.size() - 3] == ')' && name_string[name_string.size() - 2] == '^') {
				std::cout<<"This kind of line " <<name_string<<std::endl;
				// Extract the value inside the parentheses
				std::string k_str = name_string.substr(2, name_string.size() - 5);
				std::cout<<k_str<<std::endl;
				double k = std::stod(k_str);

				// Calculate the coefficients
				double a = 1.0;
				double b = 2 * k;
				double c = k * k;
				coefficients = { a, b, c };

			}
			name = name_string;
			return coefficients;
		}
		
		std::string name_from_vector(std::vector<double> coeffs) {
			std::string name;
			int numCoeffs = coeffs.size();
			if (numCoeffs == 0) {
				return name;
			}

			char variable = 'x';
			std::ostringstream oss;
			oss << std::fixed << std::setprecision(1);  // Set precision to 1 decimal place

			for (int i = 0; i < numCoeffs; i++) {
				double coeff = coeffs[i];

				if (coeff == 0.0) {
					if (i == numCoeffs - 1 && name.empty()) {
						name += "0.0";  // Append 0.0 instead of 'x'
					}
					continue;
				}

				if (!name.empty()) {
					if (coeff > 0.0) {
						name += "+";
					} else {
						name += "-";
						coeff = -coeff;
					}
				}

				if (i == numCoeffs - 1) {
					if (coeff == 1.0) {
						name += "1.0";  // Append 1.0 instead of 'x'
					} else {
						oss.str("");  // Clear the ostringstream
						oss << coeff;  // Set the coefficient value with desired precision
						name += oss.str();
					}
				} else {
					oss.str("");  // Clear the ostringstream
					oss << coeff;  // Set the coefficient value with desired precision
					name += oss.str();
					name += "x";
					if (i < numCoeffs - 1) {
						if(numCoeffs - i - 1>1){
							name += "^";
							name += std::to_string(numCoeffs - i - 1);
						}
					}
				}
			}
        	return name;
    	}
		//rename from real_coeff modified
		void rename(std::vector<double> input_vector){
			real_coeff = input_vector;
			name  = name_from_vector(input_vector);
		}
		function()
		{
			//inside empty constructor
		}
		function(std::vector<double> coeffs)
		{
			real_coeff = coeffs;
			coeff = coeffs;
			coeff[2] = 0.5;
			name = name_from_vector(coeffs);
		}
		function(std::string name_string)
		{
			real_coeff =  vector_from_name(name_string);
			name = name_string;
		}
	};
	std::vector<double> initial_coeff = {0.0,0.0,0.0};
	function prev_match = function(initial_coeff);
	function to_match = function(initial_coeff);
	function fitted = function(initial_coeff);

	void parse_function(function &selected_function);
	std::vector<double> parse_function_coeffs(std::vector<double>& coeffs_unscaled);

	struct scene_order{
		// std::vector<std::string> function_order;
		std::vector<function> functions;
		int curr_val = 0;
		scene_order()
		{
			//inside empty constructor
		}
		scene_order(std::vector<std::vector<double>> order)
		{
			for (auto &i : order)
			{
				functions.push_back(function(i));
			}
			curr_val = 0;
			//function_order = order;
			std::cout<<"Assigned function order"<<std::endl;
			//inside parameterized constructor
		}
		scene_order(std::vector<std::string> order){
			
			for (auto &i : order)
			{
				std::cout<<"input name"<< i<<std::endl;
				functions.push_back(function(i));
			}
			curr_val = 0;
		}
	}current_order;
	
    LibSerial::SerialPort serial_port;
	
	enum {
		mainmenu,
		website,
		credit,
		gamescene,
		donemode,
		scenes_mode,
		instructions,
	} location = mainmenu, next_location = mainmenu;
	enum {
		line_intercept,
		line_slope,
		parabola_concavity,
		parabola_vertex,
		experimental_1,
		experimental_2,
		custum_function,
	}scene = line_intercept;
	enum{
		begin,
		inside,
		end,
	}state = begin;
	
	enum {
		y1,
		y2,
	}intercept_state = y1;
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

	struct colors{
		glm::u8vec4 red = glm::u8vec4(0xff, 0x88, 0x88, 0xff);
		glm::u8vec4 black = glm::u8vec4(0x08, 0x44, 0x44, 0xff);
		glm::u8vec4 blue = glm::u8vec4(0x31, 0x41, 0xD3, 0xff);
		glm::u8vec4 green = glm::u8vec4(0x20, 0x95, 0x19, 0xff);
		glm::u8vec4 light_green = glm::u8vec4(0x70, 0xF7, 0x62, 0xff);		
		glm::u8vec4 light_gray = glm::u8vec4(0xa1, 0xa1, 0xa1, 0xff);		
	}colors;
	//float particle_radius = 0.01f;
	float particle_radius = 0.008f;
	// float probe_radius = 0.08f;
	float probe_radius = 0.06f;
	
	//float neighbor_radius = 0.2f;

	glm::vec2 box_min = glm::vec2(0.0f, 0.0f);
	glm::vec2 box_max = glm::vec2(1.5f, 1.0f);

	glm::vec2 axis_min = glm::vec2(0.1f, 0.05f);
	glm::vec2 axis_max = glm::vec2(1.4f, 0.95f);
	glm::vec2 center_axis = glm::vec2((axis_max.x-axis_min.x)/2 + axis_min.x, (axis_max.y-axis_min.y)/2 + axis_min.y);

	//Unit lines to show division in the cartesian plane
	float line_half_lenght = 0.015f;
	float unit_line_spacing = 0.1f;

	void reset_clay();
	void tick_clay();
	double final_error = -1.0;

	//Particle parameters
	const float viscosity_radius = 2.0f * particle_radius;
	const float wall_bounce = 0.01f;
	// const float alpha = 0.9f; //controls particle squish
	const float alpha = 0.99f; //controls particle squish
	const float kFriction = 0.25f; //0.1f 
	const float kDamping = 0.01f; //0.05f
	inline static constexpr float ClayTick = 0.001f;
	
	bool flag_switch = false;
	glm::vec2 view_min = glm::vec2(0,0);
	glm::vec2 view_max = glm::vec2(1024, 768);



	uint32_t parabola_step = 50;
	uint32_t fit_step = 05;
	//ad-hoc performance measurement:
	uint32_t ticks_acc = 0;
	uint32_t ticks_acc_filter = 0;
	std::chrono::_V2::system_clock::time_point start_time;
	double duration_acc = 0.0f;

	std::chrono::_V2::system_clock::time_point matching_start_time;


	bool show_function_name = true;
	bool show_function_line = true;
	bool show_function_line_on_finished = false;

	bool show_to_match_line = false;
	bool show_fitted_line = true;
	bool show_to_match_line_on_finished = true;
	std::string file_name = "";
	bool time_fixed = true;
	inline static constexpr float TIME_LIMIT = 20.0f; //given in seconds
	inline static constexpr float TIME_LIMIT_PER_FUNCTION = 60.0f; //given in seconds

};
