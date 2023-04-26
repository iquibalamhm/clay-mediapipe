#include "Mode.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>
#include <string>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
struct DoneMode : Mode {
	float ELAPSED_TIME;
	float ERROR;
	DoneMode(float elapsed_time,float error);
	virtual ~DoneMode();

	//functions called by main loop:
	virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
	virtual void update(float elapsed) override;
	virtual void draw(glm::uvec2 const &drawable_size) override;
	virtual void init_serial(std::string port_name) override;
	virtual void close_serial() override;
	virtual void init_function(std::string function) override;
	
	std::string serial_port_name = "None";
	struct function{
		std::string name;
		std::vector<double> coeff;
		uint8_t order;
	};
	function to_match;
	function fitted;

    LibSerial::SerialPort serial_port;

	glm::vec2 mouse_at = glm::vec2(std::numeric_limits< float >::quiet_NaN()); //in [-1,1]^2 coords
	
	//First hand coordinates

	bool do_pinch = false;
	//stored on each draw, used on each update:
	glm::mat4 world_to_clip = glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,  
										0.0f, 1.0f, 0.0f, 0.0f,  
										0.0f, 0.0f, 1.0f, 0.0f,  
										0.0f, 0.0f, 0.0f, 1.0f);

	float time_acc = 0.0f;


	glm::vec2 box_min = glm::vec2(0.0f, 0.0f);
	glm::vec2 box_max = glm::vec2(1.5f, 1.0f);

	glm::vec2 axis_min = glm::vec2(0.1f, 0.1f);
	glm::vec2 axis_max = glm::vec2(1.4f, 0.9f);

	void reset_clay();
	inline static constexpr float ClayTick = 0.001f;
	void tick_clay();

	//ad-hoc performance measurement:
	uint32_t ticks_acc = 0;
	uint32_t ticks_acc_filter = 0;

	
	double duration_acc = 0.0f;
	
};
