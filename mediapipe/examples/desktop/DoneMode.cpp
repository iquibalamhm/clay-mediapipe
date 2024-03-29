#include "DoneMode.hpp"

#include "DrawLines.hpp"
#include "gl_errors.hpp"
#include "data_path.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <array>
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/QR>
#include <boost/thread.hpp>

DoneMode::DoneMode(float elapsed_time,float error) {
	this->ELAPSED_TIME = elapsed_time;
	this->ERROR = error;
	reset_clay();
}

DoneMode::~DoneMode() {
}

bool DoneMode::handle_event(SDL_Event const &evt, glm::uvec2 const &window_size) {

	if (evt.type == SDL_KEYDOWN) {
		if (evt.key.keysym.sym == SDLK_BACKSPACE) {
			reset_clay();
		} else if (evt.key.keysym.sym == SDLK_SPACE) {

		} else if(evt.key.keysym.sym == SDLK_LEFT){
		}
		else if(evt.key.keysym.sym == SDLK_RIGHT){
		}
	} else if (evt.type == SDL_KEYUP) {
		if(evt.key.keysym.sym == SDLK_LEFT){
		}
		else if(evt.key.keysym.sym == SDLK_RIGHT){
		}
	} else if (evt.type == SDL_MOUSEBUTTONDOWN) {
		if (evt.button.button == SDL_BUTTON_LEFT) {
			//std::cout<<"do_pinch"<<std::endl;
			do_pinch = true;
		}
	} else if (evt.type == SDL_MOUSEBUTTONUP) {
		if (evt.button.button == SDL_BUTTON_LEFT) {
			do_pinch = false;
		}
	} else if (evt.type == SDL_MOUSEWHEEL) {
	} else if (evt.type == SDL_MOUSEMOTION) {
		//std::cout<<"hand found"<<std::endl;
		mouse_at.x =-1.0f + 2.0f * (evt.motion.x + 0.5f) / float(window_size.x);
		//std::cout<<window_size.x<<" "<<window_size.y<<" "<<std::endl;
		mouse_at.y = 1.0f - 2.0f * (evt.motion.y + 0.5f) / float(window_size.y);
		//std::cout<<"x: "<<mouse_at.x<<"  y:"<<mouse_at.y<<"  evt x:"<<evt.motion.x<<"  evt y:"<<evt.motion.y<<" "<<window_size.x<<" "<<window_size.y<<" "<<std::endl;
	} else if (evt.type == SDL_USEREVENT){
		std::vector<int>* coordinates = static_cast<std::vector<int>*>(evt.user.data1);
	}

	return false;
}

void DoneMode::update(float elapsed) {
	time_acc += elapsed;

	auto before = std::chrono::high_resolution_clock::now();
	uint32_t ticks = 0;
	
	auto after = std::chrono::high_resolution_clock::now();
	ticks_acc += ticks;
	ticks_acc_filter += ticks;
	duration_acc += std::chrono::duration< double >(after - before).count();
}
void DoneMode::init_serial(std::string port_name){
	// Open the hardware serial ports.
	serial_port_name = port_name;

	if (serial_port_name != "None"){
		serial_port.Open(port_name);
		//serial_stream.Open( "/dev/ttyACM1" );
		// Set the baud rates.
		serial_port.SetBaudRate( LibSerial::BaudRate::BAUD_115200 );
		serial_port.Write("i0l");
		std::string read_byte_1;
		//serial_port.Read(read_byte_1,19);
		std::cout<<"serial port opened "<<port_name<<std::endl;
	}
}

void DoneMode::init_function(std::string function_name){
	// Open the hardware serial ports.
	//function_to_match = function_name;

}
void DoneMode::close_serial(){
	// Close the serial ports
	if (serial_port_name != "None"){
		serial_port.Write("i0l");
		std::string read_byte_1;
		//serial_port.Read(read_byte_1,19);
		serial_port.Close();
		std::cout<<"serial port closed "<<read_byte_1;
	}
}
void DoneMode::draw(glm::uvec2 const &drawable_size) {
	glClearColor(0.9f, 0.9f, 0.87f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);

	const float aspect = drawable_size.x / float(drawable_size.y);
	//std::cout<< drawable_size.x<<" "<<float(drawable_size.y)<<std::endl;
	const float scale = std::min(1.5f * aspect / (box_max.x - box_min.x + 0.1f), 1.5f / (box_max.y - box_min.y + 0.1f));
	//std::cout<< aspect<<" "<<scale<<std::endl;
	//const glm::vec2 offset = -0.5f * (box_min + box_max);
	const glm::vec2 offset = -0.5f * (box_min + box_max);
	
	//std::cout<<box_min.x<<" "<<box_min.y<<" "<<box_max.x<<" "<<box_max.y<<" "<<offset.x<<" "<<offset.y<<" "<<std::endl;
	world_to_clip = glm::mat4(
		scale / aspect, 0.0f, 0.0f, 0.0f,
		0.0f, scale, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		offset.x * scale / aspect, offset.y * scale, 0.0f, 1.0f
	);

	{
		DrawLines lines(world_to_clip);

		//boundary:
		lines.draw(glm::vec3(box_min.x, box_min.y, 0.0f), glm::vec3(box_max.x, box_min.y, 0.0f), glm::u8vec4(0xff, 0x88, 0x88, 0xff));
		lines.draw(glm::vec3(box_max.x, box_min.y, 0.0f), glm::vec3(box_max.x, box_max.y, 0.0f), glm::u8vec4(0xff, 0x88, 0x88, 0xff));
		lines.draw(glm::vec3(box_max.x, box_max.y, 0.0f), glm::vec3(box_min.x, box_max.y, 0.0f), glm::u8vec4(0xff, 0x88, 0x88, 0xff));
		lines.draw(glm::vec3(box_min.x, box_max.y, 0.0f), glm::vec3(box_min.x, box_min.y, 0.0f), glm::u8vec4(0xff, 0x88, 0x88, 0xff));

		//from 15-466-f22-base6:
		static std::array< glm::vec2, 16 > const circle = [](){
			std::array< glm::vec2, 16 > ret;
			for (uint32_t a = 0; a < ret.size(); ++a) {
				float ang = a / float(ret.size()) * 2.0f * float(M_PI);
				ret[a] = glm::vec2(std::cos(ang), std::sin(ang));
			}
			return ret;
		}();
		std::string print_message;
		//std::to_string(ELAPSED_TIME);
		print_message.append("Elapsed time");
		print_message.append(std::to_string(int(ELAPSED_TIME)));
		print_message.append(" seconds");
		lines.draw_text(print_message,
			glm::vec3(0.1f, 0.5f, 0.0f),
			glm::vec3(0.10f, 0.0f, 0.0f),
			glm::vec3(0.0f, 0.10f, 0.0f),
			glm::u8vec4(0x00, 0x00, 0x00, 0x00));
		
		std::string print_message_error;
		print_message_error.append("L2_norm error ");
		print_message_error.append(std::to_string(ERROR));
		lines.draw_text(print_message_error,
			glm::vec3(0.1f, 0.3f, 0.0f),
			glm::vec3(0.10f, 0.0f, 0.0f),
			glm::vec3(0.0f, 0.10f, 0.0f),
			glm::u8vec4(0x00, 0x00, 0x00, 0x00));
		//Function to match:
		
	}
	GL_ERRORS();
}

void DoneMode::reset_clay() {
}

void DoneMode::tick_clay() {
	const float kTimeStep = 0.01f;
    const float kElasticity = 0.9f;
    const float kFriction = 0.5f;
	
}

