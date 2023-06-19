#include "PlayMode.hpp"
#include "DoneMode.hpp"
#include "DrawLines.hpp"
#include "gl_errors.hpp"
#include "MenuMode.hpp"
#include "data_path.hpp"
#include "Sprite.hpp"

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


//File management
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <cstdlib>
#include <random>

PlayMode::PlayMode() {
	reset_clay();
}

PlayMode::~PlayMode() {
}

bool PlayMode::handle_event(SDL_Event const &evt, glm::uvec2 const &window_size) {

	if (evt.type == SDL_KEYDOWN) {
		if (evt.key.keysym.sym == SDLK_BACKSPACE) {
			reset_clay();
		} else if (evt.key.keysym.sym == SDLK_SPACE) {
			//tick_clay();
			interaction = true;
			//std::cout<<"interaction"<<std::endl;
		} else if(evt.key.keysym.sym == SDLK_LEFT){
			do_rotation_left = true;
		}
		else if(evt.key.keysym.sym == SDLK_RIGHT){
			do_rotation_right = true;
		}
		else if(evt.key.keysym.sym == SDLK_r){
			rigid = !rigid;
		}
		else if(evt.key.keysym.sym == SDLK_n && location == scenes_mode){
			std::cout<<"n"<<std::endl;
			
			switch (state)
			{
			case begin:
				state = inside;
				matching_start_time = std::chrono::high_resolution_clock::now();
				std::cout<<"initialize chrono change"<<std::endl;
				break;
			case inside:
				state = end;
				append_data(file_name,{"skipped",current_order.functions[current_order.curr_val].name});
				current_order.curr_val += 1 ;
				if (current_order.curr_val < current_order.functions.size()){
					prev_match.name = to_match.name;
					parse_function(current_order.functions[current_order.curr_val]);
					state = end;
				}
				std::cout<<"skipped"<<std::endl;
				break;
			case end:
				state = begin;
				std::cout<<"begin change "<<current_order.curr_val<<" "<<current_order.functions.size()<<std::endl;

				if (current_order.curr_val >= current_order.functions.size()){
					//Mode::current->init_function(current_order.function_order[current_order.curr_val]);
					std::cout<<"done with matching"<<std::endl;
					location = donemode;
					Mode::current = shared_from_this();
					//Mode::set_current(std::make_shared<DoneMode>(0,0));
				}
				break;
			}
		}
		else if(evt.key.keysym.sym == SDLK_d){
			location = donemode;
			Mode::current = shared_from_this();
			//Mode::set_current(std::make_shared<DoneMode>(done_time,err_1));
		}
		else if(evt.key.keysym.sym == SDLK_q){
			Mode::set_current(nullptr);
			return true;
		}
	} else if (evt.type == SDL_KEYUP) {
		if(evt.key.keysym.sym == SDLK_LEFT){
			do_rotation_left = false;
		}
		else if (evt.key.keysym.sym == SDLK_SPACE) {
			interaction = false;
			reset_motor();
			//std::cout<<"no interaction"<<std::endl;
		}else if(evt.key.keysym.sym == SDLK_RIGHT){
			do_rotation_right = false;
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
		do_hand_movement = false;
	} else if (evt.type == SDL_USEREVENT){
		std::vector<int>* coordinates = static_cast<std::vector<int>*>(evt.user.data1);
		num_hands  = (*coordinates)[0];
		int x1,x2,x3,x4,y1,y2,y3,y4;
		if (num_hands == 1){
			hand_1_closed = (*coordinates)[1];
			hand_1_active = (*coordinates)[2];
			//std::cout<<"closed: "<<hand_1_closed<<"  active:"<<hand_1_active<<std::endl;
			x1 = (*coordinates)[3];
			y1 = (*coordinates)[4];
			x2 = (*coordinates)[5];
			y2 = (*coordinates)[6];
			//std::cout<<"x1: "<<x1<<"  y1:"<<y1<<"  x2:"<<x2<<"  y2:"<<y2<<std::endl;
		}
		else{
			hand_1_closed = (*coordinates)[1];
			hand_1_active = (*coordinates)[2];
			x1 = (*coordinates)[3];
			y1 = (*coordinates)[4];
			x2 = (*coordinates)[5];
			y2 = (*coordinates)[6];

			hand_2_closed = (*coordinates)[7];
			hand_2_active = (*coordinates)[8];
			x3 = (*coordinates)[9];
			y3 = (*coordinates)[10];
			x4 = (*coordinates)[11];
			y4 = (*coordinates)[12];

			index_2_at.x =-1.0f + 2.0f * (x3 + 0.5f) / float(window_size.x);
			index_2_at.y = 1.0f - 2.0f * (y3 + 0.5f) / float(window_size.y);
			thumb_2_at.x = -1.0f + 2.0f * (x4 + 0.5f) / float(window_size.x);
			thumb_2_at.y = 1.0f - 2.0f * (y4 + 0.5f) / float(window_size.y);
        	//std::cout << "Coordinates: (" << x1 << ", " << y1 << " " <<x2 << ", " << y2 << ")"<< "(" << x3 << ", " << y3 << " " <<x4 << ", " << y4 << ")"<<std::endl;
		}

        delete coordinates;
		index_1_at.x =-1.0f + 2.0f * (x1 + 0.5f) / float(window_size.x);
		index_1_at.y = 1.0f - 2.0f * (y1 + 0.5f) / float(window_size.y);
		thumb_1_at.x = -1.0f + 2.0f * (x2 + 0.5f) / float(window_size.x);
		thumb_1_at.y = 1.0f - 2.0f * (y2 + 0.5f) / float(window_size.y);
		do_hand_movement = true;
	}

	return false;
}
float l2_norm(std::vector<double> const& u) {
    float accum = 0.;
    for (float x : u) {
        accum += x * x;
    }
    return sqrt(accum);
}

float get_error_from_l2norm(std::vector<double> &coeff,uint8_t &order, std::vector<double> &err,std::vector<double> x, std::vector<double> y){
	if (order == 1){
		for(uint32_t p = 0; p < x.size(); ++ p)
		{
			double cerr = y.at(p) - (coeff[0] + coeff[1]*x.at(p));
			// std::cout<< vfitted<<", ";
			err.push_back(cerr);
		}
	}
	else if (order ==2){
		for(uint32_t p = 0; p < x.size(); ++ p)
		{
			double cerr = y.at(p) - (coeff[0] + coeff[1]*x.at(p)+coeff[2]*x.at(p)*x.at(p));
			// std::cout<< vfitted<<", ";
			err.push_back(cerr);
		}
	}
	return l2_norm(err);

}

void PlayMode::polyfit(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &coeff,float &l2_error,uint8_t order){
	// Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for example k = 3 for cubic polynomial
	//Fit a line
	Eigen::MatrixXd X(x.size(), order + 1);
	Eigen::VectorXd Y = Eigen::VectorXd::Map(&y.front(), y.size());
	Eigen::VectorXd result;

	// Populate the matrix
	for(size_t i = 0 ; i < x.size(); ++i)
	{
		for(size_t j = 0; j < order + 1.0; ++j)
		{
			X(i, j) = pow(x.at(i), j);
		}
	}

	// Solve for linear least square fit
	result  = X.householderQr().solve(Y);
	coeff.resize(order+1);
	for (int k = 0; k < order+1; k++)
	{
		coeff[k] = result[k];
	}
	std::vector<double> err;
	l2_error =  get_error_from_l2norm(coeff,order,err,x,y);
	//l2_error = l2_norm(err);
}
void PlayMode::enter_scene(float elapsed) {

    //////////////////////////////////////////////////  menu staff   /////////////////////////////////////////////////////////////
    if(location!=gamescene && location != scenes_mode){
        std::vector< MenuMode::Item > items;
        glm::vec2 at(0.1f, 0.5f);
		glm::vec2 at2(0.1f, 0.1f);
        auto add_text = [&items,&at](std::string text) {
            items.emplace_back(text, nullptr, 0.1f, nullptr, at);
            at.y -= 0.1f;
        };
        auto add_text2 = [&items,&at](std::string text) {
            items.emplace_back(text, nullptr, 0.1f, nullptr, at);
            at.y -= 0.12f;
        };

        auto add_choice = [&items,&at](std::string text, std::function< void(MenuMode::Item const &) > const &fn) {
            items.emplace_back(text, nullptr, 0.1f, fn, at + glm::vec2(0.1f, 0.0f));
            at.y -= 0.15f;
        };

        auto add_choice2 = [&items,&at](std::string text, std::function< void(MenuMode::Item const &) > const &fn) {
            items.emplace_back(text, nullptr, 0.1f, fn, at + glm::vec2(0.1f, 0.0f));
            at.y -= 0.15f;
        };
        
		if (location == mainmenu) {
            at = glm::vec2(0.300f, 0.90f);
            add_text("WELCOME   TO   MATHCLAY");
            add_text(" ");
            at.y -= 0.01; //gap before choices

			//First column of options
			at2 = at; //Save the current value
            at = glm::vec2(-0.1f, at.y);
			add_text("Select a Mode:");
            //add_text(" ");
			at.y -= 0.01; //gap before choices

            add_choice("TRAINING MODE !", [this](MenuMode::Item const &){
                location = gamescene;
                Mode::current = shared_from_this();
				Mode::current->init_function("None");
 				reset_clay();
            });
			add_choice("Y = -3", [this](MenuMode::Item const &){
                location = gamescene;
                Mode::current = shared_from_this();
				Mode::current->init_function("-3");
				reset_clay();

            });
			add_choice("Y = x", [this](MenuMode::Item const &){
                location = gamescene;
                Mode::current = shared_from_this();
				Mode::current->init_function("x");
				reset_clay();
            });
			add_choice("Y = x^2", [this](MenuMode::Item const &){
                location = gamescene;
                Mode::current = shared_from_this();
				Mode::current->init_function("x^2");
				reset_clay();
            });
			
			//Second column of options
            at = at2; //Assign the saved value
			at.x = 1.0f;
			add_text("Select a scene:");
            //add_text(" ");
			at.y -= 0.01; //gap before choices

			add_choice("Line Intercept", [this](MenuMode::Item const &){
                //location = instructions;
				// next_location = scenes_mode;
				location = scenes_mode;
				scene  = line_intercept;
				state = begin;
                Mode::current = shared_from_this();
				std::vector<std::vector<double>> function_coeffs = {
					{0.0,0.0,-3.0}, //y = ax^2 + bx + c
					// {0.0,0.0,-1.0},
					{0.0,0.0,0.0},
					// {2,0,0},
					{0.0,0.0,1.0},
					{0.0,0.0,2.0},
				};
				current_order = scene_order(function_coeffs);
				parse_function(current_order.functions[0]);
				time_fixed = true;

				reset_clay();
            });
			add_choice("Line Slope", [this](MenuMode::Item const &){
                location = scenes_mode;
				scene  = line_slope;
                Mode::current = shared_from_this();
				std::vector<std::vector<double>> function_coeffs = {
					{0,-3.0,0}, //ax^2 + bx + c
					{0,-2.0,0},
					{0,-1.0,0},
					{0,1.0,0},
					{0,2.0,0.0},
					// {0,0,2},
				};
				current_order = scene_order(function_coeffs);
				parse_function(current_order.functions[0]);
				std::cout<<"function parsed slope"<<std::endl;
				time_fixed = true;
				reset_clay();
            });
			add_choice("Parabola Concavity", [this](MenuMode::Item const &){
                location = scenes_mode;
				scene = parabola_concavity;
                Mode::current = shared_from_this();
				std::vector<std::vector<double>> function_coeffs = {
					{0.5,0,0}, //ax^2 + bx + c
					// {0,0,-2},
					{0.1,0,0},
					{-0.1,0,0},
					{-0.5,0,0},
					// {0,0,2},
				};
				current_order = scene_order(function_coeffs);
				parse_function(current_order.functions[0]);				
				reset_clay();
            });
			add_choice("Parabola Vertex", [this](MenuMode::Item const &){
                location = scenes_mode;
				scene = parabola_vertex;
                Mode::current = shared_from_this();
				std::vector<std::string> function_names = {
					"(x-4)^2",
					"(x-2)^2",
					"(x+0)^2",
					"(x+3)^2",
					"(x+5)^2",
				};
				current_order = scene_order(function_names);
				parse_function(current_order.functions[0]);				
				reset_clay();

			});
			at.x = 0.45f;
			add_choice("Exit Game", [this](MenuMode::Item const &){
				Mode::set_current(nullptr);
				return;
			});
            // // add_choice("CREDIT", [this](MenuMode::Item const &){
            //     location = credit;
            //     Mode::current = shared_from_this();
            // });

        } else if (location == credit) {
            at=glm::vec2(0.300f, 0.9f);
            add_text("MUSIC:");
            add_text2("FREEPD.COM    AND   FREESOUND.ORG");
            add_text(" ");
            add_text("IMAGE   ASSET:");
            add_text2("ALL   ICON");
            add_text(" ");
            add_text("FONT:");
            add_text2("SOURCEFORGE.NET   RODS-CUSTOM-FONT-XCF-FILES");
            add_text(" ");
            add_choice("BACK TO MAIN MENU", [this](MenuMode::Item const &){
                location = mainmenu;
                Mode::current = shared_from_this();
            });
        }
		else if (location == instructions) {
            at=glm::vec2(0.000f, 0.9f);
            add_text("INSTRUCTIONS:");
            add_text2("You will have 1 minute to sculpt the function");
            add_text2("If you believe you are done, press N to continue");
            add_text(" ");
            add_text(" ");
            add_choice("Go", [this](MenuMode::Item const &){
                location = next_location;
                Mode::current = shared_from_this();
            });
        }
		else if(location == donemode){
            at=glm::vec2(0.300f, 0.9f);
			auto final_time = std::chrono::high_resolution_clock::now();
			auto done_time = std::chrono::duration< double >(final_time - start_time).count();
			std::string print_message;
			print_message.append("Elapsed time ");
			print_message.append(std::to_string(int(done_time)));
			print_message.append(" seconds");
			add_text(print_message);
            add_text(" ");
            add_choice("BACK TO MAIN MENU", [this](MenuMode::Item const &){
                location = mainmenu;
                Mode::current = shared_from_this();
            });
		}
        std::shared_ptr< MenuMode > menu = std::make_shared< MenuMode >(items);
        // menu->left_select = sprite_left_select;
        // menu->right_select = sprite_right_select;
        menu->view_min = view_min;
        menu->view_max = view_max;
        menu->background = shared_from_this();
        Mode::current = menu;
	}
}
void PlayMode::update(float elapsed) {
	time_acc += elapsed;
	if (Mode::current.get() == this) {
			//there is no menu displayed! Make one:
			enter_scene(elapsed);
	}
	
	auto before = std::chrono::high_resolution_clock::now();
	uint32_t ticks = 0;
	while (time_acc > 0.0f) {
		time_acc -= ClayTick;

		if (do_pinch) {
			probe_pinch = std::min(1.0f, probe_pinch + ClayTick / 0.5f);
			//std::cout<<"probe_pinch"<<std::endl;
		} else {
			probe_pinch = std::max(0.0f, probe_pinch - ClayTick / 0.5f);
		}
		if (do_rotation_left) {
			probe_rot += 0.001f;
		}
		if (do_rotation_right) {
			probe_rot -= 0.001f;
		}
		if (do_hand_movement){
			//std::cout<<"hand found"<<std::endl;
			//std::cout<<"index_1_at: "<<index_1_at.x<<" "<<index_1_at.y<<"  thumb_1_at: "<<thumb_1_at.x<<" "<<thumb_1_at.y<<std::endl;
			glm::vec2 at_1, at_2;
			at_2.x = (index_1_at.x - world_to_clip[3][0]) / world_to_clip[0][0];
			at_2.y = (index_1_at.y - world_to_clip[3][1]) / world_to_clip[1][1];
			at_1.x = (thumb_1_at.x - world_to_clip[3][0]) / world_to_clip[0][0];
			at_1.y = (thumb_1_at.y - world_to_clip[3][1]) / world_to_clip[1][1];
			//The number 10.f is the min gap that can happen between the probes
			glm::vec2 gap = glm::mix(1.0f * particle_radius + 2.0f * probe_radius, 2.0f * particle_radius + 2.0f * probe_radius, probe_pinch) * glm::vec2(-std::sin(probe_rot), std::cos(probe_rot));

			if (num_hands == 1 && (hand_1_closed==0)){
				if (probes.size() != 2) {
					probes.assign(2, Probe());
					probes[0].pos = at_1 - 0.5f * gap; //I'm not sure what the 0.5f does
					probes[1].pos = at_2 + 0.5f * gap;
				}
				probes[0].target = at_1 - 0.5f * gap;
				probes[1].target = at_2 + 0.5f * gap;
				if (hand_1_active==0){
					probes[0].active = 0;
					probes[1].active = 0;
				}
				else{
					probes[0].active = 1;
					probes[1].active = 1;
				}
				float len = glm::length(probes[1].pos - probes[0].pos);
				// std::cout<<len<<std::endl;
				if (interaction==false){
					//std::cout<<"interaction false "<<std::endl;
					probes[0].active = 0;
					probes[1].active = 0;
				}
				else{
					probes[0].active = 1;
					probes[1].active = 1;
				}

			} else if (num_hands == 2){
				glm::vec2 at_3, at_4;
				at_4.x = (index_2_at.x - world_to_clip[3][0]) / world_to_clip[0][0];
				at_4.y = (index_2_at.y - world_to_clip[3][1]) / world_to_clip[1][1];
				at_3.x = (thumb_2_at.x - world_to_clip[3][0]) / world_to_clip[0][0];
				at_3.y = (thumb_2_at.y - world_to_clip[3][1]) / world_to_clip[1][1];
				//std::cout<<"at.x: "<<at_3.x<<" "<<at_3.y<<std::endl;
				//probes[0].target = at_1;
				//probes[1].target = at_2;
				if (hand_1_closed == true && hand_2_closed == false){
					if (probes.size() != 2) {
						probes.assign(2, Probe());
						probes[0].pos = at_3 - 0.5f * gap; //I'm not sure what the 0.5f does
						probes[1].pos = at_4 + 0.5f * gap;
					}

					probes[0].target = at_3 - 0.5f * gap;
					probes[1].target = at_4 + 0.5f * gap;
					if (hand_2_active==0){
						probes[0].active = 0;
						probes[1].active = 0;
					}
					else{
						probes[0].active = 1;
						probes[1].active = 1;
					}
				}
				else if (hand_1_closed == false && hand_2_closed == true){
					if (probes.size() != 2) {
						probes.assign(2, Probe());
						probes[0].pos = at_1 - 0.5f * gap; //I'm not sure what the 0.5f does
						probes[1].pos = at_2 + 0.5f * gap;
					}

					probes[0].target = at_1 - 0.5f * gap;
					probes[1].target = at_2 + 0.5f * gap;
					if (hand_1_active==0){
						probes[0].active = 0;
						probes[1].active = 0;
					}
					else{
						probes[0].active = 1;
						probes[1].active = 1;
					}
				}
				else if (hand_1_closed == true && hand_2_closed == true){
					continue;
				}
				else {
					if (probes.size() != 4) {
						probes.assign(4, Probe());
						probes[0].pos = at_1 - 0.5f * gap; //I'm not sure what the 0.5f does
						probes[1].pos = at_2 + 0.5f * gap;
					
						probes[2].pos = at_3 - 0.5f * gap; //I'm not sure what the 0.5f does
						probes[3].pos = at_4 + 0.5f * gap;
					}
					probes[0].target = at_1 - 0.5f * gap;
					probes[1].target = at_2 + 0.5f * gap;
					probes[2].target = at_3 - 0.5f * gap;
					probes[3].target = at_4 + 0.5f * gap;
					
					probes[0].active = hand_1_active;
					probes[1].active = hand_1_active;
					probes[2].active = hand_2_active;
					probes[3].active = hand_2_active;

				}
			}

		}
		else if (mouse_at.x == mouse_at.x) {
			glm::vec2 at;
			at.x = (mouse_at.x - world_to_clip[3][0]) / world_to_clip[0][0];
			at.y = (mouse_at.y - world_to_clip[3][1]) / world_to_clip[1][1];

			//glm::vec2 gap = glm::mix(10.0f * particle_radius + 2.0f * probe_radius, 2.0f * particle_radius + 2.0f * probe_radius, probe_pinch) * glm::vec2(-std::sin(probe_rot), std::cos(probe_rot));
			glm::vec2 gap = glm::mix(30.0f * particle_radius + 2.0f * probe_radius, 2.0f * particle_radius + 2.0f * probe_radius, probe_pinch) * glm::vec2(-std::sin(probe_rot), std::cos(probe_rot));

			if (probes.size() != 2) {
				probes.assign(2, Probe());
				probes[0].pos = at - 0.5f * gap;
				probes[1].pos = at + 0.5f * gap;
			}
			probes[0].target = at - 0.5f * gap;
			probes[1].target = at + 0.5f * gap;
			if (interaction==false){
				//std::cout<<"interaction false "<<std::endl;
				probes[0].active = 0;
				probes[1].active = 0;
			}
			else{
				probes[0].active = 1;
				probes[1].active = 1;
			}
		} else {
			probes.clear();
		}

		tick_clay();
		++ticks;
	}
	auto after = std::chrono::high_resolution_clock::now();
	ticks_acc += ticks;
	ticks_acc_filter += ticks;
	duration_acc += std::chrono::duration< double >(after - before).count();

	if (ticks_acc > 5.0f / ClayTick) {
		double ms_per_tick = duration_acc * 1000.0f / ticks_acc;
		std::cout << "On " << particles.size() << " particles, ran " << ticks_acc << " ticks in " << duration_acc << " seconds. That's " << ms_per_tick << " ms per tick, or " << (1000.0f * ClayTick) / ms_per_tick << "x real time." << std::endl;
		ticks_acc = 0;
		duration_acc = 0.0;
	}
}

void PlayMode::init_serial(std::string port_name){
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
void PlayMode::init_file(std::string input_file_name){
	// Open the hardware serial ports.
	//function_to_match = function_name;
	this->file_name = input_file_name;
	std::cout<<"file name "<<file_name<<std::endl;
	// Set up random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 99);
	// Reopen the file in append mode
    std::ofstream fileAppend(file_name, std::ios::app);

    if (!fileAppend) {
        std::cerr << "Failed to open the file for appending: " << file_name << std::endl;
        return;
    }

    // // Write two more rows of random data
    // for (int i = 0; i < 2; ++i) {
    //     fileAppend << dis(gen) << ',' << dis(gen) << ',' << dis(gen) << '\n';
    // }

    // Close the file again
    fileAppend.close();

    std::cout << "Two more rows added to the file." << std::endl;	
}

void PlayMode::append_data(std::string input_file_name, std::vector<std::string> data){
	// Reopen the file in append mode
    std::ofstream fileAppend(file_name, std::ios::app);

    if (!fileAppend) {
        std::cerr << "Failed to open the file for appending: " << file_name << std::endl;
        return;
    }

    // Append new row of data with out an extra comma at the end
	for (int i = 0; i < data.size()-1; ++i) {
		fileAppend << data[i] << ',';
	}
    fileAppend<< data[data.size()-1]<<'\n';
    
    // Close the file again
    fileAppend.close();
}


void PlayMode::parse_function(function &selected_function){
	// Open the hardware serial ports.
	//std::cout<<"Named function: y = "<<selected_function.name<<std::endl;
	to_match.name = "x";
	//to_match.name = selected_function.name;
	glm::vec2 start, end;
	if (selected_function.name != "None"){
		if (selected_function.real_coeff[0] ==0.0f){
			if (selected_function.real_coeff[1] ==0.0f){
				to_match.order = 1;
				start = glm::vec2(0.0f, center_axis.y + selected_function.real_coeff[2]*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y + selected_function.real_coeff[2]*unit_line_spacing); 
			}
			else if (selected_function.real_coeff[0]==0.0f){
				to_match.order = 1;
				start = glm::vec2(center_axis.x - unit_line_spacing* 1, center_axis.y - unit_line_spacing * selected_function.real_coeff[1]);
				end = glm::vec2(center_axis.x + unit_line_spacing* 1, center_axis.y + unit_line_spacing * selected_function.real_coeff[1]);
			}
			double A = (end.y-start.y)/(end.x-start.x);
			double B = end.y- A*end.x;
			std::cout << "Coefficients: {" << A << ", " << B << "}" << std::endl;
			to_match.coeff = {B,A};
			selected_function.coeff = {B,A};
			// to_match.coeff_offset = {B,A};	
		}
		else if (selected_function.real_coeff[0] != 0.0f){
			// std::cout<<"Match order 2"<<std::endl;
			to_match.order = 2;
			glm::vec2 middle;
			double A, B, C;
			std::cout<<"New origin: ("<<center_axis.x<<","<<center_axis.y<<")"<<std::endl;
			std::cout << "Coefficients real: {" << selected_function.real_coeff[0]
			 << ", " << selected_function.real_coeff[1] << ", " << selected_function.real_coeff[2] << "}" << std::endl;

			A = selected_function.real_coeff[0] * 1/unit_line_spacing;
			// B = selected_function.real_coeff[1] * 1/unit_line_spacing;
			// C = selected_function.real_coeff[2] * 1/unit_line_spacing;
			B = selected_function.real_coeff[1];
			C = selected_function.real_coeff[2]*unit_line_spacing;

			double new_B = B - 2 * A * center_axis.x;
			C = C + A * center_axis.x * center_axis.x - B * center_axis.x + center_axis.y;
			// C = C + A * center_axis.x * center_axis.y * center_axis.y;

			std::cout << "Coefficients temp: {" << A << ", " << new_B << ", " << C << "}" << std::endl;
			// start = glm::vec2(center_axis.x - unit_line_spacing * 1, center_axis.y + unit_line_spacing * (1*1) * selected_function.real_coeff[0]
			// 		+ unit_line_spacing * 1 * selected_function.real_coeff[1] + selected_function.real_coeff[2]);
			// end = glm::vec2(center_axis.x + unit_line_spacing * 1, center_axis.y + unit_line_spacing * (1*1) *selected_function.real_coeff[0]
			// 		+ unit_line_spacing * 1 * selected_function.real_coeff[1] + selected_function.real_coeff[2]);	
			// middle = glm::vec2(center_axis.x, center_axis.y + unit_line_spacing * (0*0) * selected_function.real_coeff[0]
			// 		+ unit_line_spacing * 0 * selected_function.real_coeff[1] + selected_function.real_coeff[2]);
			
			// // Compute the coefficients of the parabola that passes through these points.
			// double denom = (start.x - middle.x) * (start.x - end.x) * (middle.x - end.x);
			// A     = (end.x * (middle.y - start.y) + middle.x * (start.y - end.y) + start.x * (end.y - middle.y)) / denom;
			// B     = (end.x*end.x * (start.y - middle.y) + middle.x*middle.x * (end.y - start.y) + start.x*start.x * (middle.y - end.y)) / denom;
			// C     = (middle.x * end.x * (middle.x - end.x) * start.y + end.x * start.x * (end.x - start.x) * middle.y + start.x * middle.x * (start.x - middle.x) * end.y) / denom;
			
		
			// std::cout << "Coefficients: {" << A << ", " << B << ", " << C << "}" << std::endl;
			to_match.coeff = {C,new_B,A}; //ax^2+bx+c returning in the opposite order
			selected_function.coeff = {C,new_B,A};
			// to_match.coeff_offset = {C+0.17,B,A}; //ax^2+bx+c returning in the opposite order
		}
		//time_fixed = true;
	}
}

void PlayMode::init_function(std::string function_name){
	// Open the hardware serial ports.
	//function_to_match = function_name;
	to_match.name = function_name;
	glm::vec2 start, end;
	if (to_match.name!= "None"){
		if (function_name == "-3" || function_name == "-1" || 
			function_name == "0" ||
			function_name == "1" || function_name == "2" || function_name == "3"){
			to_match.order = 1;
			if (function_name == "-3"){
				start = glm::vec2(0.0f, center_axis.y - 3*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y - 3*unit_line_spacing); 
			}
			else if (function_name == "-2"){
				start = glm::vec2(0.0f, center_axis.y - 2*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y - 2*unit_line_spacing); 
			}
			else if (function_name == "-1"){
				start = glm::vec2(0.0f, center_axis.y - 1*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y - 1*unit_line_spacing); 
			}
			else if (function_name == "0"){
				start = glm::vec2(0.0f, center_axis.y - 0*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y - 0*unit_line_spacing); 
			}
			else if (function_name == "1"){
				start = glm::vec2(0.0f, center_axis.y + 1*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y + 1*unit_line_spacing); 
			}
			else if (function_name == "2"){
				start = glm::vec2(0.0f, center_axis.y + 2*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y +2*unit_line_spacing); 
			}
			else if (function_name == "3"){
				start = glm::vec2(0.0f, center_axis.y + 3*unit_line_spacing);
				end = glm::vec2(1.5f, center_axis.y +3*unit_line_spacing); 
			}
			double A = (end.y-start.y)/(end.x-start.x);
			double B = end.y- A*end.x;
			std::cout << "Coefficients: {" << A << ", " << B << "}" << std::endl;
			to_match.coeff = {B,A};
			// to_match.coeff_offset = {B-0.07,A}; 
		}
		else if (function_name == "-x^2" || function_name == "x^2"
			|| function_name == "-2x^2" || function_name == "2x^2"
			|| function_name == "-3x^2" || function_name == "3x^2"
			
			){
			to_match.order = 2;
			glm::vec2 middle;
			// Three points to pass through.
			if (function_name == "3x^2"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y + 0.1 * 3);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y + 0.1 * 3);	
			}
			if (function_name == "2x^2"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y + 0.1 * 2);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y + 0.1 * 2);	
			}
			else if (function_name == "x^2"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y + 0.1 * 1);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y + 0.1 * 1);	
			}
			else if (function_name == "-x^2"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y - 0.1 * 2);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y - 0.1 * 2);	
			}
			else if (function_name == "-2x^2"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y - 0.1 * 2);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y - 0.1 * 2);	
			}
			else if (function_name == "-3x^2"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y - 0.1 * 3);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y - 0.1 * 3);	
			}
			middle = glm::vec2(0.75f, 0.5f);
			// Compute the coefficients of the parabola that passes through these points.
			double denom = (start.x - middle.x) * (start.x - end.x) * (middle.x - end.x);
			double A     = (end.x * (middle.y - start.y) + middle.x * (start.y - end.y) + start.x * (end.y - middle.y)) / denom;
			double B     = (end.x*end.x * (start.y - middle.y) + middle.x*middle.x * (end.y - start.y) + start.x*start.x * (middle.y - end.y)) / denom;
			double C     = (middle.x * end.x * (middle.x - end.x) * start.y + end.x * start.x * (end.x - start.x) * middle.y + start.x * middle.x * (start.x - middle.x) * end.y) / denom;
			std::cout << "Coefficients: {" << A << ", " << B << ", " << C << "}" << std::endl;
			to_match.coeff = {C,B,A}; //ax^2+bx+c returning in the opposite order
			// to_match.coeff_offset = {C+0.17,B,A}; //ax^2+bx+c returning in the opposite order
		}
		else if (function_name == "-3x" || function_name == "-2x" || 
			function_name == "-x" || function_name == "x" || 
			function_name == "2x" || function_name == "3x"){
			to_match.order = 1;
			if (function_name == "x"){
				start = glm::vec2(center_axis.x - 0.1* 1, center_axis.y - 0.1* 1);
				end = glm::vec2(center_axis.x + 0.1* 1, center_axis.y + 0.1* 1);	
			}
			else if (function_name == "2x"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y - 0.1 * 2);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y + 0.1 * 2);	
			}
			else if (function_name == "3x"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y - 0.1 * 3);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y + 0.1 * 3);	
			}
			else if (function_name == "-x"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y + 0.1 * 1);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y - 0.1 * 1);	
			}
			else if (function_name == "-2x"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y + 0.1 * 2);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y - 0.1 * 2);	
			}
			else if (function_name == "-3x"){
				start = glm::vec2(center_axis.x - 0.1 * 1, center_axis.y + 0.1 * 3);
				end = glm::vec2(center_axis.x + 0.1 * 1, center_axis.y - 0.1 * 3);	
			}
			double A = (end.y-start.y)/(end.x-start.x);
			double B = end.y- A*end.x;
			std::cout << "Coefficients: {" << A << ", " << B << "}" << std::endl;
			to_match.coeff = {B,A};
			// to_match.coeff_offset = {B,A};
		}
		//time_fixed = true;
	}
}

bool PlayMode::areVectorsApproximatelyEqual(const std::vector<double>& v1, const std::vector<double>& v2, double tolerance) {
    if (v1.size() != v2.size()) {
        return false;
    }
    for (std::size_t i = 0; i < v1.size(); ++i) {
        if (std::abs(v1[i] - v2[i]) > tolerance) {
            return false;
        }
    }
    return true;
}
void PlayMode::close_serial(){
	// Close the serial ports
	if (serial_port_name != "None"){
		serial_port.Write("i0l");
		std::string read_byte_1;
		//serial_port.Read(read_byte_1,19);
		serial_port.Close();
		std::cout<<"serial port closed "<<read_byte_1;
	}
}
void PlayMode::reset_motor(){
	// Close the serial ports
	if (serial_port_name != "None"){
		serial_port.Write("i0l");
	}
}
void PlayMode::set_flags(std::string mode){
	// Close the serial ports
	if (mode=="scenes"){
		show_to_match_line_on_finished = true;
		time_fixed = true;
		show_fitted_line = true;
	}
}
void PlayMode::draw(glm::uvec2 const &drawable_size) {
	glClearColor(0.9f, 0.9f, 0.87f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	if(location!=gamescene && location != scenes_mode) {
		//draw.draw(*sprite_menubackground, glm::vec2(0,0));
		return;
	}
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


	DrawLines lines(world_to_clip);
	//GRID
	{
		//Horizontal lines along the y axis
		for(int i = 0; i< 10;i++){
			lines.draw(glm::vec3(axis_min.x, center_axis.y + i* unit_line_spacing/2.0f, 0.0f), 
						glm::vec3(axis_max.x, center_axis.y  + i * unit_line_spacing/2.0f, 0.0f), 
						colors.light_gray); //horizontal line positive
			lines.draw(glm::vec3(axis_min.x, center_axis.y - i* unit_line_spacing/2.0f, 0.0f), 
						glm::vec3(axis_max.x, center_axis.y - i * unit_line_spacing/2.0f, 0.0f), 
						colors.light_gray); //horizontal line
		}
		//Vertical lines along the x axis
		for(int i = 0; i< 14;i++){
			lines.draw(glm::vec3(center_axis.x + i*unit_line_spacing/2.0f, axis_min.y, 0.0f), 
						glm::vec3(center_axis.x + i*unit_line_spacing/2.0f, axis_max.y, 0.0f), 
						colors.light_gray); //vertical line
			lines.draw(glm::vec3(center_axis.x - i*unit_line_spacing/2.0f, axis_min.y, 0.0f), 
						glm::vec3(center_axis.x - i*unit_line_spacing/2.0f, axis_max.y, 0.0f), 
						colors.light_gray); //vertical line
		}
	}


	//boundary:
	lines.draw(glm::vec3(box_min.x, box_min.y, 0.0f), glm::vec3(box_max.x, box_min.y, 0.0f), colors.red);
	lines.draw(glm::vec3(box_max.x, box_min.y, 0.0f), glm::vec3(box_max.x, box_max.y, 0.0f), colors.red);
	lines.draw(glm::vec3(box_max.x, box_max.y, 0.0f), glm::vec3(box_min.x, box_max.y, 0.0f), colors.red);
	lines.draw(glm::vec3(box_min.x, box_max.y, 0.0f), glm::vec3(box_min.x, box_min.y, 0.0f), colors.red);

	//axis:
	lines.draw_bold(glm::vec3(center_axis.x, axis_min.y, 0.0f), glm::vec3(center_axis.x, axis_max.y, 0.0f), colors.black,0.0001f); //horizontal line
	lines.draw_bold(glm::vec3(axis_min.x, center_axis.y, 0.0f), glm::vec3(axis_max.x, center_axis.y, 0.0f), colors.black,0.0001); //vertical line

	//axis tags
	lines.draw_text("x",
		glm::vec3(axis_max.x + line_half_lenght, center_axis.y, 0.0f),
		glm::vec3(0.05f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.05f, 0.0f),
		glm::u8vec4(0x00, 0x00, 0x00, 0x00));
	lines.draw_text("y",
		glm::vec3(center_axis.x, axis_max.y + line_half_lenght, 0.0f),
		glm::vec3(0.05f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.05f, 0.0f),
		glm::u8vec4(0x00, 0x00, 0x00, 0x00));


	//Horizontal mini lines along the y axis
	const float text_height = 0.025f;
	for(int i = 1; i< 5;i++){
		lines.draw_bold(glm::vec3(center_axis.x - line_half_lenght, center_axis.y + i* unit_line_spacing, 0.0f), 
					glm::vec3(center_axis.x + line_half_lenght, center_axis.y  + i * unit_line_spacing, 0.0f), 
					glm::u8vec4(0x08, 0x44, 0x44, 0xff),0.001); //horizontal line positive
		lines.draw_bold(glm::vec3(center_axis.x - line_half_lenght, center_axis.y - i* unit_line_spacing, 0.0f), 
					glm::vec3(center_axis.x + line_half_lenght, center_axis.y - i * unit_line_spacing, 0.0f), 
					glm::u8vec4(0x08, 0x44, 0x44, 0xff),0.001); //horizontal line
	
		if (i>0)
			lines.draw_text(std::to_string(int(i)),
				glm::vec3(center_axis.x + line_half_lenght*1.5, center_axis.y + i* unit_line_spacing- text_height/2.0, 0.0f),
				glm::vec3(text_height, 0.0f, 0.0f),
				glm::vec3(0.0f, text_height, 0.0f),
				glm::u8vec4(0x00, 0x00, 0x00, 0x00));
	}
	//Vertical mini lines along the x axis
	for(int i = 0; i< 7;i++){
		lines.draw_bold(glm::vec3(center_axis.x + i*unit_line_spacing, center_axis.y  + line_half_lenght, 0.0f), 
					glm::vec3(center_axis.x + i*unit_line_spacing, center_axis.y  - line_half_lenght, 0.0f), 
					glm::u8vec4(0x08, 0x44, 0x44, 0xff),0.001); //vertical line
		lines.draw_bold(glm::vec3(center_axis.x - i*unit_line_spacing, center_axis.y  + line_half_lenght, 0.0f), 
					glm::vec3(center_axis.x - i*unit_line_spacing, center_axis.y  - line_half_lenght, 0.0f), 
					glm::u8vec4(0x08, 0x44, 0x44, 0xff),0.001); //vertical line
		if (i>0)
			lines.draw_text(std::to_string(int(i)),
				glm::vec3(center_axis.x + i*unit_line_spacing + text_height/4.0, center_axis.y  + line_half_lenght, 0.0f),
				glm::vec3(text_height, 0.0f, 0.0f),
				glm::vec3(0.0f, text_height, 0.0f),
				glm::u8vec4(0x00, 0x00, 0x00, 0x00));
	}



	//from 15-466-f22-base6:
	static std::array< glm::vec2, 16 > const circle = [](){
		std::array< glm::vec2, 16 > ret;
		for (uint32_t a = 0; a < ret.size(); ++a) {
			float ang = a / float(ret.size()) * 2.0f * float(M_PI);
			ret[a] = glm::vec2(std::cos(ang), std::sin(ang));
		}
		return ret;
	}();
	//draw particles
	auto draw_circle = [&](glm::vec2 const &center, float radius, glm::u8vec4 color) {
		for (uint32_t a = 0; a < circle.size(); ++a) {
			lines.draw(
				glm::vec3(center + radius * circle[a], 0.0f),
				glm::vec3(center + radius * circle[(a+1)%circle.size()], 0.0f),
				color
			);
		}
	};

	if (location == gamescene){
		{
			//particles:
			//double avg_x = 0;
			//double avg_y = 0;
			std::vector<double> xs;
			std::vector<double> ys;
			for (auto const &p : particles) {
				draw_circle(p.pos, particle_radius, colors.green);
				xs.push_back(p.pos[0]);
				ys.push_back(p.pos[1]);
			}
			if (ticks_acc % fit_step == 0){
				if (to_match.name != "None"){
					if (to_match.order ==1){
						//Fit line and compute error
						std::vector<double> coeff_1;
						//boost::thread t(polyfit(xs, ys, coeff_1, err_1,1));
						boost::thread t1(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_1), boost::ref(err_1), 1);
						t1.join();
						//std::cout<<"error_1: "<<err_1<<"  error_2: "<<err_2<<std::endl;
						std::string curr_fitted_text ="";
						fitted.order = 1;
						if (areVectorsApproximatelyEqual(coeff_1,fitted.coeff,0.01) == false){
							fitted.coeff = coeff_1;
							curr_fitted_text.append(std::to_string(fitted.coeff[1]));
							curr_fitted_text.append(" x + ");
							curr_fitted_text.append(std::to_string(fitted.coeff[0]));
							fitted.name = curr_fitted_text;
						}
					}
					else if (to_match.order ==2){
						//Fit line and compute error
						std::vector<double> coeff_2;
						//Fit parabola and compute error
						boost::thread t2(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_2), boost::ref(err_2), 2);
						t2.join();
						std::string curr_fitted_text ="";
						fitted.order = 2;
						if (areVectorsApproximatelyEqual(coeff_2,fitted.coeff,0.01) == false){
							fitted.coeff = coeff_2;
							curr_fitted_text.append(std::to_string(fitted.coeff[2]));
							curr_fitted_text.append(" x^2 + ");
							curr_fitted_text.append(std::to_string(fitted.coeff[1]));
							curr_fitted_text.append(" x + ");
							curr_fitted_text.append(std::to_string(fitted.coeff[0]));
							fitted.name = curr_fitted_text;
						}
						
					}
				}
				else{
					//Fit line and compute error
					std::vector<double> coeff_1,coeff_2;
					boost::thread t1(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_1), boost::ref(err_1), 1);
					
					//Fit parabola and compute error
					boost::thread t2(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_2), boost::ref(err_2), 2);
					t1.join();
					t2.join();
					//std::cout<<"error_1: "<<err_1<<"  error_2: "<<err_2<<std::endl;
					std::string curr_fitted_text ="";
					if(err_1-1 < err_2){
						fitted.order = 1;
						if (areVectorsApproximatelyEqual(coeff_1,fitted.coeff,0.01) == false){
							fitted.coeff = coeff_1;
							curr_fitted_text.append(std::to_string(fitted.coeff[1]));
							curr_fitted_text.append(" x + ");
							curr_fitted_text.append(std::to_string(fitted.coeff[0]));
							fitted.name = curr_fitted_text;
						}
					}
					else{
						fitted.order = 2;
						if (areVectorsApproximatelyEqual(coeff_2,fitted.coeff,0.01) == false){
							fitted.coeff = coeff_2;
							curr_fitted_text.append(std::to_string(fitted.coeff[2]));
							curr_fitted_text.append(" x^2 + ");
							curr_fitted_text.append(std::to_string(fitted.coeff[1]));
							curr_fitted_text.append(" x + ");
							curr_fitted_text.append(std::to_string(fitted.coeff[0]));
							fitted.name = curr_fitted_text;
						}
					}
				}
			}
			if (show_function_name){
				lines.draw_text(fitted.name,
					glm::vec3(0.9f, 0.1f, 0.0f),
					glm::vec3(0.05f, 0.0f, 0.0f),
					glm::vec3(0.0f, 0.05f, 0.0f),
					glm::u8vec4(0x00, 0x00, 0x00, 0x00));
			}
			//Function to match
			if (to_match.name != "None"){
				if (to_match.order == 1){
					lines.draw(glm::vec3(axis_min.x, (axis_min.x)*to_match.coeff[1] + to_match.coeff[0], 0.0f), glm::vec3(axis_max.x, axis_max.x*to_match.coeff[1] + to_match.coeff[0], 0.0f), glm::u8vec4(0x20, 0x95, 0x19, 0xff));
					//lines.draw(glm::vec3(axis_min.x, (axis_min.x)*to_match.coeff_offset[1] + to_match.coeff_offset[0], 0.0f), glm::vec3(axis_max.x, axis_max.x*to_match.coeff_offset[1] + to_match.coeff_offset[0], 0.0f), glm::u8vec4(0x20, 0x95, 0x19, 0xff));

					if (show_function_line){
						lines.draw(glm::vec3(axis_min.x, (axis_min.x)*fitted.coeff[1] + fitted.coeff[0], 0.0f), glm::vec3(axis_max.x, axis_max.x*fitted.coeff[1] + fitted.coeff[0], 0.0f), glm::u8vec4(0x31, 0x41, 0xD3, 0xff));
					}
					if (time_fixed == true){
						auto final_time = std::chrono::high_resolution_clock::now();
						auto curr_time = std::chrono::duration< double >(final_time - start_time).count();
						if (curr_time > TIME_LIMIT){
							//float final_error = get_error_from_l2norm(to_match.coeff,to_match.order,boost::ref(err), boost::ref(xs), boost::ref(ys));
							std::vector<double> err;
							// std::vector<double> curr_coef = {(to_match.coeff[0]+to_match.coeff_offset[0])/2,to_match.coeff[1]};
							// final_error = get_error_from_l2norm(curr_coef,to_match.order,boost::ref(err), boost::ref(xs), boost::ref(ys));
							location = donemode;
							//Mode::set_current(std::make_shared<DoneMode>(curr_time,final_error));
						}
					}
					else if (areVectorsApproximatelyEqual(to_match.coeff,fitted.coeff,0.02) == true){
						std::cout<<"Matched!"<<std::endl;
						auto final_time = std::chrono::high_resolution_clock::now();
						auto done_time = std::chrono::duration< double >(final_time - start_time).count();
						Mode::set_current(std::make_shared<DoneMode>(done_time,err_1));
					}
				}
				else if (to_match.order == 2){
					float step = (axis_max.x - axis_min.x) / parabola_step;
					for (uint32_t a = 0; a < parabola_step; ++a){
						lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*to_match.coeff[2]+ (axis_min.x+a*step)*to_match.coeff[1] + to_match.coeff[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*to_match.coeff[2] + (axis_min.x+(a+1)*step)*to_match.coeff[1]+to_match.coeff[0], 0.0f), glm::u8vec4(0x20, 0x95, 0x19, 0xff));
						//lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*to_match.coeff_offset[2]+ (axis_min.x+a*step)*to_match.coeff_offset[1] + to_match.coeff_offset[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*to_match.coeff_offset[2] + (axis_min.x+(a+1)*step)*to_match.coeff_offset[1]+to_match.coeff_offset[0], 0.0f), glm::u8vec4(0x20, 0x95, 0x19, 0xff));
						if (show_function_line){
							lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*fitted.coeff[2]+ (axis_min.x+a*step)*fitted.coeff[1] + fitted.coeff[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*fitted.coeff[2] + (axis_min.x+(a+1)*step)*fitted.coeff[1]+fitted.coeff[0], 0.0f), glm::u8vec4(0x31, 0x41, 0xD3, 0xff));
						}
					}
					if (time_fixed == true){
						auto final_time = std::chrono::high_resolution_clock::now();
						auto curr_time = std::chrono::duration< double >(final_time - start_time).count();
						if (curr_time > TIME_LIMIT){
							std::vector<double> err;
							// std::vector<double> curr_coef = {(to_match.coeff[0]+to_match.coeff_offset[0])/2,to_match.coeff[1],to_match.coeff[2]};
							// final_error = get_error_from_l2norm(curr_coef,to_match.order,boost::ref(err), boost::ref(xs), boost::ref(ys));
							location = donemode;
							//Mode::set_current(std::make_shared<DoneMode>(curr_time,final_error));
						}
					}
					if (areVectorsApproximatelyEqual(to_match.coeff,fitted.coeff,0.027) == true){
						std::cout<<"Matched!"<<std::endl;
						auto final_time = std::chrono::high_resolution_clock::now();
						auto done_time = std::chrono::duration< double >(final_time - start_time).count();
						Mode::set_current(std::make_shared<DoneMode>(done_time,err_2));
					}
				}
			}
			else{
				//Fitted line:
				if (fitted.order == 1){
					lines.draw(glm::vec3(axis_min.x, (axis_min.x)*fitted.coeff[1] + fitted.coeff[0], 0.0f), glm::vec3(axis_max.x, axis_max.x*fitted.coeff[1] + fitted.coeff[0], 0.0f), glm::u8vec4(0x31, 0x41, 0xD3, 0xff));
				}
				else if (fitted.order == 2){
					float step = (axis_max.x - axis_min.x) / parabola_step;
					for (uint32_t a = 0; a < parabola_step; ++a){
						lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*fitted.coeff[2]+ (axis_min.x+a*step)*fitted.coeff[1] + fitted.coeff[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*fitted.coeff[2] + (axis_min.x+(a+1)*step)*fitted.coeff[1]+fitted.coeff[0], 0.0f), glm::u8vec4(0x31, 0x41, 0xD3, 0xff));
					}
				}
			}
			//probes:
			for (auto const &p : probes) {
				if (p.active==1) {
					// draw_circle(p.pos, probe_radius, glm::u8vec4(0x88, 0x88, 0x00, 0xff));
					draw_circle(p.target, probe_radius, glm::u8vec4(0x88, 0x88, 0x88, 0xff));
					draw_circle(p.pos, probe_radius, glm::u8vec4(0x88, 0x88, 0x00, 0xff));
				} else {
					draw_circle(p.target, probe_radius, glm::u8vec4(0xff, 0x88, 0x88, 0xff));
					draw_circle(p.pos, probe_radius, glm::u8vec4(0xff, 0x88, 0x00, 0xff));
				}
			}
			
		}
	}
	if (location == scenes_mode){
		std::vector<double> xs;
		std::vector<double> ys;
		switch (state)
		{
			case begin:
				{
					std::string st = "Mold the function y = ";
					st.append(current_order.functions[current_order.curr_val].name);
					lines.draw_text(st,
						glm::vec3(0.1f, 1.15f, 0.0f), //Start position
						glm::vec3(0.05f, 0.0f, 0.0f),
						glm::vec3(0.0f, 0.05f, 0.0f),
						glm::u8vec4(0x00, 0x00, 0x00, 0x00));
					lines.draw_text("Press N to start / skip ",
						glm::vec3(0.1f, 1.07f, 0.0f), //Start position
						glm::vec3(0.05f, 0.0f, 0.0f),
						glm::vec3(0.0f, 0.05f, 0.0f),
						glm::u8vec4(0x00, 0x00, 0x00, 0x00));
				}
				break;
			case inside:
				{
				std::string st = "Mold the function y = ";
				st.append(current_order.functions[current_order.curr_val].name);
				lines.draw_text(st,
					glm::vec3(0.1f, 1.15f, 0.0f), //Start position
					glm::vec3(0.05f, 0.0f, 0.0f),
					glm::vec3(0.0f, 0.05f, 0.0f),
					glm::u8vec4(0x00, 0x00, 0x00, 0x00));
				lines.draw_text("Press N to start / skip ",
						glm::vec3(0.1f, 1.07f, 0.0f), //Start position
						glm::vec3(0.05f, 0.0f, 0.0f),
						glm::vec3(0.0f, 0.05f, 0.0f),
						glm::u8vec4(0x00, 0x00, 0x00, 0x00));
				//draw particles
				for (auto const &p : particles) {
					draw_circle(p.pos, particle_radius, colors.green);
					xs.push_back(p.pos[0]);
					ys.push_back(p.pos[1]);
				}

				//GET fitted line
				{
					if (ticks_acc % fit_step == 0){
						if (to_match.name != "None"){
							if (to_match.order ==1){
								//Fit line and compute error
								std::vector<double> coeff_1;
								//boost::thread t(polyfit(xs, ys, coeff_1, err_1,1));
								boost::thread t1(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_1), boost::ref(err_1), 1);
								t1.join();
								std::string curr_fitted_text ="";
								fitted.order = 1;
								if (areVectorsApproximatelyEqual(coeff_1,fitted.coeff,0.01) == false){
									fitted.coeff = coeff_1;
									curr_fitted_text.append(std::to_string(fitted.coeff[1]));
									curr_fitted_text.append(" x + ");
									curr_fitted_text.append(std::to_string(fitted.coeff[0]));
									fitted.name = curr_fitted_text;
								}
							}
							else if (to_match.order ==2){
								//Fit line and compute error
								std::vector<double> coeff_2;
								//Fit parabola and compute error
								boost::thread t2(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_2), boost::ref(err_2), 2);
								t2.join();
								std::string curr_fitted_text ="";
								fitted.order = 2;
								if (areVectorsApproximatelyEqual(coeff_2,fitted.coeff,0.01) == false){
									fitted.coeff = coeff_2;
									curr_fitted_text.append(std::to_string(fitted.coeff[2]));
									curr_fitted_text.append(" x^2 + ");
									curr_fitted_text.append(std::to_string(fitted.coeff[1]));
									curr_fitted_text.append(" x + ");
									curr_fitted_text.append(std::to_string(fitted.coeff[0]));
									fitted.name = curr_fitted_text;
								}
								
							}
						}
						else{
							//Fit line and compute error
							std::vector<double> coeff_1,coeff_2;
							boost::thread t1(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_1), boost::ref(err_1), 1);
							
							//Fit parabola and compute error
							boost::thread t2(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_2), boost::ref(err_2), 2);
							t1.join();
							t2.join();
							//std::cout<<"error_1: "<<err_1<<"  error_2: "<<err_2<<std::endl;
							std::string curr_fitted_text ="";
							if(err_1-1 < err_2){
								fitted.order = 1;
								if (areVectorsApproximatelyEqual(coeff_1,fitted.coeff,0.003) == false){
									fitted.coeff = coeff_1;
									curr_fitted_text.append(std::to_string(fitted.coeff[1]));
									curr_fitted_text.append(" x + ");
									curr_fitted_text.append(std::to_string(fitted.coeff[0]));
									fitted.name = curr_fitted_text;
								}
							}
							else{
								fitted.order = 2;
								if (areVectorsApproximatelyEqual(coeff_2,fitted.coeff,0.003) == false){
									fitted.coeff = coeff_2;
									curr_fitted_text.append(std::to_string(fitted.coeff[2]));
									curr_fitted_text.append(" x^2 + ");
									curr_fitted_text.append(std::to_string(fitted.coeff[1]));
									curr_fitted_text.append(" x + ");
									curr_fitted_text.append(std::to_string(fitted.coeff[0]));
									fitted.name = curr_fitted_text;
								}
							}
						}
					}
				}
				if (to_match.name != "None"){
					if (to_match.order == 1){
						if (show_to_match_line){
							lines.draw(glm::vec3(axis_min.x, (axis_min.x)*to_match.coeff[1] + to_match.coeff[0], 0.0f), 
								glm::vec3(axis_max.x, axis_max.x*to_match.coeff[1] + to_match.coeff[0], 0.0f), colors.green);
						}
						if (show_fitted_line){
							lines.draw(glm::vec3(axis_min.x, (axis_min.x)*fitted.coeff[1] + fitted.coeff[0], 0.0f), 
								glm::vec3(axis_max.x, axis_max.x*fitted.coeff[1] + fitted.coeff[0], 0.0f), colors.blue);
						}
					}
					else if (to_match.order == 2){
						if (show_to_match_line || show_fitted_line){
							float step = (axis_max.x - axis_min.x) / parabola_step;
							for (uint32_t a = 0; a < parabola_step; ++a){
								if (show_to_match_line){
									lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*to_match.coeff[2]+ (axis_min.x+a*step)*to_match.coeff[1] + to_match.coeff[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*to_match.coeff[2] + (axis_min.x+(a+1)*step)*to_match.coeff[1]+to_match.coeff[0], 0.0f), glm::u8vec4(0x20, 0x95, 0x19, 0xff));
								}
								//lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*to_match.coeff_offset[2]+ (axis_min.x+a*step)*to_match.coeff_offset[1] + to_match.coeff_offset[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*to_match.coeff_offset[2] + (axis_min.x+(a+1)*step)*to_match.coeff_offset[1]+to_match.coeff_offset[0], 0.0f), glm::u8vec4(0x20, 0x95, 0x19, 0xff));
								if (show_fitted_line){
									lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*fitted.coeff[2]+ (axis_min.x+a*step)*fitted.coeff[1] + fitted.coeff[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*fitted.coeff[2] + (axis_min.x+(a+1)*step)*fitted.coeff[1]+fitted.coeff[0], 0.0f), glm::u8vec4(0x31, 0x41, 0xD3, 0xff));
								}
							}
						}
						
					}
					if (time_fixed == true){
						auto final_time = std::chrono::high_resolution_clock::now();
						auto curr_time = std::chrono::duration< double >(final_time - matching_start_time).count();
						//Draw time left
						std::string time_left_text = "Time left: ";
						time_left_text.append(std::to_string(int(TIME_LIMIT_PER_FUNCTION - curr_time)));
						time_left_text.append(" s");
						lines.draw_text(time_left_text, 
										glm::vec3(axis_max.x-0.4, 1.15f, 0.0f), //Start position
										glm::vec3(0.05f, 0.0f, 0.0f),
										glm::vec3(0.0f, 0.05f, 0.0f),
										glm::u8vec4(0x00, 0x00, 0x00, 0x00));
						std::vector<double> err;
						//std::vector<double> curr_coef = {(to_match.coeff[0]+to_match.coeff_offset[0])/2,to_match.coeff[1],to_match.coeff[2]};
						// final_error = 0.0;
						if (curr_time > TIME_LIMIT_PER_FUNCTION){
							final_error = get_error_from_l2norm(to_match.coeff,to_match.order,boost::ref(err), boost::ref(xs), boost::ref(ys));
							prev_match = function(to_match.coeff);
							if (scene == line_intercept || scene == line_slope || scene == parabola_vertex || scene == parabola_concavity){ 								current_order.curr_val += 1 ;
								append_data(file_name, {"function",current_order.functions[current_order.curr_val].name,
											"time",std::to_string(int(curr_time)),"error",std::to_string(final_error)});
								if (current_order.curr_val < current_order.functions.size()){
									std::cout<<std::endl;
									parse_function(current_order.functions[current_order.curr_val]);
								}
								state = end;
							}
						}
					}
				}
				//probes:
				for (auto const &p : probes) {
					if (p.active==1) {
						draw_circle(p.pos, probe_radius, glm::u8vec4(0x88, 0x88, 0x00, 0xff));
						draw_circle(p.target, probe_radius, glm::u8vec4(0x88, 0x88, 0x88, 0xff));
					} else {
						draw_circle(p.target, probe_radius, colors.red);
						draw_circle(p.pos, probe_radius, colors.red);
					}
				}
				}
				break;
			case end:
				{
					//Particles
					for (auto const &p : particles) {
						draw_circle(p.pos, particle_radius, colors.green
						);
						xs.push_back(p.pos[0]);
						ys.push_back(p.pos[1]);
					}
					//probes:
					for (auto const &p : probes) {
						if (p.active==1) {
							// draw_circle(p.pos, probe_radius, glm::u8vec4(0x88, 0x88, 0x00, 0xff));
							draw_circle(p.target, probe_radius, glm::u8vec4(0x88, 0x88, 0x88, 0xff));
							draw_circle(p.pos, probe_radius, glm::u8vec4(0x88, 0x88, 0x00, 0xff));
						} else {
							draw_circle(p.target, probe_radius, colors.red);
							draw_circle(p.pos, probe_radius, colors.red);
						}
					}

					lines.draw_text("Great, time's up",
						glm::vec3(0.1f, 1.15f, 0.0f), //Start position
						glm::vec3(0.05f, 0.0f, 0.0f),
						glm::vec3(0.0f, 0.05f, 0.0f),
						glm::u8vec4(0x00, 0x00, 0x00, 0x00));
					std::string st = "Correct function, y = ";
					st.append(current_order.functions[current_order.curr_val-1].name);
					st.append(" shown in green");	
					lines.draw_text(st,
						glm::vec3(0.1f, 1.1f, 0.0f), //Start position
						glm::vec3(0.05f, 0.0f, 0.0f),
						glm::vec3(0.0f, 0.05f, 0.0f),
						glm::u8vec4(0x00, 0x00, 0x00, 0x00));
					if (to_match.name != "None"){
						// std::vector<double> curr_coeffs = current_order.functions[current_order.curr_val-1].real_coeff;
						std::vector<double> curr_coeffs = current_order.functions[current_order.curr_val-1].coeff;
						
						if (to_match.order == 1){
							if (show_to_match_line_on_finished){
								float y1 = axis_min.x * curr_coeffs[1] + curr_coeffs[0];
								float y2 = axis_max.x * curr_coeffs[1] + curr_coeffs[0];

								glm::vec3 p1(axis_min.x, y1, 0.0f);
								glm::vec3 p2(axis_max.x, y2, 0.0f);

								// Check if y-values are outside the range
								if (y1 < axis_min.y || y1 > axis_max.y) {
									// Calculate x-value at y-limits
									float x1 = (axis_min.y - curr_coeffs[0]) / curr_coeffs[1];
									// Set extreme point with y-limits
									p1 = glm::vec3(x1, axis_min.y, 0.0f);
								}

								if (y2 < axis_min.y || y2 > axis_max.y) {
									// Calculate x-value at y-limits
									float x2 = (axis_max.y - curr_coeffs[0]) / curr_coeffs[1];
									// Set extreme point with y-limits
									p2 = glm::vec3(x2, axis_max.y, 0.0f);
								}

								lines.draw_bold(p1, p2, colors.green,0.001);
							}
							if (show_function_line_on_finished){
								lines.draw(glm::vec3(axis_min.x, (axis_min.x)*fitted.coeff[1] + fitted.coeff[0], 0.0f), 
									glm::vec3(axis_max.x, axis_max.x*fitted.coeff[1] + fitted.coeff[0], 0.0f), colors.blue);
							}
						}
						else if (to_match.order == 2){
							// std::cout<<"to match order 2"<<std::endl;
							if (show_to_match_line_on_finished){
								float step = (axis_max.x - axis_min.x) / parabola_step;
								for (uint32_t a = 0; a < parabola_step; ++a) {
									// Calculate the x-values and corresponding y-values for the parabola segment
									float x1 = axis_min.x + a * step;
									float x2 = axis_min.x + (a + 1) * step;
									float y1 = x1 * x1 * curr_coeffs[2] + x1 * curr_coeffs[1] + curr_coeffs[0];
									float y2 = x2 * x2 * curr_coeffs[2] + x2 * curr_coeffs[1] + curr_coeffs[0];

									glm::vec3 p1(x1, y1, 0.0f);
									glm::vec3 p2(x2, y2, 0.0f);

									if (y1> axis_min.y && y1 < axis_max.y && y2> axis_min.y && y2 < axis_max.y){
										lines.draw_bold(p1, p2, colors.green,0.001);
									}
								}
							}
						}
					}
				}
				break;
		}

	} 
	GL_ERRORS();
}

void PlayMode::reset_clay() {
	particles.clear();
	//neighbors.clear();
	probes.clear();

	const glm::uvec2 grid_size = glm::uvec2(0.4f / particle_radius, 0.1 / particle_radius);
	const float spacing = 2.0f * particle_radius;
	const glm::vec2 offset = -0.5f * spacing * glm::vec2(grid_size.x - 1 + 0.5f, std::sqrt(3.0f) / 2.0f * (grid_size.y - 1)) + 0.5f * (box_max + box_min);
	particles.reserve(grid_size.x * grid_size.y);
	for (uint32_t r = 0; r < grid_size.y; ++r) {
		for (uint32_t c = 0; c < grid_size.x; ++c) {
			Particle p;
			p.pos = spacing * glm::vec2(c + (r % 2 ? 0.5f : 0.0f), r * std::sqrt(3.0f) / 2.0f) + offset;
			particles.emplace_back(p);
		}
	}
	start_time = std::chrono::high_resolution_clock::now();
	probe_rot = 0.0f; //Set the rotation back to zero:
}

Eigen::Vector2f toEigen(glm::vec2 v) {
    return Eigen::Vector2f(v.x, v.y);
}

glm::vec2 toGlm(const Eigen::Vector2f& v) {
    return glm::vec2(v.x(), v.y());
}

// glm::mat2 toGlm(const Eigen::Matrix2f& m) {
//     return glm::mat2(
//         m(0, 0), m(0, 1),
//         m(1, 0), m(1, 1)
//     );
// }
void PlayMode::tick_clay() {
	const float kTimeStep = 0.01f;
    const float kElasticity = 0.9f;
    const float kFriction = 0.5f;
	if (rigid==true){
		//rigid body: simulation

		//step as per velocity:
		std::vector< glm::vec2 > old_pos;
		old_pos.reserve(particles.size());
		for (auto &p : particles) {
			old_pos.emplace_back(p.pos);
			//p.vel *= std::pow(0.5f, ClayTick / 0.2f); //friction / damping orig
			p.vel *= std::pow(0.05f, ClayTick / 0.01f); //friction / damping better performandce
			//p.vel += ClayTick * glm::vec2(0.0f, -1.0f); //DEBUG: gravity
			p.pos += p.vel * ClayTick;
		}
		//probe targets:
		for (auto &p : probes) {
			float len = glm::length(p.target - p.pos);
			float new_len = std::pow(0.5f, ClayTick / 0.5f) * len - ClayTick / 0.5f;
			if (new_len <= 0.0f) {
				p.pos = p.target;
			} else {
				p.pos += (p.target - p.pos) * (new_len / len);
			}
			p.pos = p.target;
		}
	// 	for (auto& probe : probes) {
    //     if (probe.active) {
    //         Eigen::Matrix2f R;
    //         glm::vec2 center_of_mass = glm::vec2(0.0f, 0.0f);
    //         Eigen::Matrix<float, 2, Eigen::Dynamic> P(2, particles.size());
    //         P.setZero();

    //         // compute center of mass and particle positions relative to center of mass
    //         for (size_t i = 0; i < particles.size(); ++i) {
    //             center_of_mass += particles[i].pos;
    //         }
    //         center_of_mass /= particles.size();

    //         for (size_t i = 0; i < particles.size(); ++i) {
    //             particles[i].pos -= center_of_mass;
    //             P.col(i) = Eigen::Vector2f(particles[i].pos.x, particles[i].pos.y);
    //         }

    //         // compute probe target position relative to center of mass
    //         Eigen::Vector2f t = Eigen::Vector2f(probe.target.x, probe.target.y) - toEigen(center_of_mass);

    //         // compute rotation matrix R using SVD
    //         Eigen::Matrix<float, 2, Eigen::Dynamic> U, V;
    //         Eigen::Vector2f S;
    //         Eigen::BDCSVD<Eigen::MatrixXf> svd(P * t);
    //         U = svd.matrixU();
    //         V = svd.matrixV();
    //         S = svd.singularValues();
    //         R = V * U.transpose();

    //         // update particle positions and velocities
    //         for (size_t i = 0; i < particles.size(); ++i) {
    //             Eigen::Vector2f new_pos = R * P.col(i) + toEigen(center_of_mass);
    //             glm::vec2 pos_diff = glm::vec2(new_pos.x(), new_pos.y()) - particles[i].pos;
    //             glm::vec2 vel_diff = pos_diff / ClayTick - particles[i].vel;
    //             particles[i].pos = glm::vec2(new_pos.x(), new_pos.y());
    //             particles[i].vel += vel_diff * kElasticity;
    //             particles[i].vel *= 1.0f - kFriction;
    //         }

    //         // check for collisions and move particles away from probe
    //         for (size_t i = 0; i < particles.size(); ++i) {
    //             glm::vec2 dir = particles[i].pos - probe.pos;
    //             float dist = glm::length(dir);
    //             float radius = 0.5f;
    //             if (dist < radius) {
    //                 dir = glm::normalize(dir);
    //                 particles[i].pos -= 2.0f * (radius - dist) * dir;
    //             }
    //         }
    //     }
		//particles vs particles (the slow way):

		float alpha = 0.1f; //controls particle squish
		for (auto &p : particles) {
			for (auto &p2 : particles) {
				if (&p == &p2) break;
				glm::vec2 to = p2.pos - p.pos;
				float len2 = glm::length2(to);
				if (len2 > 0.0f && len2 < (2.0f * particle_radius) * (2.0f * particle_radius)) {
					glm::vec2 step = to * (2.0f * particle_radius / std::sqrt(len2) - 1.0f);
					p.pos -= alpha * 0.5f * step;
					p2.pos += alpha * 0.5f * step;
				}
			}
		}

		// Collect all the active probes
		std::vector<Probe> active_probes;
		for (auto& probe : probes) {
			if (probe.active) {
				active_probes.push_back(probe);
			}
		}

		if (active_probes.size() < 1) {
			std::cout<<"No active probes"<<std::endl;
			return;
		}

		// Compute the merged indices of the particles within all the active probes' radii
		std::vector<size_t> indices;
		for (size_t i = 0; i < particles.size(); i++) {
			glm::vec2 pos = particles[i].pos;
			bool in_radius = true;
			for (auto& probe : active_probes) {
				float dist = glm::distance(pos, probe.pos);
				if (dist < probe_radius) {
					indices.push_back(i);
				}
			}
			// if (in_radius) {
			// 	indices.push_back(i);
			// }
		}

		if (indices.size() < 3) {
			//std::cout<<"No touching particles"<<std::endl;
			return;
		}
		// else{
		// 	std::cout<<"Touching particles"<<std::endl;
		// }

		// Compute the center of mass of the particles within all the active probes' radii
		Eigen::Vector2f center_of_mass(0.0f, 0.0f);
		float total_mass = 0.0f;
		for (auto i : indices) {
			float mass = 1.0f;  // All particles have the same mass
			center_of_mass += mass * Eigen::Vector2f(particles[i].pos.x, particles[i].pos.y);
			total_mass += mass;
		}
		center_of_mass /= total_mass;

		// Compute the merged covariance matrix of the particles within all the active probes' radii
		Eigen::Matrix2f covariance_matrix = Eigen::Matrix2f::Zero();
		for (auto i : indices) {
			float mass = 1.0f;  // All particles have the same mass
			Eigen::Vector2f p_i(particles[i].pos.x, particles[i].pos.y);
			Eigen::Vector2f delta_p = p_i - center_of_mass;
			covariance_matrix += mass * delta_p * delta_p.transpose();
		}

		// Compute the singular value decomposition of the merged covariance matrix
		Eigen::JacobiSVD<Eigen::Matrix2f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

		// Compute the optimal rotation and translation
		Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
		Eigen::Vector2f t = Eigen::Vector2f::Zero();
		for (auto& probe : active_probes) {
			t += toEigen(probe.target) - R * center_of_mass - toEigen(probe.pos);
		}
		t /= active_probes.size();

		// Update the position of the particles and the probes
		for (auto &p : particles) {
			glm::vec2 p_i = p.pos;
			Eigen::Vector2f new_pos = R * Eigen::Vector2f(p_i.x, p_i.y) - 0.001f * t;
			p.pos.x = new_pos[0];
			p.pos.y = new_pos[1];
		}
		for (auto &probe : active_probes) {
			glm::vec2 pr = probe.pos;
			probe.pos = toGlm(R * toEigen(pr) + 0.001f * t);
		}

		// for (auto& probe : probes) {
		// 	if (probe.active) {
		// 		// Get the indices of the particles within the probe's radius
		// 		std::vector<size_t> indices;
		// 		for (size_t i = 0; i < particles.size(); i++) {
		// 			float dist = glm::distance(particles[i].pos, probe.pos);
		// 			
		
		// 		}

		// 		if (indices.size() < 3) {
		// 			continue;
		// 		}

		// 		// Compute the center of mass of the particles within the probe's radius
		// 		Eigen::Vector2f center_of_mass(0.0f, 0.0f);
		// 		float total_mass = 0.0f;
		// 		for (auto i : indices) {
		// 			float mass = 1.0f;  // All particles have the same mass
		// 			center_of_mass += mass * Eigen::Vector2f(particles[i].pos.x, particles[i].pos.y);
		// 			total_mass += mass;
		// 		}
		// 		center_of_mass /= total_mass;

		// 		// Compute the covariance matrix of the particles within the probe's radius
		// 		Eigen::Matrix2f covariance_matrix = Eigen::Matrix2f::Zero();
		// 		for (auto i : indices) {
		// 			float mass = 1.0f;  // All particles have the same mass
		// 			Eigen::Vector2f p_i(particles[i].pos.x, particles[i].pos.y);
		// 			Eigen::Vector2f delta_p = p_i - center_of_mass;
		// 			covariance_matrix += mass * delta_p * delta_p.transpose();
		// 		}

		// 		// Compute the singular value decomposition of the covariance matrix
		// 		Eigen::JacobiSVD<Eigen::Matrix2f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

		// 		// Compute the optimal rotation and translation
		// 		Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
		// 		Eigen::Vector2f t = toEigen(probe.target) - R * center_of_mass;

		// 		// Update the position of the particles
		// 		// for (auto i : indices) {
		// 		//     glm::vec2 p_i = particles[i].pos;
		// 		//     Eigen::Vector2f new_pos = R * Eigen::Vector2f(p_i.x, p_i.y) + t;
		// 		//     particles[i].pos.x = new_pos[0];
		// 		//     particles[i].pos.y = new_pos[1];
		// 		// }
		// 		for (auto &p : particles) {
		// 			glm::vec2 p_i = p.pos;
		// 			Eigen::Vector2f new_pos = R * Eigen::Vector2f(p_i.x, p_i.y) - 0.001f * t;
		// 			p.pos.x = new_pos[0];
		// 			p.pos.y = new_pos[1];
		// 		}
		// 		glm::vec2 pr = probe.pos;
		// 		probe.pos.x += 0.001f * t[0];
		// 		probe.pos.y += 0.001f * t[1];

				
		// 	}
		// }
		//estimate velocity from motion:
		for (uint32_t i = 0; i < particles.size(); ++i) {
			auto &p = particles[i];
			p.vel = (p.pos - old_pos[i]) / ClayTick;
		}
		

		//viscosity (the slow way):
		// for (auto &p : particles) {
		// 	for (auto &p2 : particles) {
		// 		if (&p == &p2) break;
		// 		glm::vec2 to = p2.pos - p.pos;
		// 		float len2 = glm::length2(to);
		// 		const float outer2 = (2.0f * viscosity_radius) * (2.0f * viscosity_radius);
		// 		const float inner2 = (2.0f * particle_radius) * (2.0f * particle_radius);
		// 		if (len2 > 0.0f && len2 < outer2) {
		// 			float amt = 1.0f - std::max(0.0f, len2 - inner2) / (outer2 - inner2);
		// 			glm::vec2 avg = 0.5f * (p2.vel + p.vel);
		// 			p.vel += (avg - p.vel) * amt;
		// 			p2.vel += (avg - p2.vel) * amt;
		// 		}
		// 	}
		// }
    // 	cx = 0.0;
    // 	cy = 0.0;
    // 	I = 0.0;		
	// 	for (auto &p : particles) {
    //         cx += p.pos.x;
    //         cy += p.pos.y;
    //         //m += particles[i].mass;
    //         Ixx += (p.pos.y * p.pos.y);
    //         Iyy += (p.pos.x * p.pos.x);
    //     }
	// 	cx /= particles.size();
	// 	cy /= particles.size();
	// 	for (auto &p : particles) {
	// 		double x = p.pos.x - cx;
	// 		double y = p.pos.y - cy;
	// 		I += (x * x + y * y);
    // 	}
	// 	double Iinv = 1.0 / I;
	// 	double Ixx = Iinv * (cy * cy);
	// 	double Iyy = Iinv * (cx * cx);
	// 	double Izz = Ixx + Iyy;
	// 	//step as per velocity:
	// 	std::vector< glm::vec2 > old_pos;
	// 	old_pos.reserve(particles.size());
	// 	for (auto &p : particles) {
	// 		old_pos.emplace_back(p.pos);
	// 		//p.vel *= std::pow(0.5f, ClayTick / 0.2f); //friction / damping orig
	// 		p.vel *= std::pow(0.1f, ClayTick / 0.05f); //friction / damping better performandce
	// 		//p.vel += ClayTick * glm::vec2(0.0f, -1.0f); //DEBUG: gravity
	// 		p.pos += p.vel * ClayTick;
	// 	}
	// 	//probe targets:
	// 	for (auto &p : probes) {
	// 		float len = glm::length(p.target - p.pos);
	// 		float new_len = std::pow(0.5f, ClayTick / 0.5f) * len - ClayTick / 0.5f;
	// 		if (new_len <= 0.0f) {
	// 			p.pos = p.target;
	// 		} else {
	// 			p.pos += (p.target - p.pos) * (new_len / len);
	// 		}
	// 		p.pos = p.target;
	// 	}
	// 	//particles vs particles (the slow way):
	// 	float alpha = 0.9f; //controls particle squish
	// 	std::vector<Particle> prev_particles;
	// 	std::vector<Probe> new_particles;
	// 	// Step 1: Compute centroids of particles and probes
	// // Compute the center of mass of active particles
	// 	Eigen::Vector2f center_of_mass = Eigen::Vector2f::Zero();
	// 	int active_particle_count = 0;
	// 	for (auto &p2 : particles) {
	// 		if (p2.vel != Eigen::Vector2f::Zero()) {
	// 			center_of_mass += p2.pos;
	// 			active_particle_count++;
	// 		}
	// 	}
	// 	if (active_particle_count == 0) {
	// 		return;  // No active particles
	// 	}
	// 	center_of_mass /= active_particle_count;

	// 	// Compute the covariance matrix of active particles
	// 	Eigen::Matrix2f covariance_matrix = Eigen::Matrix2f::Zero();
	// 	for (const Particle& particle : particles) {
	// 		if (particle.vel != Eigen::Vector2f::Zero()) {
	// 			Eigen::Vector2f deviation = particle.pos - center_of_mass;
	// 			covariance_matrix += deviation * deviation.transpose();
	// 		}
	// 	}
	// 	covariance_matrix /= active_particle_count;

	// 	for (auto &p2 : probes) {
	// 		bool flag_moved = false;
	// 		glm::vec2 step;
	// 		for (auto &p : particles) {
	// 		// bool touching = false;
	// 			//if (p2.active == false) continue;
	// 				glm::vec2 to = p2.pos - p.pos;
	// 				float len2 = glm::length2(to);
	// 				const float near = particle_radius + probe_radius;
	// 				const float near2 = near * near;
	// 				if (len2 > 0.0f && len2 < near2) {
	// 					step = to * (near / std::sqrt(len2) - 1.0f);
	// 					//p.pos -= 0.5f * step;
	// 					prev_particles.push_back(p);
	// 					p2.pos += 0.5f * step;
	// 					new_particles.push_back(p2);
	// 					flag_moved = true;
	// 					break;
	// 				}
	// 		}
	// 		// 	//std::cout<<"touching "<<touching<<std::endl;
	// 		// }
	// 		// Apply the spring force between the probe and particles
	// 		// for (int i = 0; i < particles.size(); i++) {
	// 		// 	double dx = particles[i].pos.x - cx;
	// 		// 	double dy = particles[i].pos.y - cy;
	// 		// 	double r = sqrt(dx * dx + dy * dy);
	// 		// 	double f = -k * (r - probe_radius);
	// 		// 	particles[i].vx += dt * (probe.fx * f) / particles[i].mass;
	// 		// 	particles[i].vy += dt * (probe.fy * f) / particles[i].mass;
	// 		// 	probe.fx += dt * (dx * f);
	// 		// 	probe.fy += dt * (dy * f);
	// 		// }
	// 		if (flag_moved == true){
	// 			for (auto &p : particles) {
	// 				p.pos -= 0.5f * step;
	// 			}
	// 		}
			
	// 		flag_moved = false;
	// 	}
	}
	else if(mod_1){
		std::vector< glm::vec2 > old_pos;
		old_pos.reserve(particles.size());

		//step as per velocity:
		for (auto &p : particles) {
			old_pos.emplace_back(p.pos);
			//p.vel *= std::pow(0.5f, ClayTick / 0.2f); //friction / damping orig
			p.vel *= std::pow(kFriction, ClayTick / kDamping); //friction / damping better performandce
			//p.vel += ClayTick * glm::vec2(0.0f, -1.0f); //DEBUG: gravity
			p.pos += p.vel * ClayTick;
		}

		//probe targets:
		for (auto &p : probes) {
			float len = glm::length(p.target - p.pos);
			float new_len = std::pow(0.5f, ClayTick / 0.5f) * len - ClayTick / 0.5f;
			if (new_len <= 0.0f) {
				p.pos = p.target;
			} else {
				p.pos += (p.target - p.pos) * (new_len / len);
			}
			p.pos = p.target;
		}
		//particles vs world:
		for (auto &p : particles) {
			if (p.pos.x < box_min.x) {
				p.pos.x = box_min.x;
				if (p.vel.x < 0.0f) {
					p.vel.x = wall_bounce * std::abs(p.vel.x);
					old_pos[&p - &particles[0]].x = p.pos.x - ClayTick * p.vel.x;
				}
			}
			if (p.pos.x > box_max.x) {
				p.pos.x = box_max.x;
				if (p.vel.x > 0.0f) {
					p.vel.x = wall_bounce * -std::abs(p.vel.x);
					old_pos[&p - &particles[0]].x = p.pos.x - ClayTick * p.vel.x;
				}
			}
			if (p.pos.y < box_min.y) {
				p.pos.y = box_min.y;
				if (p.vel.y < 0.0f) {
					p.vel.y = wall_bounce * std::abs(p.vel.y);
					old_pos[&p - &particles[0]].y = p.pos.y - ClayTick * p.vel.y;
				}
			}
			if (p.pos.y > box_max.y) {
				p.pos.y = box_max.y;
				if (p.vel.y > 0.0f) {
					p.vel.y = wall_bounce * -std::abs(p.vel.y);
					old_pos[&p - &particles[0]].y = p.pos.y - ClayTick * p.vel.y;
				}
			}
		}
		//particles vs particles (the slow way):
		touching = false;
		float dist = 0.0f;
		for (auto &p : particles) {
			for (auto &p2 : particles) {
				if (&p == &p2) break;
				glm::vec2 to = p2.pos - p.pos;
				float len2 = glm::length2(to);
				if (len2 > 0.0f && len2 < (2.0f * particle_radius) * (2.0f * particle_radius)) {
					glm::vec2 step = to * (2.0f * particle_radius / std::sqrt(len2) - 1.0f);
					p.pos -= alpha * 0.5f * step;
					p2.pos += alpha * 0.5f * step;
				}
			}
			for (auto &p2 : probes) {
				if (p2.active == false) continue;
				glm::vec2 to = p2.pos - p.pos;
				float len2 = glm::length2(to);
				const float near = particle_radius + probe_radius;
				const float near2 = near * near;
				if (len2 > 0.0f && len2 < near2) {
					glm::vec2 step = to * (near / std::sqrt(len2) - 1.0f);
					p.pos -= 0.5f * step;
					p2.pos += 0.5f * step;
					//dist = glm::length2(probes[0].pos-probes[1].pos);
					//touching = true;
				}
				if (len2 > 0.0f && (abs(len2-near2)<0.0005)) {
					touching = true;
				}
			}
		}
		std::stringstream stream;
		stream << std::fixed << std::setprecision(0) << dist*10000;
		std::string s = stream.str();
		//Count the continous number of no touchs
		const int cycleThreshold = 5;
		if (serial_port_name != "None"){
			std::string read_byte_1;
			if(touching != isActive) { 
				std::string str;
				std::cout<<"toggle"<<std::endl;
				isActive = touching;
				if (filter_signal==false){
					if (touching) {
						std::cout << "Writing Signal is true!" << std::endl;
						// Reset the cycle count if the signal is true
						ticks_acc_filter = 0;
						str.append("i1l");
						serial_port.Write(str);
					} else {
						// Increment the cycle count if the signal is false
						//ticks_acc++;
						// Check the cycle count against the threshold value
						std::cout << "Writing Signal is false." << std::endl;
						str.append("i0l");
						serial_port.Write(str);
					}
				}
				else {
					if (touching) {
						std::cout << "Writing Signal is true!" << std::endl;
						// Reset the cycle count if the signal is true
						ticks_acc_filter = 0;
						str.append("i1l");
						serial_port.Write(str);
					} else {
						// Increment the cycle count if the signal is false
						//ticks_acc++;
						// Check the cycle count against the threshold value
						if (ticks_acc_filter <= cycleThreshold) {
							std::cout << "Writing Assuming signal is true." << std::endl;
							// Reset the cycle count if the threshold is reached
							ticks_acc_filter = 0;
							str.append("i1l");
							serial_port.Write(str);

						}else{
							std::cout << "Writing Signal is false." << std::endl;
							str.append("i0l");
							serial_port.Write(str);
						}
					}
				}
			}
			// if(ticks_acc_filter>1000 && touching == false){
			// 	ticks_acc_filter = 0;
			// 	std::cout << "Timeout Writing Signal is false." << std::endl;
			// 	serial_port.Write("i0l");
			// 	//std::cout<<"reset"<<std::endl;
			// }
		}
	
		//estimate velocity from motion:
		for (uint32_t i = 0; i < particles.size(); ++i) {
			auto &p = particles[i];
			p.vel = (p.pos - old_pos[i]) / ClayTick;
		}

		//viscosity (the slow way):
		for (auto &p : particles) {
			for (auto &p2 : particles) {
				if (&p == &p2) break;
				glm::vec2 to = p2.pos - p.pos;
				float len2 = glm::length2(to);
				const float outer2 = (2.0f * viscosity_radius) * (2.0f * viscosity_radius);
				const float inner2 = (2.0f * particle_radius) * (2.0f * particle_radius);
				if (len2 > 0.0f && len2 < outer2) {
					float amt = 1.0f - std::max(0.0f, len2 - inner2) / (outer2 - inner2);
					glm::vec2 avg = 0.5f * (p2.vel + p.vel);
					p.vel += (avg - p.vel) * amt;
					p2.vel += (avg - p2.vel) * amt;
				}
			}
		}

	}
	else{
		std::vector< glm::vec2 > old_pos;
		old_pos.reserve(particles.size());

		//step as per velocity:
		for (auto &p : particles) {
			old_pos.emplace_back(p.pos);
			//p.vel *= std::pow(0.5f, ClayTick / 0.2f); //friction / damping orig
			p.vel *= std::pow(0.1f, ClayTick / 0.05f); //friction / damping better performandce
			//p.vel += ClayTick * glm::vec2(0.0f, -1.0f); //DEBUG: gravity
			p.pos += p.vel * ClayTick;
		}

		//probe targets:
		for (auto &p : probes) {
			float len = glm::length(p.target - p.pos);
			float new_len = std::pow(0.5f, ClayTick / 0.5f) * len - ClayTick / 0.5f;
			if (new_len <= 0.0f) {
				p.pos = p.target;
			} else {
				p.pos += (p.target - p.pos) * (new_len / len);
			}
			p.pos = p.target;
		}
		// const float wall_bounce = 0.5f; prev version
		const float wall_bounce = 0.1f;
		// const float viscosity_radius = 2.0f * particle_radius;
		const float viscosity_radius = 4.0f * particle_radius;

		//particles vs world:
		for (auto &p : particles) {
			if (p.pos.x < box_min.x) {
				p.pos.x = box_min.x;
				if (p.vel.x < 0.0f) {
					p.vel.x = wall_bounce * std::abs(p.vel.x);
					old_pos[&p - &particles[0]].x = p.pos.x - ClayTick * p.vel.x;
				}
			}
			if (p.pos.x > box_max.x) {
				p.pos.x = box_max.x;
				if (p.vel.x > 0.0f) {
					p.vel.x = wall_bounce * -std::abs(p.vel.x);
					old_pos[&p - &particles[0]].x = p.pos.x - ClayTick * p.vel.x;
				}
			}
			if (p.pos.y < box_min.y) {
				p.pos.y = box_min.y;
				if (p.vel.y < 0.0f) {
					p.vel.y = wall_bounce * std::abs(p.vel.y);
					old_pos[&p - &particles[0]].y = p.pos.y - ClayTick * p.vel.y;
				}
			}
			if (p.pos.y > box_max.y) {
				p.pos.y = box_max.y;
				if (p.vel.y > 0.0f) {
					p.vel.y = wall_bounce * -std::abs(p.vel.y);
					old_pos[&p - &particles[0]].y = p.pos.y - ClayTick * p.vel.y;
				}
			}
		}

		//particles vs particles (the slow way):
		float alpha = 0.9f; //controls particle squish
		for (auto &p : particles) {
			for (auto &p2 : particles) {
				if (&p == &p2) break;
				glm::vec2 to = p2.pos - p.pos;
				float len2 = glm::length2(to);
				if (len2 > 0.0f && len2 < (2.0f * particle_radius) * (2.0f * particle_radius)) {
					glm::vec2 step = to * (2.0f * particle_radius / std::sqrt(len2) - 1.0f);
					p.pos -= alpha * 0.5f * step;
					p2.pos += alpha * 0.5f * step;
				}
			}
			for (auto &p2 : probes) {
				if (p2.active == false) continue;
				glm::vec2 to = p2.pos - p.pos;
				float len2 = glm::length2(to);
				const float near = particle_radius + probe_radius;
				const float near2 = near * near;
				if (len2 > 0.0f && len2 < near2) {
					glm::vec2 step = to * (near / std::sqrt(len2) - 1.0f);
					p.pos -= 0.5f * step;
					p2.pos += 0.5f * step;
				}
			}
		}

		//estimate velocity from motion:
		for (uint32_t i = 0; i < particles.size(); ++i) {
			auto &p = particles[i];
			p.vel = (p.pos - old_pos[i]) / ClayTick;
		}

		//viscosity (the slow way):
		for (auto &p : particles) {
			for (auto &p2 : particles) {
				if (&p == &p2) break;
				glm::vec2 to = p2.pos - p.pos;
				float len2 = glm::length2(to);
				const float outer2 = (2.0f * viscosity_radius) * (2.0f * viscosity_radius);
				const float inner2 = (2.0f * particle_radius) * (2.0f * particle_radius);
				if (len2 > 0.0f && len2 < outer2) {
					float amt = 1.0f - std::max(0.0f, len2 - inner2) / (outer2 - inner2);
					glm::vec2 avg = 0.5f * (p2.vel + p.vel);
					p.vel += (avg - p.vel) * amt;
					p2.vel += (avg - p2.vel) * amt;
				}
			}
		}
	}
}
// void PlayMode::tick_clay() {
// 	std::vector< glm::vec2 > old_pos;
// 	old_pos.reserve(particles.size());

// 	//step as per velocity:
// 	for (auto &p : particles) {
// 		old_pos.emplace_back(p.pos);
// 		//p.vel *= std::pow(0.5f, ClayTick / 0.2f); //friction / damping orig
// 		p.vel *= std::pow(0.1f, ClayTick / 0.05f); //friction / damping better performandce
// 		//p.vel += ClayTick * glm::vec2(0.0f, -1.0f); //DEBUG: gravity
// 		p.pos += p.vel * ClayTick;
// 	}

// 	//probe targets:
// 	for (auto &p : probes) {
// 		float len = glm::length(p.target - p.pos);
// 		float new_len = std::pow(0.5f, ClayTick / 0.5f) * len - ClayTick / 0.5f;
// 		if (new_len <= 0.0f) {
// 			p.pos = p.target;
// 		} else {
// 			p.pos += (p.target - p.pos) * (new_len / len);
// 		}
// 		p.pos = p.target;
// 	}
// 	// const float wall_bounce = 0.5f; prev version
// 	const float wall_bounce = 0.1f;
// 	// const float viscosity_radius = 2.0f * particle_radius;
// 	const float viscosity_radius = 4.0f * particle_radius;

// 	//particles vs world:
// 	for (auto &p : particles) {
// 		if (p.pos.x < box_min.x) {
// 			p.pos.x = box_min.x;
// 			if (p.vel.x < 0.0f) {
// 				p.vel.x = wall_bounce * std::abs(p.vel.x);
// 				old_pos[&p - &particles[0]].x = p.pos.x - ClayTick * p.vel.x;
// 			}
// 		}
// 		if (p.pos.x > box_max.x) {
// 			p.pos.x = box_max.x;
// 			if (p.vel.x > 0.0f) {
// 				p.vel.x = wall_bounce * -std::abs(p.vel.x);
// 				old_pos[&p - &particles[0]].x = p.pos.x - ClayTick * p.vel.x;
// 			}
// 		}
// 		if (p.pos.y < box_min.y) {
// 			p.pos.y = box_min.y;
// 			if (p.vel.y < 0.0f) {
// 				p.vel.y = wall_bounce * std::abs(p.vel.y);
// 				old_pos[&p - &particles[0]].y = p.pos.y - ClayTick * p.vel.y;
// 			}
// 		}
// 		if (p.pos.y > box_max.y) {
// 			p.pos.y = box_max.y;
// 			if (p.vel.y > 0.0f) {
// 				p.vel.y = wall_bounce * -std::abs(p.vel.y);
// 				old_pos[&p - &particles[0]].y = p.pos.y - ClayTick * p.vel.y;
// 			}
// 		}
// 	}

// 	//particles vs particles (the slow way):
// 	float alpha = 0.9f; //controls particle squish
// 	for (auto &p : particles) {
// 		for (auto &p2 : particles) {
// 			if (&p == &p2) break;
// 			glm::vec2 to = p2.pos - p.pos;
// 			float len2 = glm::length2(to);
// 			if (len2 > 0.0f && len2 < (2.0f * particle_radius) * (2.0f * particle_radius)) {
// 				glm::vec2 step = to * (2.0f * particle_radius / std::sqrt(len2) - 1.0f);
// 				p.pos -= alpha * 0.5f * step;
// 				p2.pos += alpha * 0.5f * step;
// 			}
// 		}
// 		for (auto &p2 : probes) {
// 			if (p2.active == false) continue;
// 			glm::vec2 to = p2.pos - p.pos;
// 			float len2 = glm::length2(to);
// 			const float near = particle_radius + probe_radius;
// 			const float near2 = near * near;
// 			if (len2 > 0.0f && len2 < near2) {
// 				glm::vec2 step = to * (near / std::sqrt(len2) - 1.0f);
// 				p.pos -= 0.5f * step;
// 				p2.pos += 0.5f * step;
// 			}
// 		}
// 	}

// 	//estimate velocity from motion:
// 	for (uint32_t i = 0; i < particles.size(); ++i) {
// 		auto &p = particles[i];
// 		p.vel = (p.pos - old_pos[i]) / ClayTick;
// 	}

// 	//viscosity (the slow way):

// 	for (auto &p : particles) {
// 		for (auto &p2 : particles) {
// 			if (&p == &p2) break;
// 			glm::vec2 to = p2.pos - p.pos;
// 			float len2 = glm::length2(to);
// 			const float outer2 = (2.0f * viscosity_radius) * (2.0f * viscosity_radius);
// 			const float inner2 = (2.0f * particle_radius) * (2.0f * particle_radius);
// 			if (len2 > 0.0f && len2 < outer2) {
// 				float amt = 1.0f - std::max(0.0f, len2 - inner2) / (outer2 - inner2);
// 				glm::vec2 avg = 0.5f * (p2.vel + p.vel);
// 				p.vel += (avg - p.vel) * amt;
// 				p2.vel += (avg - p2.vel) * amt;
// 			}
// 		}
// 	}
// }
