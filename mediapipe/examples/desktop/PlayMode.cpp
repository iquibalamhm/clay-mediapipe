#include "PlayMode.hpp"

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
			tick_clay();
		} else if(evt.key.keysym.sym == SDLK_LEFT){
			do_rotation_left = true;
		}
		else if(evt.key.keysym.sym == SDLK_RIGHT){
			do_rotation_right = true;
		}
	} else if (evt.type == SDL_KEYUP) {
		if(evt.key.keysym.sym == SDLK_LEFT){
			do_rotation_left = false;
		}
		else if(evt.key.keysym.sym == SDLK_RIGHT){
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
			x1 = (*coordinates)[1];
			y1 = (*coordinates)[2];
			x2 = (*coordinates)[3];
			y2 = (*coordinates)[4];
			
		}
		else{
			x1 = (*coordinates)[1];
			y1 = (*coordinates)[2];
			x2 = (*coordinates)[3];
			y2 = (*coordinates)[4];
			x3 = (*coordinates)[5];
			y3 = (*coordinates)[6];
			x4 = (*coordinates)[7];
			y4 = (*coordinates)[8];

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
void PlayMode::polyfit(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &coeff,float &l2_error,int order){
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
	if (order == 1){
		for(uint32_t p = 0; p < x.size(); ++ p)
		{
			double cerr = y.at(p) - coeff[0] + coeff[1]*x.at(p);
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
	l2_error = l2_norm(err);
}

void PlayMode::update(float elapsed) {
	time_acc += elapsed;

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

			// std::cout<<"at.x: "<<at_1.x<<" "<<at_1.y<<std::endl;
			//probes[0].target = at_1;
			//probes[1].target = at_2;
			if (num_hands ==1){
				if (probes.size() != 2) {
					probes.assign(2, Probe());
					probes[0].pos = at_1 - 0.5f * gap; //I'm not sure what the 0.5f does
					probes[1].pos = at_2 + 0.5f * gap;
					std::cout<<" here 1";
				}
				probes[0].target = at_1 - 0.5f * gap;
				probes[1].target = at_2 + 0.5f * gap;
			} else if (num_hands == 2){
				glm::vec2 at_3, at_4;
				at_4.x = (index_2_at.x - world_to_clip[3][0]) / world_to_clip[0][0];
				at_4.y = (index_2_at.y - world_to_clip[3][1]) / world_to_clip[1][1];
				at_3.x = (thumb_2_at.x - world_to_clip[3][0]) / world_to_clip[0][0];
				at_3.y = (thumb_2_at.y - world_to_clip[3][1]) / world_to_clip[1][1];
				//The number 10.f is the min gap that can happen between the probes
				//glm::vec2 gap = glm::mix(1.0f * particle_radius + 2.0f * probe_radius, 2.0f * particle_radius + 2.0f * probe_radius, probe_pinch) * glm::vec2(-std::sin(probe_rot), std::cos(probe_rot));
				// std::cout << "At: (" << at_1.x << ", " << at_1.y << " " <<at_2.x << ", " << at_2.y << ")"<<
				// 			 "(" << at_3.x << ", " << at_3.y << " " <<at_4.x << ", " << at_4.y << ")"<<std::endl;

				//std::cout<<"at.x: "<<at_3.x<<" "<<at_3.y<<std::endl;
				//probes[0].target = at_1;
				//probes[1].target = at_2;
				if (probes.size() != 4) {
					probes.assign(4, Probe());
					probes[0].pos = at_1 - 0.5f * gap; //I'm not sure what the 0.5f does
					probes[1].pos = at_2 + 0.5f * gap;
					probes[2].pos = at_3 - 0.5f * gap; //I'm not sure what the 0.5f does
					probes[3].pos = at_4 + 0.5f * gap;
					std::cout<<" here 2 "<<std::endl;
				}

				probes[0].target = at_1 - 0.5f * gap;
				probes[1].target = at_2 + 0.5f * gap;

				probes[2].target = at_3 - 0.5f * gap;
				probes[3].target = at_4 + 0.5f * gap;
			}

			for (auto const &p : probes) {
				continue;
				std::cout<<"pos : "<<p.pos.x<<" "<<p.pos.y<<" "<<"target "<<p.target.x<<" "<<p.target.y<<" ";
			}
			//std::cout<<std::endl;
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
				std::cout<<" here 4 "<<std::endl;
			}
			probes[0].target = at - 0.5f * gap;
			probes[1].target = at + 0.5f * gap;

		} else {
			probes.clear();
		}

		tick_clay();
		++ticks;
	}
	auto after = std::chrono::high_resolution_clock::now();
	ticks_acc += ticks;
	duration_acc += std::chrono::duration< double >(after - before).count();

	if (ticks_acc > 5.0f / ClayTick) {
		double ms_per_tick = duration_acc * 1000.0f / ticks_acc;
		std::cout << "On " << particles.size() << " particles, ran " << ticks_acc << " ticks in " << duration_acc << " seconds. That's " << ms_per_tick << " ms per tick, or " << (1000.0f * ClayTick) / ms_per_tick << "x real time." << std::endl;
		ticks_acc = 0;
		duration_acc = 0.0;
	}
}

void PlayMode::draw(glm::uvec2 const &drawable_size) {
	glClearColor(0.9f, 0.9f, 0.87f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);

	const float aspect = drawable_size.x / float(drawable_size.y);

	const float scale = std::min(2.0f * aspect / (box_max.x - box_min.x + 0.1f), 2.0f / (box_max.y - box_min.y + 0.1f));
	const glm::vec2 offset = -0.5f * (box_min + box_max);

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

		//axis:
		lines.draw(glm::vec3((axis_max.x-axis_min.x)/2 + axis_min.x, axis_min.y, 0.0f), glm::vec3((axis_max.x-axis_min.x)/2 + axis_min.x, axis_max.y, 0.0f), glm::u8vec4(0x08, 0x44, 0x44, 0xff));
		lines.draw(glm::vec3(axis_min.x, (axis_max.y-axis_min.y)/2 + axis_min.y, 0.0f), glm::vec3(axis_max.x, (axis_max.y-axis_min.y)/2 + axis_min.y, 0.0f), glm::u8vec4(0x08, 0x44, 0x44, 0xff));

		//lines.draw_text("hola ",glm::vec3(0.01f, 0.01f, 0.0f),glm::vec3(axis_min.x, axis_max.x, 0.0f), glm::vec3(axis_min.y, axis_max.y, 0.0f),glm::u8vec4(0x08, 0x44, 0x44, 0xff));
		// constexpr float H = 0.5f;

		// lines.draw_text("Mouse",
		// 	glm::vec3(-aspect + 0.1f * H, -1.0 + 0.1f * H, 0.0),
		// 	glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
		// 	glm::u8vec4(0x00, 0x00, 0x00, 0x00));

		//from 15-466-f22-base6:
		static std::array< glm::vec2, 16 > const circle = [](){
			std::array< glm::vec2, 16 > ret;
			for (uint32_t a = 0; a < ret.size(); ++a) {
				float ang = a / float(ret.size()) * 2.0f * float(M_PI);
				ret[a] = glm::vec2(std::cos(ang), std::sin(ang));
			}
			return ret;
		}();

		auto draw_circle = [&](glm::vec2 const &center, float radius, glm::u8vec4 color) {
			for (uint32_t a = 0; a < circle.size(); ++a) {
				lines.draw(
					glm::vec3(center + radius * circle[a], 0.0f),
					glm::vec3(center + radius * circle[(a+1)%circle.size()], 0.0f),
					color
				);
			}
		};

		//particles:
		//double avg_x = 0;
		//double avg_y = 0;
		std::vector<double> xs;
		std::vector<double> ys;
		for (auto const &p : particles) {
			draw_circle(p.pos, particle_radius, glm::u8vec4(0x22, 0x44, 0x44, 0xff));
			xs.push_back(p.pos[0]);
			ys.push_back(p.pos[1]);
		}
		if (ticks_acc % fit_step == 0){
			// std::cout<<" ran 100 ticks"<<std::endl;
			//Fit line and compute error
			
			float err_1,err_2;
			std::vector<double> coeff_1,coeff_2;
			//boost::thread t(polyfit(xs, ys, coeff_1, err_1,1));
			boost::thread t1(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_1), boost::ref(err_1), 1);
    		
			//Fit parabola and compute error
			//polyfit(xs, ys, coeff_2, err_2,2);
			boost::thread t2(&PlayMode::polyfit, this, boost::ref(xs), boost::ref(ys), boost::ref(coeff_2), boost::ref(err_2), 2);
    		t1.join();
			t2.join();
			//std::cout<<"error_1: "<<err_1<<"  error_2: "<<err_2<<std::endl;
			std::string curr_fitted_text ="";
			if(err_1-1 < err_2){
				poly_order = 1;
				coeff = coeff_1;
				curr_fitted_text.append(std::to_string(coeff[1]));
				curr_fitted_text.append(" x + ");
				curr_fitted_text.append(std::to_string(coeff[0]));
				fitted_text = curr_fitted_text;
			}
			else{
				poly_order = 2;
				coeff = coeff_2;
				curr_fitted_text.append(std::to_string(coeff[2]));
				curr_fitted_text.append(" x^2 + ");
				curr_fitted_text.append(std::to_string(coeff[1]));
				curr_fitted_text.append(" x + ");
				curr_fitted_text.append(std::to_string(coeff[0]));
				fitted_text = curr_fitted_text;
			}
		}
		lines.draw_text(fitted_text,
			glm::vec3(0.9f, 0.1f, 0.0f),
			glm::vec3(0.05f, 0.0f, 0.0f),
			glm::vec3(0.0f, 0.05f, 0.0f),
			glm::u8vec4(0x00, 0x00, 0x00, 0x00));
		//fitted line:

		if (poly_order == 1){
			lines.draw(glm::vec3(axis_min.x, (axis_min.x)*coeff[1] + coeff[0], 0.0f), glm::vec3(axis_max.x, axis_max.x*coeff[1] + coeff[0], 0.0f), glm::u8vec4(0x31, 0x41, 0xD3, 0xff));
		}
		else if (poly_order == 2){
			float step = (axis_max.x - axis_min.x) / parabola_step;
			for (uint32_t a = 0; a < parabola_step; ++a){
				lines.draw(glm::vec3(axis_min.x + a*step, (axis_min.x+a*step)*(axis_min.x+a*step)*coeff[2]+ (axis_min.x+a*step)*coeff[1] + coeff[0], 0.0f), glm::vec3((axis_min.x+(a+1)*step), (axis_min.x+(a+1)*step)*(axis_min.x+(a+1)*step)*coeff[2] + (axis_min.x+(a+1)*step)*coeff[1]+coeff[0], 0.0f), glm::u8vec4(0x31, 0x41, 0xD3, 0xff));
			}
		}
		//std::cout<<avg_x<<" "<<avg_y<<std::endl;
		//probes:
		for (auto const &p : probes) {
			draw_circle(p.target, probe_radius, glm::u8vec4(0x88, 0x88, 0x88, 0xff));
			draw_circle(p.pos, probe_radius, glm::u8vec4(0x88, 0x88, 0x00, 0xff));
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

	probe_rot = 0.0f; //Set the rotation back to zero:
}

void PlayMode::tick_clay() {
	std::vector< glm::vec2 > old_pos;
	old_pos.reserve(particles.size());

	//step as per velocity:
	for (auto &p : particles) {
		old_pos.emplace_back(p.pos);
		p.vel *= std::pow(0.5f, ClayTick / 0.2f); //friction / damping
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
	//std::cout<<probes.size()<<" ";
	const float wall_bounce = 0.5f;
	const float viscosity_radius = 2.0f * particle_radius;

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
