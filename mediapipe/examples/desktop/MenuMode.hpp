#include "Mode.hpp"
#include "Sprite.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <functional>

#include <deque>
#include <string>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
struct MenuMode : Mode {
	float ELAPSED_TIME;
	struct Item;
	MenuMode(std::vector< Item > const &items);
	virtual ~MenuMode();

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

	glm::vec2 button_f1_min = glm::vec2(0.0f, 0.0f);
	glm::vec2 button_f1_max = glm::vec2(1.5f, 1.0f);

	glm::vec2 button_f2_min = glm::vec2(0.0f, 0.0f);
	glm::vec2 button_f2_max = glm::vec2(1.5f, 1.0f);


	void reset_clay();
	inline static constexpr float ClayTick = 0.001f;
	void tick_clay();

	//ad-hoc performance measurement:
	uint32_t ticks_acc = 0;
	uint32_t ticks_acc_filter = 0;

	//Each menu item is an "Item":
	struct Item {
		Item(
			std::string const &name_,
			Sprite const *sprite_ = nullptr,
			float scale_ = 1.0f,
			std::function< void(Item const &) > const &on_select_ = nullptr,
			glm::vec2 const &at_ = glm::vec2(0.0f)
			) : name(name_), sprite(sprite_), scale(scale_), on_select(on_select_), at(at_) {
		}
		std::string name;
		Sprite const *sprite; //sprite drawn for item
		float scale; //scale for sprite
		std::function< void(Item const &) > on_select; //if set, item is selectable
		glm::vec2 at; //location to draw item
	};
	std::vector< Item > items;
		//if set, used to highlight the current selection:
	Sprite const *left_select = nullptr;
	Sprite const *right_select = nullptr;

	//must be set to the atlas from which all the sprites used herein are taken:
	SpriteAtlas const *atlas = nullptr;

	//currently selected item:
	uint32_t selected = 0;

	//area to display; by default, menu lays items out in the [-1,1]^2 box:
	glm::uvec2 view_min = glm::vec2(-1.0f, -1.0f);
	glm::uvec2 view_max = glm::vec2( 1.0f,  1.0f);

	double duration_acc = 0.0f;
	
	//if not nullptr, background's functions are called as follows:
	// background->handle_event() is called at the end of handle_event() [if this doesn't handle the event]
	// background->update() is called at the end of update()
	// background->draw() is called at the start of draw()
	//IMPORTANT NOTE: this means that if background->draw() ends up deleting this (e.g., by removing
	//  the last shared_ptr that references it), then it will crash. Don't do that!
	std::shared_ptr< Mode > background;
};
