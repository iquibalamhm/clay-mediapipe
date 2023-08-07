// Mode.hpp declares the "Mode::current" static member variable, which is used to decide where event-handling, updating, and drawing events go:
#include "Mode.hpp"

// The 'PlayMode' mode plays the game:
#include "PlayMode.hpp"

// The 'MenuMode' mode plays the game:
#include "MenuMode.hpp"

// For asset loading:
#include "Load.hpp"

// GL.hpp will include a non-namespace-polluting set of opengl prototypes:
#include "GL.hpp"

// for screenshots:
#include "load_save_png.hpp"

// Includes for libSDL:
#include <SDL.h>
#include <mutex>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
// Shared memory
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>

//...and for c++ standard library functions:
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <algorithm>

// File management
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <cstdlib>
#include <random>

#ifdef _WIN32
extern "C"
{
	uint32_t GetACP();
}
#endif
constexpr char kSharedMemorySegmentName[] = "SharedMemory";

bool exists()
{
	try
	{
		boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, kSharedMemorySegmentName);
		return segment.check_sanity();
	}
	catch (const std::exception &ex)
	{
		std::cout << "managed_shared_memory ex: " << ex.what();
	}
	return false;
}
int main(int argc, char **argv)
{
#ifdef _WIN32
	{ // when compiled on windows, check that code page is forced to utf-8 (makes file loading/saving work right):
		// see: https://docs.microsoft.com/en-us/windows/apps/design/globalizing/use-utf8-code-page
		uint32_t code_page = GetACP();
		if (code_page == 65001)
		{
			std::cout << "Code page is properly set to UTF-8." << std::endl;
		}
		else
		{
			std::cout << "WARNING: code page is set to " << code_page << " instead of 65001 (UTF-8). Some file handling functions may fail." << std::endl;
		}
	}

	// when compiled on windows, unhandled exceptions don't have their message printed, which can make debugging simple issues difficult.
	try
	{
#endif

		// Check if shared memory object exists
		bool exist = exists();
		// Access the vector in shared memory
		typedef boost::interprocess::allocator<int, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;
		typedef boost::interprocess::vector<int, ShmemAllocator> MyVector;
		typedef boost::interprocess::interprocess_mutex Mutex;
		MyVector *myvector;
		Mutex *mutex;
		boost::interprocess::managed_shared_memory segment;
		typedef boost::interprocess::interprocess_condition CondVar;
		CondVar *condvar;
		if (exist)
		{
			// Open existing shared memory object
			std::cout << "Configuring shared memory" << std::endl;
			segment = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, kSharedMemorySegmentName);

			//------------  initialization ------------
			// MyVector *myvector = segment.find<MyVector>("MyVector").first;
			std::cout << "Configuring vector memory" << std::endl;

			myvector = segment.find<MyVector>("MyVector").first;
			std::cout << "Configuring mutex" << std::endl;

			// Access the mutex in shared memory
			mutex = segment.find<Mutex>("Mutex").first;
			std::cout << "Configuring condvar" << std::endl;

			// Access the condition variable in shared memory
			condvar = segment.find<CondVar>("CondVar").first;
		}
		else
		{
			// Create new shared memory object
			std::cerr << "Failed to open shared memory segment: " << std::endl;
		}

		//------------  initialization ------------
		// Initialize SDL library:
		SDL_Init(SDL_INIT_VIDEO);

		// Ask for an OpenGL context version 3.3, core profile, enable debug:
		SDL_GL_ResetAttributes();
		SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
		SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
		SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

		// create window:
		SDL_Window *window = SDL_CreateWindow(
			"clay-game", // TODO: remember to set a title for your game!
			SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
			// 1280, 720, //TODO: modify window size if you'd like
			// 640, 420,
			// 1280,960, //640x480 x2
			1344, 752,								 // 672x376 x2
			SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE // uncomment to allow resizing
				| SDL_WINDOW_ALLOW_HIGHDPI			 // uncomment for full resolution on high-DPI screens
		);
		// Shared memory segment

		// prevent exceedingly tiny windows when resizing:
		SDL_SetWindowMinimumSize(window, 100, 100);

		if (!window)
		{
			std::cerr << "Error creating SDL window: " << SDL_GetError() << std::endl;
			return 1;
		}

		// Create OpenGL context:
		SDL_GLContext context = SDL_GL_CreateContext(window);

		if (!context)
		{
			SDL_DestroyWindow(window);
			std::cerr << "Error creating OpenGL context: " << SDL_GetError() << std::endl;
			return 1;
		}

		// On windows, load OpenGL entrypoint s: (does nothing on other platforms)
		init_GL();

		// Set VSYNC + Late Swap (prevents crazy FPS):
		if (SDL_GL_SetSwapInterval(-1) != 0)
		{
			std::cerr << "NOTE: couldn't set vsync + late swap tearing (" << SDL_GetError() << ")." << std::endl;
			if (SDL_GL_SetSwapInterval(1) != 0)
			{
				std::cerr << "NOTE: couldn't set vsync (" << SDL_GetError() << ")." << std::endl;
			}
		}
		// Hide mouse cursor (note: showing can be useful for debugging):
		SDL_ShowCursor(SDL_DISABLE);

		//------------ load assets --------------
		call_load_functions();

		//------------ create game mode + make current --------------
		// Mode::set_current(std::make_shared< MenuMode >());
		Mode::set_current(std::make_shared<PlayMode>());

		//------------ main loop ------------

		// this inline function will be called whenever the window is resized,
		//  and will update the window_size and drawable_size variables:
		glm::uvec2 window_size;	  // size of window (layout pixels)
		glm::uvec2 drawable_size; // size of drawable (physical pixels)
		// On non-highDPI displays, window_size will always equal drawable_size.
		auto on_resize = [&]()
		{
			int w, h;
			SDL_GetWindowSize(window, &w, &h);
			window_size = glm::uvec2(w, h);
			SDL_GL_GetDrawableSize(window, &w, &h);
			drawable_size = glm::uvec2(w, h);
			glViewport(0, 0, drawable_size.x, drawable_size.y);
		};
		on_resize();

		// This will loop until the current mode is set to null:
		bool flag_closed = false;
		// std::cout<<"entering while loop"<<std::endl;

		{
			// Get the current date and time
			std::time_t t = std::time(nullptr);
			std::tm *now = std::localtime(&t);

			// Format the date and time for the filename
			std::stringstream filename;
			filename << std::put_time(now, "%Y-%m-%d_%H-%M-%S") << ".csv";

			// Create the file
			std::ofstream file(filename.str());

			if (!file)
			{
				std::cerr << "Failed to create the file: " << filename.str() << std::endl;
				return 1;
			}
			// Write column headers
			// file << "Column1,Column2,Column3\n";

			std::cout << "File created successfully: " << filename.str() << std::endl;
			// Close the file
			file.close();
			Mode::current->init_logfile(filename.str());
		}

		// parse arguments
		{
			std::string arg_port = std::string(argv[1]);
			if (arg_port.find("/dev/") != std::string::npos)
			{
				std::cout << "using " << arg_port << std::endl;
				Mode::current->init_serial(arg_port);
			}
			else
			{
				Mode::current->init_serial("None");
			}
			// std::string str;
			// for (int i = 1; i < argc; i++)
			// {
			// 	std::string arg = argv[i];
			// 	if (arg.substr(0, 11) == "--function=")
			// 	{
			// 		str = arg.substr(11);
			// 	}
			// }
			// if (!str.empty())
			// {
			// 	std::cout << "Function to match: " << str << std::endl;
			// 	Mode::current->init_function(str);
			// }
			// else
			// {
			// 	std::cout << "No function specified" << std::endl;
			// 	// if you don't want to show the fitted line, comment this
			// 	Mode::current->init_function("None");
			// }
		}

		while (Mode::current)
		{
			// every pass through the game loop creates one frame of output
			//   by performing three steps:
			static SDL_Event evt;
			if (exist)
			{
				// std::cout << "Waiting for data" << std::endl;
				//  Lock the mutex
				std::unique_lock<Mutex> lock(*mutex);
				// std::cout << "Waiting for data" << std::endl;
				//  Check if the data has been modified
				if (myvector->size() == 0)
				{
					lock.unlock();
					// continue;
				}
				else
				{
					condvar->notify_one();
					double diff_y = abs((*myvector)[1] - (*myvector)[3]);
					SDL_Event event;
					event.type = SDL_USEREVENT;
					std::vector<int> *coordinates = nullptr;
					int num_hands = (*myvector)[0];
					if (num_hands == 1)
					{
						coordinates = new std::vector<int>({(*myvector)[0], (*myvector)[1], (*myvector)[2], (*myvector)[3], (*myvector)[4], (*myvector)[5], (*myvector)[6],(*myvector)[7]});
						event.user.data1 = coordinates;
						SDL_PushEvent(&event);
					}
					else if (num_hands == 2)
					{
						coordinates = new std::vector<int>({(*myvector)[0], (*myvector)[1], (*myvector)[2], (*myvector)[3], (*myvector)[4], (*myvector)[5], (*myvector)[6],
															(*myvector)[7], (*myvector)[8], (*myvector)[9], (*myvector)[10], (*myvector)[11], (*myvector)[12]});
						event.user.data1 = coordinates;
						if ((*myvector)[1] && (*myvector)[7])
						{
							SDL_Event event_reset;
							event_reset.type = SDL_KEYDOWN;
							event_reset.key.keysym.sym = SDLK_BACKSPACE;
							SDL_PushEvent(&event_reset);
						}
						else
						{
							SDL_PushEvent(&event);
						}
					}
					myvector->clear();
				}
			}
			{ //(1) process any events that are pending
				while (SDL_PollEvent(&evt) == 1)
				{
					// handle resizing:
					if (evt.type == SDL_WINDOWEVENT && evt.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)
					{
						on_resize();
					}
					// handle input:
					if (Mode::current && Mode::current->handle_event(evt, window_size))
					{
						// mode handled it; great
					}
					else if (evt.type == SDL_QUIT)
					{
						Mode::current->close_serial();
						Mode::set_current(nullptr);
						break;
					}
					else if (evt.type == SDL_KEYDOWN && evt.key.keysym.sym == SDLK_PRINTSCREEN)
					{
						// --- screenshot key ---
						std::string filename = "screenshot.png";
						std::cout << "Saving screenshot to '" << filename << "'." << std::endl;
						glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
						glReadBuffer(GL_FRONT);
						int w, h;
						SDL_GL_GetDrawableSize(window, &w, &h);
						std::vector<glm::u8vec4> data(w * h);
						glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
						for (auto &px : data)
						{
							px.a = 0xff;
						}
						save_png(filename, glm::uvec2(w, h), data.data(), LowerLeftOrigin);
					}
				}
				if (!Mode::current)
					break;
			}

			{ //(2) call the current mode's "update" function to deal with elapsed time:
				auto current_time = std::chrono::high_resolution_clock::now();
				static auto previous_time = current_time;
				float elapsed = std::chrono::duration<float>(current_time - previous_time).count();
				previous_time = current_time;

				// if frames are taking a very long time to process,
				// lag to avoid spiral of death:
				elapsed = std::min(0.1f, elapsed);
				Mode::current->update(elapsed);
				if (!Mode::current)
					break;
			}

			{ //(3) call the current mode's "draw" function to produce output:
				Mode::current->draw(drawable_size);
			}

			// Wait until the recently-drawn frame is shown before doing it all again:
			SDL_GL_SwapWindow(window);
		}
		//------------  teardown ------------
		SDL_GL_DeleteContext(context);
		context = 0;

		SDL_DestroyWindow(window);
		window = NULL;

		return 0;

#ifdef _WIN32
	}
	catch (std::exception const &e)
	{
		std::cerr << "Unhandled exception:\n"
				  << e.what() << std::endl;
		return 1;
	}
	catch (...)
	{
		std::cerr << "Unhandled exception (unknown type)." << std::endl;
		throw;
	}
#endif
}
