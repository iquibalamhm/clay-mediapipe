//File management
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <cstdlib>
#include <random>

int main ()
{
    // Get the current date and time
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);

    // Format the date and time for the filename
    std::stringstream filename;
    filename << std::put_time(now, "%Y-%m-%d_%H-%M-%S") << ".csv";

    // Create the file
    std::ofstream file(filename.str());

    if (!file) {
        std::cerr << "Failed to create the file: " << filename.str() << std::endl;
        return 1;
    }

    // Write column headers
    file << "Column1,Column2,Column3\n";

    // Set up random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 99);

    // Write random data rows
    for (int i = 0; i < 3; ++i) {
        file << dis(gen) << ',' << dis(gen) << ',' << dis(gen) << '\n';
    }

    // Close the file
    file.close();

    std::cout << "File created successfully: " << filename.str() << std::endl;
    // intercept_order = scene_order({"-1","0","1","2"});
   return 0;
}
                