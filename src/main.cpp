#include <io2d.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include "render.h"
#include "route_model.h"
#include "route_planner.h"

using namespace std::experimental;  // For io2d library

/**
 * Reads binary data from a file and returns it as a vector of std::byte.
 * @param path The path to the file.
 * @return An optional vector of std::byte containing the file data, or std::nullopt if the file cannot be read.
 */
static std::optional<std::vector<std::byte>> ReadFile(const std::string& path) {
    std::ifstream is{path, std::ios::binary | std::ios::ate};  // Open file in binary mode and seek to the end
    if (!is) {
        std::cerr << "Failed to open file: " << path << std::endl;
        return std::nullopt;
    }

    auto size = is.tellg();  // Get file size
    if (size <= 0) {
        std::cerr << "File is empty or invalid: " << path << std::endl;
        return std::nullopt;
    }

    std::vector<std::byte> contents(size);  // Allocate buffer for file contents
    is.seekg(0);  // Seek back to the beginning of the file
    is.read(reinterpret_cast<char*>(contents.data()), size);  // Read file data into buffer

    if (!is) {
        std::cerr << "Failed to read file: " << path << std::endl;
        return std::nullopt;
    }

    return std::move(contents);  // Return the file contents
}

int main(int argc, const char** argv) {
    std::string osm_data_file = "";  // Path to the OSM data file

    // Parse command-line arguments
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            if (std::string_view{argv[i]} == "-f" && ++i < argc) {
                osm_data_file = argv[i];  // Set OSM data file path from command-line argument
            }
        }
    } else {
        // Display usage instructions if no arguments are provided
        std::cout << "To specify a map file, use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";  // Default map file
    }

    std::vector<std::byte> osm_data;  // Vector to store OSM data

    // Read OSM data from the specified file
    if (osm_data.empty() && !osm_data_file.empty()) {
        std::cout << "Reading OpenStreetMap data from the following file: "
                  << osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if (!data) {
            std::cerr << "Failed to read OSM data. Exiting." << std::endl;
            return -1;
        }
        osm_data = std::move(*data);  // Move file data into osm_data
    }

    // Get user input for start and end coordinates
    std::cout << "Enter start and end points (start_x start_y end_x end_y): ";
    float start_x{}, start_y{}, end_x{}, end_y{};
    std::cin >> start_x >> start_y >> end_x >> end_y;

    // Validate user input
    if (std::cin.fail()) {
        std::cerr << "Invalid input. Please enter numeric values." << std::endl;
        return -1;
    }

    // Build the RouteModel from OSM data
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    // Display the distance of the calculated route
    std::cout << "Distance: " << route_planner.GetDistance() << " meters." << std::endl;

    // Render the results of the search
    Render render{model};

    // Set up io2d display
    auto display = io2d::output_surface{400,
                                        400,
                                        io2d::format::argb32,
                                        io2d::scaling::none,
                                        io2d::refresh_style::fixed,
                                        30};
    display.size_change_callback([](io2d::output_surface& surface) {
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback(
        [&](io2d::output_surface& surface) { render.Display(surface); });
    display.begin_show();

    return 0;
}