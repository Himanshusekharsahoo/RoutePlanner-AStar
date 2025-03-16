# RoutePlanner-AStar 🚀

This project implements a **route planning application** using the **A* search algorithm**. It utilizes **OpenStreetMap (OSM) data** for real-world map visualization and the **io2d** library for rendering the map and computed path. The application allows users to find the **shortest path** between two points on a map and visualize the route.

---

## 📌 Table of Contents
1. [Features](#-features)
2. [Tech Stack](#-tech-stack)
3. [Requirements](#-requirements)
4. [Installation](#-installation)
5. [Usage](#-usage)
6. [Example](#-example)
7. [Project Structure](#-project-structure)
8. [Dependencies](#-dependencies)
9. [Contributing](#-contributing)
10. [License](#-license)
11. [Acknowledgments](#-acknowledgments)
12. [Contact](#-contact)

---

## 🚀 Features
- ✅ **A* Algorithm**: Computes the shortest path between two locations.
- ✅ **OpenStreetMap Integration**: Uses real-world OSM data for accurate routing.
- ✅ **Map Rendering**: Displays the map and computed path using the `io2d` library.
- ✅ **Efficient Data Handling**: Processes large OSM datasets efficiently.
- ✅ **Cross-Platform Compatibility**: Works on **Windows, Linux, and macOS**.

---

## 💻 Tech Stack
- **Programming Language**: C++
- **Libraries**:
  - **io2d** → For rendering maps.
  - **pugixml** → For parsing OpenStreetMap data.
- **Build System**: CMake
- **Dependency Manager**: vcpkg
- **Version Control**: Git
- **Map Data**: OpenStreetMap (OSM)

---

## 🔧 Requirements
To build and run this project, you need:
- **C++ Compiler** (GCC, Clang, or MSVC)
- **CMake** (version 3.10 or higher)
- **io2d Library** (for rendering)
- **OpenStreetMap Data** (`.osm` file)

---

## ⚙️ Installation

### Step 1: Clone the Repository
```bash
git clone https://github.com/Himanshusekharsahoo/RoutePlanner-AStar.git
cd RoutePlanner-AStar
```

### Step 2: Install Dependencies (Using vcpkg)
```bash
vcpkg install cairo io2d pugixml
vcpkg integrate install
```

### Step 3: Build the Project with CMake
```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[path-to-vcpkg]/scripts/buildsystems/vcpkg.cmake
cmake --build .
```

---

## ▶️ Usage
1. Download an OpenStreetMap (`.osm`) file for the region you want to use. You can get it from [OpenStreetMap](https://www.openstreetmap.org/).
2. Run the program with the OSM file:
```bash
./OSM_A_star_search -f path/to/map.osm
```
3. Enter start and end coordinates when prompted:
```bash
Enter start and end points (start_x start_y end_x end_y): 0.1 0.1 0.9 0.9
```
4. View the computed shortest path on the map.

---

## 🌍 Example: Bhubaneswar, India
1. Download the Bhubaneswar map from OpenStreetMap.
2. Run the program:
```bash
./OSM_A_star_search -f bhubaneswar.osm
```
3. Enter start and end coordinates (e.g., Railway Station to OUAT College):
```bash
Enter start and end points (start_x start_y end_x end_y): 0.30 0.77 0.28 0.59
```

---

## 📂 Project Structure
```
RoutePlanner-AStar/
├── CMakeLists.txt          # CMake build configuration
├── map.osm                 # OpenStreetMap data file
├── map.png                 # Rendered map image (optional)
│
├── src/                    # Source code files
│   ├── main.cpp            # Main application logic
│   ├── model.cpp           # Map data parsing and handling
│   ├── model.h             # Model class header
│   ├── render.cpp          # Map rendering using io2d
│   ├── render.h            # Render class header
│   ├── route_model.cpp     # Route model implementation
│   ├── route_model.h       # Route model header
│   ├── route_planner.cpp   # A* algorithm implementation
│   └── route_planner.h     # Route planner header
│
├── test/                   # Unit tests
│   └── utest_rp_a_star_search.cpp  # Unit test for A* algorithm
│
├── thirdparty/             # Third-party libraries
│   └── googletest/         # Google Test framework for unit testing
│
└── build/                  # Build directory (generated by CMake)
    ├── Debug/              # Debug build output
    │   └── OSM_A_star_search.exe  # Executable file
    └── ...                 # Other build-related files
```

---

## 📝 Dependencies
- **io2d** → For rendering maps.
- **pugixml** → For parsing OpenStreetMap data.
- **vcpkg** → For managing dependencies.

---

## 🤝 Contributing
Contributions are welcome! If you'd like to contribute:
1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a clear description.

---

## 💌 License
This project is licensed under the **MIT License**. See the `LICENSE` file for details.

---

## 🙏 Acknowledgments
- **OpenStreetMap** for providing the map data.
- **io2d** for rendering support.
- **A* Algorithm** for pathfinding.

---

## 📩 Contact
For questions or feedback, reach out:
- **GitHub**: [Himanshusekharsahoo](https://github.com/Himanshusekharsahoo)
- **Email**: work.himanshuse@gmail.com
- 
