#include <iostream>
#include <filesystem>
#include <cstdlib>

namespace fs = std::filesystem;

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  // Define base paths
  fs::path workspace_dir = "/home/tong/roboconcpp_ws";
  fs::path src_dir = workspace_dir / "src";
  fs::path robocon_desktop_dir = src_dir / "robocon_desktop";
  fs::path build_dir = robocon_desktop_dir / "build-robocon_gui-Desktop-Debug";
  fs::path gui_executable = build_dir / "robocon_gui";
  fs::path gui_source_dir = robocon_desktop_dir / "robocon_gui";

  // Check if gui is not built yet
  if (!fs::exists(gui_executable)) {
    std::cerr << "Error: robocon_gui is not built yet." << std::endl;
    std::cout << "Building robocon_gui..." << std::endl;

    std::string build_command = 
      "/usr/bin/cmake -S " + gui_source_dir.string() +
      " -B " + build_dir.string() +
      " '-GUnix Makefiles' -DCMAKE_BUILD_TYPE:STRING=Debug "
      "-DCMAKE_PROJECT_INCLUDE_BEFORE:PATH=/usr/share/qtcreator/package-manager/auto-setup.cmake "
      "-DQT_QMAKE_EXECUTABLE:STRING=/usr/lib/qt5/bin/qmake "
      "-DCMAKE_PREFIX_PATH:STRING=/usr "
      "-DCMAKE_C_COMPILER:STRING=/usr/bin/gcc "
      "-DCMAKE_CXX_COMPILER:STRING=/usr/bin/g++ && "
      "/usr/bin/make -C " + build_dir.string();

    int ret_code = system(build_command.c_str());
    if (ret_code != 0) {
      std::cerr << "Error: Failed to build robocon_gui." << std::endl;
      return 1; // Exit with error
    }
  }

  // Run the executable
  std::string run_command = gui_executable.string();
  int run_ret_code = system(run_command.c_str());
  if (run_ret_code != 0) {
    std::cerr << "Error: Failed to run robocon_gui." << std::endl;
    return 1; // Exit with error
  }

  return 0;
}
