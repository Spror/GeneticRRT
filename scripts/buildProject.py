import subprocess
import os

def build_project(source_dir, build_dir):
    # Create build directory if it doesn't exist
    if not os.path.exists(build_dir):
        os.makedirs(build_dir)
    
    # Change directory to the build directory
    os.chdir(build_dir)
    
    # Execute CMake to configure the project
    cmake_configure_cmd = ["cmake", source_dir]
    subprocess.run(cmake_configure_cmd, check=True)
    
    # Execute CMake to build the project
    cmake_build_cmd = ["cmake", "--build", ".", "--config", "Release"]
    subprocess.run(cmake_build_cmd, check=True)

if __name__ == "__main__":
    # Replace these paths with your source and build directories
    source_directory = "/home/wiktor/cxx/Sampling-basedMP"
    build_directory = "/home/wiktor/cxx/Sampling-basedMP/build"
    
    build_project(source_directory, build_directory)
