# build.py
import subprocess
import sys
import PyInstaller.__main__

def build():
    # Define hidden imports and modules required by Pygame/your project
    args = [
        'main.py',                         # Main entry point
        '--name=PizzaDroneSimulation',      # Name of the output executable
        '--onefile',                       # Bundle into a single standalone EXE
        '--windowed',                      # Hide command prompt window on launch
        '--clean',                         # Clean cache before building
        
        # Explicitly include project modules to prevent import errors
        '--hidden-import=pygame',
        '--hidden-import=camera',
        '--hidden-import=contracts',
        '--hidden-import=editor',
        '--hidden-import=order_spawner',
        '--hidden-import=pygame_ui',
        '--hidden-import=router',
        
        # Include data files if you have custom JSON scenarios or assets
        # '--add-data=scenario.json;.',  # Format: "source;destination_folder"
    ]

    print("Building executable...")
    PyInstaller.__main__.run(args)
    print("Build complete! Check the 'dist/' folder.")

if __name__ == "__main__":
    build()