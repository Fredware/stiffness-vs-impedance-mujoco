import os
import sys

# Ensure the 'src' directory is in the python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from sim import HelloMujoco

def main():
    # Robustly find the asset path regardless of where the script is run from
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xml_path = os.path.join(project_root, "assets", "scene.xml")
    
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"Could not find asset at: {xml_path}")

    print("Initializing Hello World Simulation...")
    sim = HelloMujoco(xml_path)
    sim.run()

if __name__ == "__main__":
    main()