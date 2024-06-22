from roboflow import Roboflow
import ultralytics
from ultralytics import YOLO
import os

# Check if NVIDIA GPU is available
os.system('nvidia-smi')

# Get the current working directory
HOME = os.getcwd()
print(f"Current working directory: {HOME}")


# Run ultralytics checks
ultralytics.checks()

# Create a directory for datasets
datasets_dir = os.path.join(HOME, 'datasets')
os.makedirs(datasets_dir, exist_ok=True)
print(f"Datasets directory: {datasets_dir}")

# Change the working directory to datasets directory
os.chdir(datasets_dir)


# Initialize Roboflow and download the dataset
rf = Roboflow(api_key="xt6XkLuFy89FQJajxuIh")
project = rf.workspace(
    "food-recipe-ingredient-images-0gnku").project("food-ingredients-dataset")
version = project.version(4)
dataset = version.download("yolov8")

print(f"Dataset location: {dataset.location}")

# List the contents of the dataset location
os.system(f'ls {dataset.location}')

# Change the working directory back to the original HOME directory
os.chdir(HOME)

# Train the YOLOv8 model
os.system(
    f'yolo task=detect mode=train model=yolov8s.pt data={dataset.location}/data.yaml epochs=25 imgsz=800 plots=True')
