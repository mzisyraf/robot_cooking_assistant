import os
import torch
import ultralytics
from roboflow import Roboflow
from ultralytics import YOLO

# Check if CUDA (GPU) is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Using device: {device}\n")

# Get the current working directory
HOME = os.getcwd()
print(f"Current working directory: {HOME}\n")

# Run ultralytics checks
ultralytics.checks()

# Create a directory for datasets
datasets_dir = os.path.join(HOME, 'datasets')
os.makedirs(datasets_dir, exist_ok=True)
print(f"Datasets directory: {datasets_dir}\n")

# Change the working directory to datasets directory
os.chdir(datasets_dir)

# Dataset location
dataset_name = "FOOD-INGREDIENTS-dataset-4"
dataset_path = os.path.join(datasets_dir, dataset_name)

# Check if dataset is already downloaded
if not os.path.exists(dataset_path):
    # Initialize Roboflow and download the dataset
    rf = Roboflow(api_key="xt6XkLuFy89FQJajxuIh")
    project = rf.workspace(
        "food-recipe-ingredient-images-0gnku").project("food-ingredients-dataset")
    version = project.version(4)
    dataset = version.download("yolov8")
    dataset_path = dataset.location
    print(f"Dataset downloaded to: {dataset_path}")
else:
    print(f"Dataset already exists at: {dataset_path}")

# Verify the dataset structure
if not os.path.exists(os.path.join(dataset_path, 'data.yaml')):
    raise FileNotFoundError(f"'data.yaml' not found in {dataset_path}\n")

# Change back to the original HOME directory
os.chdir(HOME)

# Train the YOLOv8 model using the Python API and the GPU if available
model = YOLO('yolov8s.pt').to(device)

# Check if valid images directory exists
valid_images_dir = os.path.join(dataset_path, 'valid', 'images')
if not os.path.exists(valid_images_dir):
    raise FileNotFoundError(
        f"Valid images directory not found: {valid_images_dir}")

# Start training
model.train(data=os.path.join(dataset_path, 'data.yaml'),
            epochs=25, imgsz=800, plots=True, device=device)
