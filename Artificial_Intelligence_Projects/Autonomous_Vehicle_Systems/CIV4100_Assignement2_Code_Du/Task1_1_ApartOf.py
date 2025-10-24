


# Task1_1_ApartOfanalysis.py

import os
from collections import defaultdict
from PIL import Image
import numpy as np
from torchvision import datasets, transforms
from torch.utils.data import random_split
import torch

# 
data_root = r"C:/Users/dwx/Desktop/CIV4100_2/traffic_sign_final_dataset/traffic_sign_final_dataset/traffic_sign_final_dataset/Train"


class_counts = defaultdict(int)
image_shapes = []

print(" Starting full dataset analysis...\n")

# Loop through each class, read all images with live progress
for class_name in sorted(os.listdir(data_root)):
    class_path = os.path.join(data_root, class_name)
    if not os.path.isdir(class_path):
        continue

    print(f" Reading class {class_name} ...")
    img_files = [f for f in os.listdir(class_path) if f.lower().endswith((".jpg", ".jpeg", ".png"))]

    for i, fname in enumerate(img_files):
        fpath = os.path.join(class_path, fname)
        try:
            with Image.open(fpath) as img:
                class_counts[class_name] += 1
                image_shapes.append(img.size)
        except:
            print(f"   Failed to read: {fpath}")
        
        if (i + 1) % 100 == 0:
            print(f"   Processed {i + 1} images...")

    print(f"   Finished class {class_name}: {len(img_files)} files total\n")

print(" Finished reading all classes. Calculating statistics...\n")

# Print class counts
print("Number of images per class:")
for cls, count in sorted(class_counts.items(), key=lambda x: int(x[0])):
    print(f"  Class {cls}: {count} images")

# Image size stats
if image_shapes:
    shapes_array = np.array(image_shapes)
    mean_w, mean_h = np.mean(shapes_array, axis=0)
    min_w, min_h = np.min(shapes_array, axis=0)
    max_w, max_h = np.max(shapes_array, axis=0)

    print("\n Image size summary:")
    print(f"  Mean size: {mean_w:.1f} x {mean_h:.1f}")
    print(f"  Min size: {min_w} x {min_h}")
    print(f"  Max size: {max_w} x {max_h}")
else:
    print(" No image data found.")



print("\nPreparing data for model input...")

# Define transformation: resize and convert to tensor
resize_transform = transforms.Compose([
    transforms.Resize((64, 64)),
    transforms.ToTensor()
])

# Reload dataset using ImageFolder with transform applied
dataset = datasets.ImageFolder(root=data_root, transform=resize_transform)

# Split into 80% train, 20% val
dataset_size = len(dataset)
train_size = int(0.8 * dataset_size)
val_size = dataset_size - train_size

# Fixed random seed for reproducibility
train_set, val_set = random_split(dataset, [train_size, val_size], generator=torch.Generator().manual_seed(42))

# Output summary
print("\nDataset preparation complete:")
print(f"  Total images: {dataset_size}")
print(f"  Training set size: {len(train_set)}")
print(f"  Validation set size: {len(val_set)}")
print(f"  Number of classes: {len(dataset.classes)}")




