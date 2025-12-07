#!/bin/bash

# Define the source of your old project
# Based on your pwd context: ~/c/STM32CubeIDE/FINALLYSTM/
OLD_PROJECT_PATH="$HOME/c/STM32CubeIDE/FINALLYSTM"

# Verify source exists
if [ ! -d "$OLD_PROJECT_PATH" ]; then
  echo "Error: Old project path not found at $OLD_PROJECT_PATH"
  exit 1
fi

echo "Starting migration from $OLD_PROJECT_PATH..."

# 1. Copy Core Source files (Overwrites main.c, copies MahonyAHRS.c, etc.)
echo "Copying Core/Src..."
cp -r "$OLD_PROJECT_PATH/Core/Src/"* Core/Src/

# 2. Copy Core Include files (Overwrites main.h, copies MahonyAHRS.h, etc.)
echo "Copying Core/Inc..."
cp -r "$OLD_PROJECT_PATH/Core/Inc/"* Core/Inc/

# 3. Copy USB Device Logic (Crucial for CDC if modifications were made)
if [ -d "$OLD_PROJECT_PATH/USB_DEVICE" ]; then
  echo "Copying USB_DEVICE..."
  # We copy recursively to catch App and Target folders
  cp -r "$OLD_PROJECT_PATH/USB_DEVICE/"* USB_DEVICE/
else
  echo "Warning: USB_DEVICE folder not found in old project. Skipping."
fi

echo "------------------------------------------------"
echo "Migration Complete!"
echo "1. Open STM32CubeIDE."
echo "2. Right-click the project -> Refresh (F5)."
echo "3. Clean and Build."
