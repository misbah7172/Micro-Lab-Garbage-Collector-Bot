#!/usr/bin/env python3
"""
Configuration Management Script for Micro Lab Garbage Collector
Easily switch between different models and configurations
"""

import os
import shutil
from pathlib import Path

def list_available_models():
    """List all available model files"""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    
    model_paths = []
    
    # Search common model locations
    search_paths = [
        script_dir,  # Final-Model directory
        project_root,  # Project root
        project_root / "models",  # Models directory
        project_root / "Test-Model",  # Test-Model directory
        project_root / "Temp"  # Temp directory
    ]
    
    for search_path in search_paths:
        if search_path.exists():
            for model_file in search_path.glob("*.pt"):
                model_paths.append(model_file)
    
    return sorted(set(model_paths))

def update_env_file(model_path, **kwargs):
    """Update .env file with new model path and other settings"""
    env_file = Path(__file__).parent / ".env"
    
    if not env_file.exists():
        print(".env file not found!")
        return False
    
    # Read current .env file
    with open(env_file, 'r') as f:
        lines = f.readlines()
    
    # Update lines
    updated_lines = []
    updated_keys = set()
    
    for line in lines:
        if line.strip().startswith('MODEL_PATH='):
            updated_lines.append(f"MODEL_PATH={model_path}\n")
            updated_keys.add('MODEL_PATH')
        else:
            # Check for other parameters
            for key, value in kwargs.items():
                if line.strip().startswith(f'{key}='):
                    updated_lines.append(f"{key}={value}\n")
                    updated_keys.add(key)
                    break
            else:
                updated_lines.append(line)
    
    # Add any new parameters that weren't in the file
    for key, value in kwargs.items():
        if key not in updated_keys:
            updated_lines.append(f"{key}={value}\n")
    
    # Write updated .env file
    with open(env_file, 'w') as f:
        f.writelines(updated_lines)
    
    print(f"Updated .env file with MODEL_PATH={model_path}")
    return True

def main():
    """Main configuration management interface"""
    print("Micro Lab Garbage Collector - Configuration Manager")
    print("=" * 60)
    
    while True:
        print("\nAvailable options:")
        print("1. List available models")
        print("2. Change model path")
        print("3. Switch to mock mode")
        print("4. Reset to defaults")
        print("5. Show current configuration")
        print("6. Exit")
        
        choice = input("\nEnter your choice (1-6): ").strip()
        
        if choice == '1':
            print("\nAvailable Models:")
            models = list_available_models()
            if models:
                for i, model in enumerate(models, 1):
                    print(f"   {i}. {model}")
            else:
                print("   No .pt model files found!")
        
        elif choice == '2':
            models = list_available_models()
            if not models:
                print("No model files found!")
                continue
            
            print("\nAvailable Models:")
            for i, model in enumerate(models, 1):
                print(f"   {i}. {model}")
            
            try:
                model_choice = int(input("\nSelect model number: ")) - 1
                if 0 <= model_choice < len(models):
                    selected_model = models[model_choice]
                    # Use relative path if possible
                    script_dir = Path(__file__).parent
                    try:
                        relative_path = selected_model.relative_to(script_dir)
                        update_env_file(str(relative_path))
                    except ValueError:
                        # Use absolute path if relative doesn't work
                        update_env_file(str(selected_model))
                else:
                    print("Invalid selection!")
            except ValueError:
                print("Please enter a valid number!")
        
        elif choice == '3':
            print("\nMock Mode Configuration:")
            print("1. Enable ESP32 mock mode")
            print("2. Enable Camera mock mode")
            print("3. Enable both mock modes")
            print("4. Disable all mock modes")
            
            mock_choice = input("Select option (1-4): ").strip()
            
            if mock_choice == '1':
                update_env_file(model_path="best.pt", MOCK_ESP32="True")
            elif mock_choice == '2':
                update_env_file(model_path="best.pt", MOCK_CAMERA="True")
            elif mock_choice == '3':
                update_env_file(model_path="best.pt", MOCK_ESP32="True", MOCK_CAMERA="True")
            elif mock_choice == '4':
                update_env_file(model_path="best.pt", MOCK_ESP32="False", MOCK_CAMERA="False")
            else:
                print("Invalid selection!")
        
        elif choice == '4':
            confirm = input("Reset to default configuration? (y/N): ")
            if confirm.lower() == 'y':
                update_env_file(
                    "best.pt",
                    CAMERA_INDEX="0",
                    SERIAL_PORT="AUTO",
                    WEB_PORT="5000",
                    MOCK_ESP32="False",
                    MOCK_CAMERA="False"
                )
                print("Configuration reset to defaults!")
        
        elif choice == '5':
            env_file = Path(__file__).parent / ".env"
            if env_file.exists():
                print("\nCurrent Configuration:")
                with open(env_file, 'r') as f:
                    for line in f:
                        if line.strip() and not line.strip().startswith('#'):
                            print(f"   {line.strip()}")
            else:
                print(".env file not found!")
        
        elif choice == '6':
            print("Goodbye!")
            break
        
        else:
            print("Invalid choice! Please select 1-6.")

if __name__ == "__main__":
    main()