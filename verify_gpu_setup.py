#!/usr/bin/env python3
"""
GPU Setup Verification Script

Verifies that all components (PyTorch, Whisper, Ollama) can use the GPU correctly.
Run this after any system reboot or CUDA-related changes.
"""

import sys
import subprocess

def print_header(title):
    print(f"\n{'='*70}")
    print(f"{title:^70}")
    print(f"{'='*70}\n")

def check_nvidia_driver():
    """Check NVIDIA driver and GPU"""
    print_header("NVIDIA Driver & GPU")
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name,driver_version,memory.total,memory.used', '--format=csv,noheader'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            gpu_info = result.stdout.strip().split(',')
            print(f"✓ GPU: {gpu_info[0].strip()}")
            print(f"✓ Driver: {gpu_info[1].strip()}")
            print(f"✓ Total Memory: {gpu_info[2].strip()}")
            print(f"✓ Used Memory: {gpu_info[3].strip()}")
            return True
        else:
            print("✗ nvidia-smi failed")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def check_pytorch_cuda():
    """Check PyTorch CUDA availability"""
    print_header("PyTorch CUDA")
    try:
        import torch
        print(f"✓ PyTorch version: {torch.__version__}")
        print(f"✓ CUDA compiled version: {torch.version.cuda}")

        if torch.cuda.is_available():
            print(f"✓ CUDA available: True")
            print(f"✓ CUDA device count: {torch.cuda.device_count()}")
            print(f"✓ Current device: {torch.cuda.current_device()}")
            print(f"✓ Device name: {torch.cuda.get_device_name(0)}")
            print(f"✓ Device capability: {torch.cuda.get_device_capability(0)}")

            # Test tensor creation on GPU
            test_tensor = torch.zeros(1).cuda()
            print(f"✓ GPU tensor test: SUCCESS")
            del test_tensor
            return True
        else:
            print("✗ CUDA not available")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def check_ollama():
    """Check Ollama GPU usage"""
    print_header("Ollama GPU Usage")
    try:
        import requests
        response = requests.get('http://localhost:11434/api/ps', timeout=5)
        if response.status_code == 200:
            data = response.json()
            models = data.get('models', [])
            if models:
                for model in models:
                    vram_usage = model.get('size_vram', 0)
                    if vram_usage > 0:
                        vram_gb = vram_usage / (1024**3)
                        print(f"✓ Model '{model['name']}' using GPU: {vram_gb:.2f} GB VRAM")
                    else:
                        print(f"✗ Model '{model['name']}' NOT using GPU")
                return True
            else:
                print("ℹ No models currently loaded (start Ollama to test)")
                return True
        else:
            print("✗ Ollama not running")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def check_whisper_rosie():
    """Check Whisper/ROSIE GPU detection"""
    print_header("Whisper/ROSIE GPU Detection")
    try:
        sys.path.insert(0, '/home/mike/projects/b4m_yahboom')
        from rosie_conversation import RosieConversation

        print("Creating ROSIE instance...")
        rosie = RosieConversation()

        print("Loading Whisper model...")
        rosie._load_whisper_model()

        # Check model device
        import torch
        if rosie.whisper_model is not None:
            for param in rosie.whisper_model.parameters():
                device = str(param.device)
                if 'cuda' in device:
                    print(f"✓ Whisper model on GPU: {device}")
                    return True
                else:
                    print(f"✗ Whisper model on CPU: {device}")
                    return False
                break
        else:
            print("✗ Whisper model not loaded")
            return False
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all verification checks"""
    print("\n" + "="*70)
    print("GPU Setup Verification")
    print("="*70)

    results = {}
    results['nvidia_driver'] = check_nvidia_driver()
    results['pytorch_cuda'] = check_pytorch_cuda()
    results['ollama'] = check_ollama()
    results['whisper_rosie'] = check_whisper_rosie()

    # Summary
    print_header("SUMMARY")

    all_pass = True
    for component, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{component:20s}: {status}")
        if not passed:
            all_pass = False

    print("\n" + "="*70)
    if all_pass:
        print("✓ ALL CHECKS PASSED - GPU is fully functional!")
    else:
        print("⚠ SOME CHECKS FAILED - Review output above")
    print("="*70 + "\n")

    return 0 if all_pass else 1

if __name__ == '__main__':
    sys.exit(main())
