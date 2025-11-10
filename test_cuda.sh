#!/bin/bash
# Test CUDA availability after driver installation

python3 << 'EOF'
import torch
print("="*70)
print("PyTorch CUDA Test")
print("="*70)
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA compiled version: {torch.version.cuda}")
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"CUDA device count: {torch.cuda.device_count()}")

if torch.cuda.is_available():
    print(f"CUDA device name: {torch.cuda.get_device_name(0)}")
    print(f"CUDA capability: {torch.cuda.get_device_capability(0)}")

    # Test CUDA functionality
    print("\nTesting CUDA with tensor operation...")
    test_tensor = torch.zeros(10, 10).cuda()
    print(f"✓ Successfully created tensor on GPU: {test_tensor.device}")
    del test_tensor
    print("\n" + "="*70)
    print("SUCCESS! CUDA is fully functional!")
    print("="*70)
else:
    print("\n✗ CUDA not available")
print("="*70)
EOF
