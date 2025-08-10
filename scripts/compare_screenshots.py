#!/usr/bin/env python3
"""
Screenshot Comparison Tool for Regression Testing

Compares actual screenshots with reference screenshots and determines similarity percentage.
Uses multiple comparison methods to account for small differences in timing and rendering.
"""

import cv2
import numpy as np
import argparse
import sys
from pathlib import Path
from skimage.metrics import structural_similarity as ssim
import json

class ScreenshotComparator:
    def __init__(self, similarity_threshold=0.90):
        self.similarity_threshold = similarity_threshold
        self.comparison_results = {}
        
    def load_and_resize_images(self, img1_path, img2_path):
        """Load two images and resize them to the same dimensions if needed"""
        try:
            img1 = cv2.imread(str(img1_path))
            img2 = cv2.imread(str(img2_path))
            
            if img1 is None:
                raise ValueError(f"Could not load image: {img1_path}")
            if img2 is None:
                raise ValueError(f"Could not load image: {img2_path}")
                
            # Get dimensions
            h1, w1 = img1.shape[:2]
            h2, w2 = img2.shape[:2]
            
            # Resize to smaller dimensions if they differ
            if (h1, w1) != (h2, w2):
                min_h, min_w = min(h1, h2), min(w1, w2)
                img1 = cv2.resize(img1, (min_w, min_h))
                img2 = cv2.resize(img2, (min_w, min_h))
                print(f"   Resized images to {min_w}x{min_h}")
                
            return img1, img2
            
        except Exception as e:
            print(f"❌ Error loading images: {e}")
            return None, None
    
    def calculate_histogram_similarity(self, img1, img2):
        """Calculate similarity using color histogram comparison"""
        # Convert to HSV for better color comparison
        hsv1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
        
        # Calculate histograms
        hist1 = cv2.calcHist([hsv1], [0, 1, 2], None, [50, 60, 60], [0, 180, 0, 256, 0, 256])
        hist2 = cv2.calcHist([hsv2], [0, 1, 2], None, [50, 60, 60], [0, 180, 0, 256, 0, 256])
        
        # Compare using correlation
        similarity = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
        return max(0, similarity)  # Ensure non-negative
    
    def calculate_ssim_similarity(self, img1, img2):
        """Calculate similarity using Structural Similarity Index"""
        # Convert to grayscale
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        # Calculate SSIM
        similarity, _ = ssim(gray1, gray2, full=True)
        return max(0, similarity)  # Ensure non-negative
    
    def calculate_feature_similarity(self, img1, img2):
        """Calculate similarity using ORB feature matching"""
        try:
            # Convert to grayscale
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            
            # Create ORB detector
            orb = cv2.ORB_create(nfeatures=1000)
            
            # Detect keypoints and descriptors
            kp1, des1 = orb.detectAndCompute(gray1, None)
            kp2, des2 = orb.detectAndCompute(gray2, None)
            
            if des1 is None or des2 is None:
                return 0.0
                
            # Match features
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)
            
            # Calculate similarity based on number of good matches
            if len(kp1) == 0 or len(kp2) == 0:
                return 0.0
                
            good_matches = len(matches)
            max_possible_matches = min(len(kp1), len(kp2))
            
            if max_possible_matches == 0:
                return 0.0
                
            similarity = good_matches / max_possible_matches
            return min(1.0, similarity)  # Cap at 1.0
            
        except Exception as e:
            print(f"   Warning: Feature matching failed: {e}")
            return 0.0
    
    def calculate_template_matching(self, img1, img2):
        """Calculate similarity using template matching on key regions"""
        try:
            # Convert to grayscale
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            
            # Use smaller image as template, larger as source
            if gray1.size < gray2.size:
                template, source = gray1, gray2
            else:
                template, source = gray2, gray1
                
            # Perform template matching
            result = cv2.matchTemplate(source, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(result)
            
            return max(0, max_val)
            
        except Exception as e:
            print(f"   Warning: Template matching failed: {e}")
            return 0.0
    
    def compare_images(self, actual_path, reference_path, comparison_name="comparison"):
        """Compare two images using multiple methods and return weighted similarity score"""
        print(f"\n🔍 Comparing {comparison_name}:")
        print(f"   Actual: {actual_path}")
        print(f"   Reference: {reference_path}")
        
        # Load and resize images
        img1, img2 = self.load_and_resize_images(actual_path, reference_path)
        if img1 is None or img2 is None:
            return 0.0, {"error": "Could not load images"}
        
        # Calculate different similarity metrics
        hist_sim = self.calculate_histogram_similarity(img1, img2)
        ssim_sim = self.calculate_ssim_similarity(img1, img2)
        feature_sim = self.calculate_feature_similarity(img1, img2)
        template_sim = self.calculate_template_matching(img1, img2)
        
        # Weighted average (SSIM and histogram are most important for our use case)
        weights = {
            'histogram': 0.3,   # Color distribution
            'ssim': 0.4,        # Structural similarity
            'features': 0.2,    # Feature matching
            'template': 0.1     # Overall template match
        }
        
        weighted_similarity = (
            hist_sim * weights['histogram'] +
            ssim_sim * weights['ssim'] +
            feature_sim * weights['features'] +
            template_sim * weights['template']
        )
        
        # Store detailed results (ensure all types are JSON serializable)
        details = {
            'histogram_similarity': float(hist_sim),
            'ssim_similarity': float(ssim_sim),
            'feature_similarity': float(feature_sim),
            'template_similarity': float(template_sim),
            'weighted_similarity': float(weighted_similarity),
            'threshold': float(self.similarity_threshold),
            'passed': bool(weighted_similarity >= self.similarity_threshold)
        }
        
        # Print results
        print(f"   Histogram similarity: {hist_sim:.3f}")
        print(f"   SSIM similarity: {ssim_sim:.3f}")
        print(f"   Feature similarity: {feature_sim:.3f}")
        print(f"   Template similarity: {template_sim:.3f}")
        print(f"   📊 Weighted similarity: {weighted_similarity:.3f}")
        
        if weighted_similarity >= self.similarity_threshold:
            print(f"   ✅ PASS (≥{self.similarity_threshold:.1%})")
        else:
            print(f"   ❌ FAIL (<{self.similarity_threshold:.1%})")
            
        return weighted_similarity, details
    
    def compare_screenshot_set(self, actual_dir, reference_dir, output_file=None):
        """Compare a set of screenshots and return overall results"""
        actual_dir = Path(actual_dir)
        reference_dir = Path(reference_dir)
        
        print("🖼️  SCREENSHOT COMPARISON")
        print("=" * 50)
        
        comparisons = [
            ("initial", "rotation_initial_", "reference_initial.png"),
            ("mid-rotation", "rotation_mid_", "reference_mid.png"),  
            ("final", "rotation_final_", "reference_final.png")
        ]
        
        results = {}
        all_passed = True
        
        for name, actual_prefix, ref_filename in comparisons:
            # Find the most recent actual screenshot
            actual_files = list(actual_dir.glob(f"{actual_prefix}*.png"))
            if not actual_files:
                print(f"\n❌ No actual screenshot found for {name} (pattern: {actual_prefix}*.png)")
                results[name] = {"error": "Actual screenshot not found", "passed": False}
                all_passed = False
                continue
                
            # Use most recent file (sorted by name which includes timestamp)
            actual_file = sorted(actual_files)[-1]
            reference_file = reference_dir / ref_filename
            
            if not reference_file.exists():
                print(f"\n❌ Reference screenshot not found: {reference_file}")
                results[name] = {"error": "Reference screenshot not found", "passed": False}
                all_passed = False
                continue
            
            # Perform comparison
            similarity, details = self.compare_images(actual_file, reference_file, name)
            results[name] = details
            
            if not details.get("passed", False):
                all_passed = False
        
        # Overall results
        print(f"\n" + "=" * 50)
        print("📋 OVERALL COMPARISON RESULTS")
        print("=" * 50)
        
        for name, result in results.items():
            if "error" in result:
                print(f"   {name.upper()}: ❌ ERROR - {result['error']}")
            else:
                status = "✅ PASS" if result["passed"] else "❌ FAIL"
                similarity = result.get("weighted_similarity", 0)
                print(f"   {name.upper()}: {status} ({similarity:.1%})")
        
        print(f"\n🎯 FINAL RESULT: {'✅ ALL COMPARISONS PASSED' if all_passed else '❌ SOME COMPARISONS FAILED'}")
        
        # Save detailed results if requested
        if output_file:
            with open(output_file, 'w') as f:
                json.dump({
                    'overall_passed': bool(all_passed),
                    'threshold': float(self.similarity_threshold),
                    'comparisons': results
                }, f, indent=2)
            print(f"📄 Detailed results saved to: {output_file}")
        
        return all_passed, results

def main():
    parser = argparse.ArgumentParser(description="Compare regression test screenshots with references")
    parser.add_argument("--actual-dir", default="regression/screenshots", 
                       help="Directory containing actual screenshots")
    parser.add_argument("--reference-dir", default="regression/reference_screenshots",
                       help="Directory containing reference screenshots")  
    parser.add_argument("--threshold", type=float, default=0.90,
                       help="Similarity threshold (0.0-1.0, default: 0.90)")
    parser.add_argument("--output", help="Save detailed results to JSON file")
    parser.add_argument("--single", nargs=2, metavar=('ACTUAL', 'REFERENCE'),
                       help="Compare two specific images")
    
    args = parser.parse_args()
    
    comparator = ScreenshotComparator(args.threshold)
    
    if args.single:
        # Single image comparison
        actual_path, reference_path = args.single
        similarity, details = comparator.compare_images(actual_path, reference_path)
        return 0 if details.get("passed", False) else 1
    else:
        # Full screenshot set comparison
        all_passed, results = comparator.compare_screenshot_set(
            args.actual_dir, args.reference_dir, args.output
        )
        return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())