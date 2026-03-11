import cv2
import numpy as np
import glob
import os

CHECKERBOARD = (8, 5)
SQUARE_SIZE = 19.0
IMAGE_DIR = "calibration_images/*.jpg"

print("=" * 70)
print("ESP32-CAM CALIBRATION SCRIPT")
print("=" * 70)
print(f"Checkerboard: {CHECKERBOARD[0]+1}x{CHECKERBOARD[1]+1} squares ({CHECKERBOARD[0]}x{CHECKERBOARD[1]} internal corners)")
print(f"Square size: {SQUARE_SIZE}mm")
print("=" * 70)

os.makedirs("calibration_images", exist_ok=True)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

obj_points = []
img_points = []

images = glob.glob(IMAGE_DIR)

if len(images) == 0:
    print("\nNo images found in 'calibration_images/' folder!")
    print("\nInstructions:")
    print("1. Create a folder called 'calibration_images'")
    print("2. Capture 15-20 images from your ESP32-CAM showing the checkerboard:")
    print("   - Different angles")
    print("   - Different positions (corners, center, edges)")
    print("   - Different distances")
    print("   - Make sure the checkerboard is flat and fully visible")
    print("3. Save images as: calibration_images/img_001.jpg, img_002.jpg, etc.")
    print("4. Run this script again")
    exit()

print(f"\nFound {len(images)} calibration images")
print("\nProcessing images...\n")

successful = 0
failed = 0

for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    if img is None:
        print(f"Could not read: {fname}")
        failed += 1
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        obj_points.append(objp)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        img_points.append(corners2)

        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        display_img = img.copy()
        if display_img.shape[1] > 800:
            scale = 800 / display_img.shape[1]
            display_img = cv2.resize(display_img, None, fx=scale, fy=scale)

        cv2.imshow("Detected Corners", display_img)
        cv2.waitKey(200)

        successful += 1
        print(f"{idx+1}/{len(images)}: {os.path.basename(fname)} - corners found")
    else:
        failed += 1
        print(f"{idx+1}/{len(images)}: {os.path.basename(fname)} - no corners detected")

cv2.destroyAllWindows()

print("\n" + "=" * 70)
print(f"Results: {successful} successful, {failed} failed")
print("=" * 70)

if successful < 10:
    print("\nWarning: less than 10 successful images.")
    print("Aim for at least 15-20 for reliable calibration.")
    print("\nTips:")
    print("- Ensure even lighting with no shadows on the board")
    print("- Keep the checkerboard flat")
    print("- Make sure all corners are in frame")
    print("- Avoid motion blur")

if successful == 0:
    print("\nNo valid calibration images found, exiting.")
    exit()

print("\nRunning calibration...")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)

mean_error = 0
for i in range(len(obj_points)):
    imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

mean_error /= len(obj_points)

print("\n" + "=" * 70)
print("CALIBRATION RESULTS")
print("=" * 70)
print("\nCamera matrix (K):")
print(camera_matrix)
print("\nDistortion coefficients:")
print(dist_coeffs)
print(f"\nMean reprojection error: {mean_error:.4f} pixels")

if mean_error < 0.5:
    print("Excellent calibration.")
elif mean_error < 1.0:
    print("Good calibration.")
elif mean_error < 2.0:
    print("Acceptable calibration (consider recapturing with better images).")
else:
    print("Poor calibration - recapture with better images.")

np.save("camera_matrix.npy", camera_matrix)
np.save("dist_coeffs.npy", dist_coeffs)
print("\nSaved camera_matrix.npy and dist_coeffs.npy")

fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

print("\n" + "=" * 70)
print("CAMERA PARAMETERS")
print("=" * 70)
print(f"Focal length X (fx): {fx:.2f} px")
print(f"Focal length Y (fy): {fy:.2f} px")
print(f"Principal point (cx, cy): ({cx:.2f}, {cy:.2f})")
print(f"Image center: ({gray.shape[1]/2:.1f}, {gray.shape[0]/2:.1f})")

print("\nTesting undistortion on first image...")
test_img = cv2.imread(images[0])
if test_img is not None:
    h, w = test_img.shape[:2]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(test_img, camera_matrix, dist_coeffs, None, newcameramtx)

    comparison = np.hstack([test_img, undistorted])
    cv2.imwrite("calibration_test.jpg", comparison)
    print("Saved calibration_test.jpg (original | undistorted)")

    x, y, w, h = roi
    if x > 0 or y > 0:
        undistorted_cropped = undistorted[y:y+h, x:x+w]
        cv2.imwrite("calibration_test_cropped.jpg", undistorted_cropped)
        print("Saved calibration_test_cropped.jpg")

print("\n" + "=" * 70)
print("Calibration complete.")
print("=" * 70)