import cv2
import numpy as np
# from pyngrok import ngrok  # for creating a temporary public URL (optional)

# Function to initialize ARToolKit
def initialize_ar_toolkit():
    # Initialize ARToolKit parameters and settings
    # Set camera calibration parameters, marker patterns, etc.
    # This may involve loading a camera calibration file or specifying intrinsic parameters.

    # Example:
    ar_params = cv2.aruco.DetectorParameters_create()
    ar_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    return ar_params, ar_dict

# Function to detect markers
def detect_markers(frame, ar_params, ar_dict):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ar_dict, parameters=ar_params)

    # Draw marker outlines
    if ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    return frame, corners, ids

# Function for camera calibration
def calibrate_camera(images, ar_params, ar_dict):
    # Collect calibration images with detected markers
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_points, img_points = [], []

    for image in images:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ar_dict, parameters=ar_params)

        if ids is not None:
            obj_points.append(np.zeros((len(ids), 3), dtype=np.float32))
            obj_points[-1][:, :2] = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
            img_points.append(corners)

    # Perform camera calibration
    _, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    return mtx, dist

# Main program
def main():
    # Initialize camera or video stream
    cap = cv2.VideoCapture(0)  # Use 0 for default camera, adjust if using an external camera

    # Initialize ARToolKit
    ar_params, ar_dict = initialize_ar_toolkit()

    # Create a list to store calibration images
    calibration_images = []

    while True:
        # Capture frame
        ret, frame = cap.read()

        # Detect markers
        frame, corners, ids = detect_markers(frame, ar_params, ar_dict)

        # Display the frame with markers
        cv2.imshow("AR Marker Detection", frame)

        # Capture calibration images when a key is pressed (e.g., 'c')
        key = cv2.waitKey(1)
        if key == ord('c'):
            calibration_images.append(frame.copy())
            print(f"Captured {len(calibration_images)} calibration images.")

        # Exit the loop when 'q' is pressed
        elif key == ord('q'):
            break

    # Perform camera calibration using captured images
    camera_matrix, distortion_coefficients = calibrate_camera(calibration_images, ar_params, ar_dict)

    # Print camera matrix and distortion coefficients
    print("Camera Matrix:")
    print(camera_matrix)
    print("Distortion Coefficients:")
    print(distortion_coefficients)

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
