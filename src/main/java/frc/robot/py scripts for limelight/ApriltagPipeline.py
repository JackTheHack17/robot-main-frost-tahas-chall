import cv2
import numpy as np
import apriltag

# Create an AprilTag detector object
detector = apriltag.Detector()

text = "APRILTAG"
font = cv2.FONT_HERSHEY_SIMPLEX
org = (50,50)
fontScale = 1
color = (0,255,0)
thickness = 2

# Load the Limelight camera parameters
# Have to adjust these two for lemlight
focal_length = 998.2  # focal length of the camera in pixels
principal_point = (640/2, 480/2)  # principal point of the camera in pixels

def runPipeline():
    # Open the camera
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the grayscale frame
        result = detector.detect(gray)

        # Draw the AprilTag detections
        for tag in result:
            corners = tag.corners
            cv2.polylines(frame, [np.int32(corners)], True, (0, 0, 255), 2)

            # Estimate the distance and position of the AprilTag
            tag_size = 0.1  # physical size of the tag in meters
            center = np.mean(corners, axis=0)
            x, y = center[0], center[1]
            z = focal_length * tag_size / (2 * tag.tag_size)
            position = np.array([x, y, z])

            # Print the position of the AprilTag
            print("Position:", position)
        
        cv2.putText(frame, text, org, font, color, thickness)
        # Display the frame with the AprilTag detections
        cv2.imshow("AprilTag Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()

# Call the pipeline function - Don't need this for the lemlight
runPipeline()
