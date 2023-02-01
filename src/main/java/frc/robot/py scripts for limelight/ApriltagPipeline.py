import cv2
import numpy as np
from pupil_apriltags import Detector

tag_id = None
def runPipeline(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Define the AprilTag detector with the 16h5 family
    detector = Detector(families='16h5', nthreads=1, quad_decimate=1.0,
                        quad_sigma=0.0, refine_edges=True, decode=True,
                        Debug=False)

    # Detect AprilTags in the frame
    tags = detector.detect(frame, gray, return_image=False, camera_params=None)

    if len(tags) > 0:
        # Find the tag with the largest area
        tag = max(tags, key=lambda x: x.area)
        center = tag.center
        tag_id = tags.tag_id
        x, y = center[0], center[1]

        # Calculate the distance using the size of the tag in the frame
        focal_length = 100
        tag_size = 0.1  # meters
        pixel_width = tag.corners[2][0] - tag.corners[0][0]
        distance = (tag_size * focal_length) / pixel_width

        # Calculate the position using the Limelight pipeline
        x_image_center = frame.shape[1] / 2
        y_image_center = frame.shape[0] / 2
        x_offset = x - x_image_center
        y_offset = y - y_image_center
        position = np.array([x_offset, y_offset, distance])

        return distance, position, tag_id, x, y
    else:
        return None, None


# Capture video from a camera
cap = cv2.VideoCapture(0)

# Loop over the frames
while True:
    # Capture a frame
    ret, frame = cap.read()

    # Run the Limelight pipeline on the frame
    distance, position = runPipeline(frame)

    # Display the information of the AprilTag
    # I don't understand why x,y shows as undefined - Have to find solution later
    if distance is not None and position is not None:
        cv2.putText(frame, f"ID: {tag_id}", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Distance: {distance:.2f} m", (x,y + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})", (x, y + 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Display the frame
    cv2.imshow("Frame", frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture
cap.release()

# Destroy all windows
cv2.destroyAllWindows()
