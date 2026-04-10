import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory

def main():
    # Load the known face image
    known_image_path = get_package_share_directory('demo_python_service') + "/resource/zidane.jpg"
    image = cv2.imread(known_image_path)
    face_locations = face_recognition.face_locations(image)
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
    cv2.imshow('Known Face', image)
    cv2.waitKey(0)

    # known_image = face_recognition.load_image_file(known_image_path)
    # known_face_encoding = face_recognition.face_encodings(known_image)[0]

    # # Initialize the webcam
    # video_capture = cv2.VideoCapture(0)

    # while True:
    #     # Capture a single frame from the webcam
    #     ret, frame = video_capture.read()

    #     # Convert the frame from BGR to RGB color space
    #     rgb_frame = frame[:, :, ::-1]

    #     # Find all the faces and face encodings in the current frame
    #     face_locations = face_recognition.face_locations(rgb_frame)
    #     face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    #     for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
    #         # Compare the detected face encoding with the known face encoding
    #         matches = face_recognition.compare_faces([known_face_encoding], face_encoding)

    #         if matches[0]:
    #             # If a match is found, draw a box around the face and label it
    #             cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
    #             cv2.putText(frame, "Zidane", (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 1)

    #     # Display the resulting frame
    #     cv2.imshow('Video', frame)

    #     # Exit on 'q' key press
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # # Release the webcam and close windows
    # video_capture.release()
    cv2.destroyAllWindows()