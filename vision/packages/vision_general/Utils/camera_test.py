import cv2
import sys


class ZedSimulator():
    def __init__(self, video_id:int=0):
        """Initialize the node and the camera source."""
        print(f"ZedSimulator has started with video ID: {video_id}")
        self.cap = cv2.VideoCapture(video_id)

    def run(self):
        """Get frames from the webcam and publish them."""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    
    print("Arguments passed:", sys.argv)
    
    if len(sys.argv) > 1:
        video_id = int(sys.argv[1])
        print("Video ID:", video_id)
    else:
        video_id = 0
        print("No video ID provided, using default 0")

    simulator = ZedSimulator(video_id)
    simulator.run()