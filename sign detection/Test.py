import cv2
import depthai as dai
from ultralytics import YOLO

model = YOLO('yolov8n.pt')

pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.Camera).build()
output = cam.requestOutput((640, 640), type=dai.ImgFrame.Type.BGR888p)
q_rgb = output.createOutputQueue()

pipeline.start()
while pipeline.isRunning():
    in_rgb = q_rgb.get()
    frame = in_rgb.getCvFrame()

    results = model(frame, classes=[11], verbose=False)
    annotated = results[0].plot()

    cv2.imshow("Stop Sign Detection", annotated)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()