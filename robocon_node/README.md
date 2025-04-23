## 🔄 Exporting YOLOv8 Models

To use your YOLOv8 model with OpenVINO, you need to export it first. Use the command below to export the model:

```bash
yolo export model=yolov8s.pt imgsz=640 format=openvino
```