import cv2
from ultralytics import YOLO

CAMERA_INDEX = 0
MODEL_PATH = r"D:\trialscara\runs\detect\ball_model\weights\best.pt"

# 🎯 ROI Limits (Main code er sathe match kore triple width kora holo)
ROI_X_MIN = 200
ROI_X_MAX = 440


def main():
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    small_radius, large_radius = None, None
    threshold = None

    print("===========================================")
    print("📏 RADIUS CALIBRATOR STARTED!")
    print("1. ROI er majhe choto object rekhe '1' chapun.")
    print("2. ROI er majhe boro object rekhe '2' chapun.")
    print("3. Live test korun, tarpor 'Enter' chepe save & exit korun.")
    print("===========================================")

    while True:
        ret, frame = cap.read()
        if not ret: break

        H, W, _ = frame.shape
        cv2.line(frame, (ROI_X_MIN, 0), (ROI_X_MIN, H), (255, 0, 0), 2)
        cv2.line(frame, (ROI_X_MAX, 0), (ROI_X_MAX, H), (255, 0, 0), 2)

        results = model(frame, conf=0.5, verbose=False)
        current_radius = 0
        cx = 0

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            current_radius = ((x2 - x1) + (y2 - y1)) / 4.0
            color = model.names[int(box.cls[0])].lower()

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"R: {current_radius:.1f}px", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)

            if threshold is not None and (ROI_X_MIN < cx < ROI_X_MAX):
                size_label = "LARGE" if current_radius >= threshold else "SMALL"
                live_text = f"LIVE DETECT: {color}_{size_label}"
                cv2.putText(frame, live_text, (x1, y1 - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

        cv2.putText(frame, f"Small: {small_radius if small_radius else 'Not Set'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Large: {large_radius if large_radius else 'Not Set'}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        if threshold: cv2.putText(frame, f"Threshold: {threshold:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                  (0, 255, 0), 2)

        cv2.imshow("Radius Calibrator", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('1') and current_radius > 0:
            small_radius = current_radius
            print(f"✅ Small Radius Set: {small_radius:.1f} px")
            if large_radius: threshold = (small_radius + large_radius) / 2.0
        elif key == ord('2') and current_radius > 0:
            large_radius = current_radius
            print(f"✅ Large Radius Set: {large_radius:.1f} px")
            if small_radius: threshold = (small_radius + large_radius) / 2.0
        elif key == 13:  # Enter Key
            if small_radius and large_radius:
                with open("threshold.txt", "w") as f:
                    f.write(str(threshold))
                print(f"🎉 Threshold Saved: {threshold:.1f} px");
                break
            else:
                print("⚠️ Aage 1 ebong 2 chepe size set korun!")
        elif key == 27:
            break

    cap.release();
    cv2.destroyAllWindows()


if __name__ == "__main__": main()