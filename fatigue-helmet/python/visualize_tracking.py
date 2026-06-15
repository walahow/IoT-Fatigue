import argparse
import os
import sys
import cv2
import pandas as pd
import numpy as np

def draw_graph(img, y_pos, height, values, threshold=0.22):
    # Draw a black background for the graph
    cv2.rectangle(img, (0, y_pos), (img.shape[1], y_pos + height), (0,0,0), -1)
    
    # Draw threshold line
    th_y = int(y_pos + height - (threshold * height))
    cv2.line(img, (0, th_y), (img.shape[1], th_y), (0,0,255), 1)
    
    if len(values) < 2:
        return
        
    # Scale x to fit width
    dx = img.shape[1] / float(max(1, len(values)-1))
    
    pts = []
    for i, v in enumerate(values):
        x = int(i * dx)
        # ear usually 0 to 1.0
        v_clamped = max(0, min(1.0, v))
        y = int(y_pos + height - (v_clamped * height))
        pts.append((x, y))
        
    for i in range(1, len(pts)):
        cv2.line(img, pts[i-1], pts[i], (0, 255, 100), 2)

def main():
    parser = argparse.ArgumentParser(description="Visualize eye tracking from CSV on video.")
    parser.add_argument("--session", required=True, help="Path to session directory")
    parser.add_argument("--rotate-180", action="store_true", help="Rotate image 180 degrees")
    parser.add_argument("--no-enhance", action="store_true", help="Skip CLAHE contrast enhancement")
    args = parser.parse_args()

    session_path = os.path.abspath(args.session)
    csv_path = os.path.join(session_path, "camera_features.csv")
    frames_dir = os.path.join(session_path, "frames")
    out_path = os.path.join(session_path, "tracking_video_detailed.mp4")

    if not os.path.exists(csv_path):
        print(f"[ERROR] No camera_features.csv found in {session_path}")
        return

    df = pd.read_csv(csv_path)

    if len(df) == 0: return

    first_row = df.iloc[0]
    first_frame_path = os.path.join(frames_dir, f"{int(first_row['timestamp_ms'])}.jpg")
    first_img = cv2.imread(first_frame_path)
    if first_img is None: return
        
    h, w = first_img.shape[:2]
    # Add space for the graph at the bottom
    graph_h = 100
    out_h = h + graph_h
    timestamps = sorted(df['timestamp_ms'].tolist())
    duration_sec = (timestamps[-1] - timestamps[0]) / 1000.0
    fps = len(timestamps) / duration_sec if duration_sec > 0 else 30.0
    print(f"[INFO] Detected framerate: {fps:.2f} FPS")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, (w, out_h))

    ear_history = []
    max_history = 150 # 5 seconds at 30 fps
    
    last_bbox = None

    print(f"[INFO] Generating {out_path}...")
    
    for idx, row in df.iterrows():
        ts = int(row['timestamp_ms'])
        frame_path = os.path.join(frames_dir, f"{ts}.jpg")
        img = cv2.imread(frame_path)
        if img is None: continue
            
        eye_detected = int(row.get('eye_detected', 0))
        ear = float(row.get('ear', 0.0))
        is_blink = int(row.get('is_blink_frame', 0))
        
        ear_history.append(ear)
        if len(ear_history) > max_history:
            ear_history.pop(0)
        
        if args.rotate_180:
            img = cv2.rotate(img, cv2.ROTATE_180)
            
        if not args.no_enhance:
            clahe_bgr = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l2 = clahe_bgr.apply(l)
            lab = cv2.merge((l2, a, b))
            img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
            
        # Create full output canvas
        canvas = np.zeros((out_h, w, 3), dtype=np.uint8)
        canvas[0:h, 0:w] = img
        
        if eye_detected == 1:
            color = (0, 0, 255) if is_blink else (0, 255, 100)
            
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # The pupil scanner requires the CLAHE gray version for robust Otsu
            clahe_gray = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
            gray_for_pupil = clahe_gray.apply(gray) if args.no_enhance else gray # already enhanced above if not skipped
            
            if pd.notna(row.get('eye_x')) and row['eye_x'] != "":
                ex = int(row['eye_x'])
                ey = int(row['eye_y'])
                ew = int(row['eye_w'])
                eh = int(row['eye_h'])
                
                # 1. Draw green box
                cv2.rectangle(canvas, (ex, ey), (ex + ew, ey + eh), color, 2)
                cv2.putText(canvas, f"EAR: {ear:.3f}", (ex, ey - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # 2. Extract exactly how the algorithm sees the pupil
                margin = max(4, ew // 8)
                x1 = max(0, ex - margin)
                y1 = max(0, ey - margin)
                x2 = min(w, ex + ew + margin)
                y2 = min(h, ey + eh + margin)
                crop = gray_for_pupil[y1:y2, x1:x2]
                
                if crop.size > 0:
                    crop_rs = cv2.resize(crop, (128, 128), interpolation=cv2.INTER_LINEAR)
                    _, thresh = cv2.threshold(crop_rs, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
                    
                    # Draw contours on the pupil view
                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    pupil_color = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                    
                    if contours:
                        cnts = sorted(contours, key=cv2.contourArea, reverse=True)
                        for cnt in cnts:
                            if len(cnt) >= 5 and cv2.contourArea(cnt) >= 30:
                                cv2.ellipse(pupil_color, cv2.fitEllipse(cnt), (0, 0, 255), 2)
                                break
                    
                    # Overlay the pupil vision in the top right corner
                    canvas[10:138, w-138:w-10] = pupil_color
                    cv2.putText(canvas, "Pupil Scanner", (w-138, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

            if is_blink:
                cv2.putText(canvas, "BLINK", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                
        # Draw the graph at the bottom
        draw_graph(canvas, h, graph_h, ear_history)
        
        writer.write(canvas)
        
        if (idx + 1) % 500 == 0:
            print(f"Processed {idx + 1}/{len(df)} frames...")

    writer.release()
    print(f"[DONE] Detailed tracking video saved to {out_path}")

if __name__ == "__main__":
    main()
