from ultralytics import YOLO
import cv2
import numpy as np
import math

# --------------------------配置区--------------------------
MODEL_PATH = "./runs/detect/train-5/weights/best.pt"   # 你的训练好模型
SOURCE = "leidafindthetarget.mp4"      # 图片/视频/摄像头0
#SOURCE = 0
CONF_THRESH = 0.2        # 置信度过滤，低于该值不参与连线
OUT_AVI = "output.avi"
# -----------------------------------------------------------

def get_box_xyxy_center(xyxy: np.ndarray):
    """输入xyxy [x1,y1,x2,y2]，返回中心点整数坐标(cx, cy)"""
    x1, y1, x2, y2 = xyxy
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)
    return (cx, cy)

def calc_point_distance(p1, p2):
    """计算两点之间欧式像素距离"""
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    dist = math.sqrt(dx**2 + dy**2)
    return round(dist, 2)

# 加载模型
model = YOLO(MODEL_PATH)

# 初始化avi保存器
cap = cv2.VideoCapture(SOURCE)
fps = cap.get(cv2.CAP_PROP_FPS)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap.release()
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(OUT_AVI, fourcc, fps, (w, h))

# 推理，stream=True视频逐帧，图片可去掉
results = model.predict(
    source=SOURCE,
    save=False,       # 不保存原生预测图
    conf=CONF_THRESH,
    stream=True
)

frame_count = 0  # 帧计数器，区分每一帧打印
for res in results:
    frame_count += 1
    img = res.orig_img.copy()  # 取原始画面绘制连线
    warn_center = None
    target_info = []  # 存(类别名, 中心点坐标)
    
    # 遍历所有检测框
    for box in res.boxes:
        cls_id = int(box.cls)
        cls_name = res.names[cls_id]
        box_xyxy = box.xyxy[0].cpu().numpy()
        center = get_box_xyxy_center(box_xyxy)

        if cls_name == "WARN":
            warn_center = center
        elif cls_name in ["left", "right", "back","forward"]:
            target_info.append((cls_name, center))
    
    print(f"\n========== 第{frame_count}帧 连线距离 ==========")
    # 同时存在WARN和点位，开始绘制连线并计算长度
    if warn_center is not None and len(target_info) > 0:
        # 绘制WARN中心点 蓝色实心圆
        cv2.circle(img, warn_center, 6, (255, 0, 0), -1)
        
        # 循环连线、算距离、打印、绘制文字
        for cls_name, pt in target_info:
            dist = calc_point_distance(warn_center, pt)
            # 控制台输出每条线长度
            print(f"WARN <---> {cls_name} 像素距离：{dist}")
            
            # 红色连线，线宽2
            cv2.line(img, warn_center, pt, (0, 0, 255), thickness=2)
            # 点位绿色实心小圆
            cv2.circle(img, pt, 5, (0, 255, 0), -1)
            
            # 在连线中点绘制距离文字
            mid_x = int((warn_center[0] + pt[0]) / 2)
            mid_y = int((warn_center[1] + pt[1]) / 2)
            cv2.putText(img, f"{dist}", (mid_x, mid_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
    
    # 保存带连线的效果图（会覆盖，如需保存每一帧改名）
    cv2.imwrite("draw_line_result.jpg", img)
    # 写入avi视频帧
    out.write(img)
    # 实时画面显示（视频/摄像头用）
    cv2.imshow("result", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放视频写入
out.release()
cv2.destroyAllWindows()
print(f"视频已保存为 {OUT_AVI}")