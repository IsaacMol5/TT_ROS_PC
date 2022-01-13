#! /usr/bin/env python3

import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from pathlib import Path

import numpy as np
import cv2
import torch
import torch.backends.cudnn as cudnn

from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import apply_classifier, check_img_size, check_imshow, check_requirements, check_suffix, colorstr, \
    increment_path, non_max_suppression, print_args, save_one_box, scale_coords, set_logging, \
    strip_optimizer, xyxy2xywh
from utils.plots import Annotator, colors
from utils.torch_utils import load_classifier, select_device, time_sync

limit = 0
ros_image = 0
enabled_node = False

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, 32), np.mod(dh, 32)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)
    
def loadimg(img):
    img_size=640
    cap=None
    path=None
    img0 = img
    img = letterbox(img0, new_shape=img_size)[0]
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)
    return path, img, img0, cap

def detect(img):
    dataset = loadimg(img)

    view_img = True
    conf_thres = 0.3
    iou_thres = 0.45
    line_thickness=3
    cudnn.benchmark = True
    pt = True 
    augment=False
    agnostic_nms = False
    hide_labels=False
    hide_conf=False
    max_det=1000
    classes = None
    stride = int(model.stride.max())  # model stride
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    if half:
        model.half()  # to FP16
    #imgsz = check_img_size(imgsz, s=stride)  # check image size
    path = dataset[0]
    img = dataset[1]
    im0s = dataset[2]
    vid_cap = dataset[3]

    # Run inference
    if pt and device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once
    dt, seen = [0.0, 0.0, 0.0], 0
    
    t1 = time_sync()
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]  # expand for batch dim
    t2 = time_sync()
    dt[0] += t2 - t1


    pred = model(img, augment=augment, visualize=False)[0]

    t3 = time_sync()
    dt[1] += t3 - t2

    # NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
    dt[2] += time_sync() - t3

    signals_detected = []
    n1 = 0
    # Process predictions
    for i, det in enumerate(pred):  # per image
        seen += 1
        p, s, im0 = path, '', im0s
        s += '%gx%g ' % img.shape[2:]  # print string
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        #imc = im0.copy() if save_crop else im0  # for save_crop
        annotator = Annotator(im0, line_width=line_thickness, example=str(names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                signals_detected.append(names[int(c)])
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det):

                if view_img:  # Add bbox to image
                    c = int(cls)  # integer class
                    label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    posicion_rectangule = xyxy
        # Print time (inference-only)
        print(f'{s}Done. ({t3 - t2:.3f}s)')
        signals_detected = set(signals_detected)
        publish_classes(signals_detected)
        # Stream results
        im0 = annotator.result()
        #cv2.imshow("YoloV5_ROS", im0)
        #cv2.waitKey(1)  # 1 millisecond
        publish_image(im0)
    #height, width = im0.shape[:2]
    #print((height, width))
    #print("Classes: "+ str(signals_detected))
    #print(posicion_rectangule.index(0))
    # Print results
    #t = tuple(x / seen * 1E3 for x in dt)  # speeds per image

    #print(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    
def publish_image(im0):
    image_temp = CompressedImage()
    image_temp.header.stamp = rospy.Time.now()
    image_temp.format = "jpeg"
    image_temp.data = np.array(cv2.imencode('.jpg', im0)[1]).tostring()
    image_pub.publish(image_temp)

def publish_classes(classes):
    classes = "-".join(classes)
    classes_pub.publish(classes)

def image_callback(image):
    global ros_image, limit, enabled_node
    #if(limit > 1 and enabled_node):
    if(enabled_node):
        #ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        np_arr = np.fromstring(image.data, np.uint8)
        ros_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #ros_image = cv2.cvtColor(ros_image, cv2.COLOR_BGR2RGB)
        #ros_image = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
        #ros_image = cv2.resize(ros_image, dsize=(640, 480))
        # Print results
        print("Hace algo")
        with torch.no_grad():
            limit = 0
            detect(ros_image)
    else:
        limit = limit + 1
def callback_enabled_node(data):
    global enabled_node
    enabled_node = data.data

if __name__ == '__main__':
    set_logging()
    device = ''
    device = select_device(device)
    half = device.type != 'cpu'  # half precision only supported on CUDA
    weights = 'best.pt'
    imgsz = 640
    model = attempt_load(weights, map_location=device)  # load FP32 model
    #model = load_model(device)
    imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
  
    rospy.init_node('ros_yolo_sf')
    image_topic= "/usb_cam/image_raw"
    image_topic_1 = "/fisheye_correction/image/compressed"
    rospy.Subscriber(image_topic_1, CompressedImage, image_callback, queue_size=1, buff_size=52428800)
    rospy.Subscriber('/gui/activation', Bool, callback_enabled_node)
    image_pub = rospy.Publisher('/ros_yolo_sf/image', CompressedImage, queue_size=1)
    classes_pub = rospy.Publisher('/ros_yolo_sf/traffic_signals', String, queue_size=1)
    #rospy.init_node("yolo_result_out_node", anonymous=True)
    

    rospy.spin()