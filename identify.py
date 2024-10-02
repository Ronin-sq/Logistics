# 识别二维码
import cv2
import numpy as np
import constants
from PIL import Image, ImageDraw, ImageFont 
from skimage import data,draw,color,transform,feature
class VideoCapture:
    def __init__(self,index):
        self.frame = None
        self.video = cv2.VideoCapture(index)
        self.K = np.array(
            [[366.63195575538515, 0.0, 323.0034899844723], [0.0, 490.40727832499107, 213.25799648860593],
             [0.0, 0.0, 1.0]])
        self.D = np.array([[-0.14827706218149106], [0.10272564498472946], [-0.144721204897414], [0.06967408567430872]])
        self.width, self.height = int(self.video.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.K, self.D, (self.width, self.height), None)
        self.mapx2, self.mapy2 = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, None, self.p, (self.width, self.height), cv2.CV_32F)

        self.frame = None
        self.video = cv2.VideoCapture(index)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if not self.video.isOpened():
            print("无法打开摄像头")
            exit()
            
    def get_frame(self):
        ret, self.frame = self.video.read()

        self.frame = cv2.remap(self.frame, self.mapx2, self.mapy2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        if not ret:
            print("无法接收帧 (stream end?)")
        return self.frame
            
            
    def QR_code(self):
        qr_coder = cv2.QRCodeDetector()
        if 1:
        #try:
            frame = self.frame
            # qr检测并解码
            codeinfo, points, straight_qrcode = qr_coder.detectAndDecode(frame)


        # 绘制qr的检测结果
            if codeinfo !='' :
                print(points)
                # 打印解码结果
                print("qrcode :", codeinfo)
                return codeinfo
        #except:
            #print("检测二维码失败")



    
    def find_material(self, list):
        # for i,element in enumerate(list):
            frame = self.get_frame()
            src = frame.copy()
            kernel = (5, 5)
            hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)         # 转HSV通道
            res = src.copy()
            mask = cv2.inRange(hsv, constants.color_config[list][0], constants.color_config[list][1])
            #与函数，起到将两张黑色区域合并的效果
            res = cv2.bitwise_and(src, src, mask=mask)
            h,w = res.shape[:2]
            #滤波函数，滤除噪点：
            blured = cv2.blur(res,(5,5))
            ret, bright = cv2.threshold(blured,10,255,cv2.THRESH_BINARY)
            gray = cv2.cvtColor(bright,cv2.COLOR_BGR2GRAY)
            #开闭运算函数，滤除噪点，使得集合区域更加平滑
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel)
            opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)



            contours, hierarchy = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            contours=sorted(contours,key = cv2.contourArea, reverse=True)
            if contours:

                M = cv2.moments(contours[0])
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # draw the contour and center of the shape on the image
                cv2.drawContours(frame, [contours[0]], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 1, (255, 255, 255), -1)
                print("x",cX,'y',cY)




    def cv_imshow(self):
        cv2.imshow("frame", self.frame)


    def release(self):
        cv2.destroyAllWindows()
        self.video.release()


def get_text(list):
    texts =""
    config = {
        1:"1",
        2:"2",
        3:"3"
    }
    for i in list:
        text = config[list[i-1]]
        texts = texts+text+"  "
    return texts
    
    
def show_mission(list):
    img = cv2.imread("white.jpg")
    pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    # 创建一个可以在图像上绘图的对象
    draw = ImageDraw.Draw(pil_img)
    # 定义字体和字号  
    font = ImageFont.truetype("simkai.ttf", 400, encoding="unic")
    # 指定文本位置和颜色
    text = get_text(list)
    position = (0, 350)
    text_color = (0, 0, 0)
    # 在图像上绘制文本
    draw.text(position, text, fill=text_color, font=font)
    # 将含有文本的PIL格式的图像转换回OpenCV格式
    img_with_text = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    # 保存处理后的图像
    cv2.imwrite('image_with_text.jpg', img_with_text)
    cv2.imshow("Mission", img_with_text)
    while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 按 'q' 键关闭窗口
                break
    cv2.destroyAllWindows()
    
    
    
    
if __name__ == "__main__":
    # show_mission([1,2,3])
    video = VideoCapture(0)
    while True:

        video.find_material(2)
        video.cv_imshow()

        #video.cv_imshow()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    video.release()
    # cap = cv2.VideoCapture(0)
    # #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # if not cap.isOpened():
    #     print("无法打开摄像头")
    #     exit()
    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         print("无法接收帧 (stream end?)")
    #         break
    #     cv2.imshow('frame', frame)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    #
    # cap.release()
    # cv2.destroyAllWindows()
