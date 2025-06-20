import cv2
import numpy as np

class Test:

    def __init__(self):
        self.brightness = 0
        # HSV 초기값 설정
        self.low_h = 0
        self.low_s = 0
        self.low_v = 200
        self.high_h = 180
        self.high_s = 50
        self.high_v = 255

        cv2.namedWindow("White Mask")
        cv2.createTrackbar("Low H", "White Mask", self.low_h, 180, lambda x: None)
        cv2.createTrackbar("High H", "White Mask", self.high_h, 180, lambda x: None)
        cv2.createTrackbar("Low S", "White Mask", self.low_s, 255, lambda x: None)
        cv2.createTrackbar("High S", "White Mask", self.high_s, 255, lambda x: None)
        cv2.createTrackbar("Low V", "White Mask", self.low_v, 255, lambda x: None)
        cv2.createTrackbar("High V", "White Mask", self.high_v, 255, lambda x: None)

    def run(self):
        cap = cv2.VideoCapture(2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            print("❌ 카메라를 열 수 없습니다.")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            #frame = self.clahe(frame)
  
            print(f"Brightness: {self.brightness:.2f}")
            self.update_trackbar_values()
            frame = self.equalize_sv_channel_for_masking(frame)
            self.brightness = self.get_brightness(frame)
            self.show_white_mask(frame)

            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    def update_trackbar_values(self):
        self.low_h = cv2.getTrackbarPos("Low H", "White Mask")
        self.high_h = cv2.getTrackbarPos("High H", "White Mask")
        self.low_s = cv2.getTrackbarPos("Low S", "White Mask")
        self.high_s = cv2.getTrackbarPos("High S", "White Mask")
        self.low_v = cv2.getTrackbarPos("Low V", "White Mask")
        self.high_v = cv2.getTrackbarPos("High V", "White Mask")

    def equalize_sv_channel_for_masking(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v_eq = cv2.equalizeHist(v)
        s_eq = cv2.equalizeHist(s)
        v_eq = cv2.GaussianBlur(v_eq, (3, 3), 0)
        
        hsv_eq = cv2.merge((h, s, v_eq))
        return hsv_eq 
    
    def get_brightness(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return np.mean(gray)

    def show_white_mask(self, image):
        hsv = image
        lower = np.array([self.low_h, self.low_s, self.low_v])
        upper = np.array([self.high_h, self.high_s, self.high_v])
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
        cv2.imshow("White Mask", mask)
# 145 <
# 0 180 0 8 240 255   yellow = 23 90 85 255 180 255
# 140 ~ 145
# 0 180 0 11 238 255  23 90 85 255 180 255
# b 130 ~ 140 
#  0 180 0 11 230 255    25 90 160 230 190 255
# b 120 ~ 130
# 0 180 0 11 230 255   25 90 160 230 190 255
# b 110 ~ 120 
# 0 180 0 11 245 255   25 90  130 230 195 255


# 145 
# 0 180 0 10 240 255 yel = 15 35 0 255 235 255
# 140 ~ 145
# 0  180 0 20 240 255 yel = 21 75 30 250 210 255
# 130 ~ 140
# 0 120 0 51 230 255       20 49 25 255  180 255
# 120 ~ 130
# 0 120 0 60 220 255   20 50 25 255 150 255
# 110 ~ 120 
# 0 180 30 70 210 255 yel = 20 50 25 255 150 255
# 105 ~ 110
# 30 180 5 75 150 255 yel = 20 50 25 255 70 255
# 105
# 50 180 5 60 120 255 


#120~130
# yel = 10 65 40 255 80 255
#110~120
# 0 180 0 171 200 255  10 65 40 255 80 255

def main():
    t = Test()
    t.run()

if __name__ == "__main__":
    main()
