import cv2 as cv
import numpy as np
import time as t
from scam import cam
from sservo import servo
from uservice import service

def robot_jiggle():
    print("% >>> JIGGLE! Target Identified!")
    # Avanti veloce per 0.1 secondi
    service.send("robobot/cmd/ti", "rc 0.20 0.00")
    t.sleep(0.1)
    # Indietro veloce per 0.1 secondi
    service.send("robobot/cmd/ti", "rc -0.20 0.00")
    t.sleep(0.1)
    # Stop
    service.send("robobot/cmd/ti", "rc 0.00 0.00")

def loop():
    # Parametri di filtraggio validati dai tuoi test
    MIN_AREA = 2500
    CIRC_MIN = 0.60
    CIRC_MAX = 0.95
    ASPECT_MIN = 0.5
    ASPECT_MAX = 2.0

    target_found = False

    print("% Searching for hole... Place the robot in front of it.")

    while not service.stop and not target_found:
        ok, img, imgTime = cam.getImage()
        if not ok:
            continue

        h, w = img.shape[:2]
        # Taglia il primo 40% dell'altezza
        roi = img[int(h*0.40):, :]
        
        # Pre-processing potenziato
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (11, 11), 0)
        edged = cv.Canny(blurred, 30, 50)
        
        kernel_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, (30, 30))
        edged = cv.morphologyEx(edged, cv.MORPH_CLOSE, kernel_close)
        edged = cv.dilate(edged, np.ones((7, 7), np.uint8), iterations=3)
        
        contours, _ = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < MIN_AREA: continue
            
            peri = cv.arcLength(cnt, True)
            if peri == 0: continue
            circularity = 4 * np.pi * (area / (peri * peri))
            
            x, y, w_box, h_box = cv.boundingRect(cnt)
            aspect_ratio = float(w_box) / h_box

            # Logica di riconoscimento
            if CIRC_MIN < circularity < CIRC_MAX and ASPECT_MIN < aspect_ratio < ASPECT_MAX:
                # ABBIAMO TROVATO IL BUCO!
                # 1. Feedback visivo
                cv.rectangle(roi, (x, y), (x + w_box, y + h_box), (0, 255, 0), 3)
                cv.imshow('Test Hole', roi)
                cv.waitKey(1)
                
                # 2. Feedback fisico (Sussulto)
                robot_jiggle()
                
                # 3. Segnale visivo con LED
                service.send("robobot/cmd/T0", "leds 16 0 30 0") 
                
                target_found = True
                break

        cv.imshow('Test Hole', roi)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    print("% Test completed.")
    service.set_motors(0, 0)
    service.terminate()

if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        try:
            loop()
        except KeyboardInterrupt:
            pass
    
    # Cleanup
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    service.terminate()