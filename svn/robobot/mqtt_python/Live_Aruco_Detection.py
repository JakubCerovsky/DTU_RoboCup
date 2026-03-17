import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service
from sgpio import gpio

def stop_requested():
    if gpio.test_stop_button():
        service.stop = True
        print("% mission-run: stop button pressed")
        return True
    return service.stop

import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service

def robot_jiggle():
    """Fa sussultare il robot per confermare il riconoscimento del marker"""
    print("% >>> JIGGLE! Marker Identified!")
    # Avanti veloce per 0.1 secondi
    service.send("robobot/cmd/ti", "rc 0.20 0.00")
    t.sleep(0.1)
    # Indietro veloce per 0.1 secondi
    service.send("robobot/cmd/ti", "rc -0.20 0.00")
    t.sleep(0.1)
    # Stop definitivo
    service.send("robobot/cmd/ti", "rc 0.00 0.00")

def loop():
    # --- 1. INIZIALIZZAZIONE ---
    print("% Connecting to robot service...")
    service.setup('localhost')
    
    if not service.connected:
        print("% Error: MQTT broker not found.")
        return

    # Configurazione ArUco
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
    params = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(aruco_dict, params)

    # --- PARAMETRI TEST ---
    TARGET_ID = 20         # L'ID che deve attivare il sussulto

    print(f"% TEST MODE: Waiting for ID {TARGET_ID} (No Rotation).")
    
    # Assicuriamoci che il robot sia fermo all'inizio
    service.send("robobot/cmd/ti", "rc 0.00 0.00")

    while not service.stop:
        if stop_requested():
            break
        
        ok, img, imgTime = cam.getImage()
        
        if ok:
            # Rilevamento marker
            corners, ids, _ = detector.detectMarkers(img)
            
            if ids is not None and TARGET_ID in ids:
                # 1. Feedback visivo (LED Verde)
                service.send("robobot/cmd/T0", "leds 16 0 30 0") 
                
                # 2. Esecuzione del sussulto fisico
                robot_jiggle()
                
                # 3. Termina il loop immediatamente
                print(f"% Target {TARGET_ID} found! Test completed.")
                break 
            else:
                # Robot fermo in attesa del marker
                service.send("robobot/cmd/ti", "rc 0.00 0.00")
                service.send("robobot/cmd/T0", "leds 16 30 0 0") # Rosso: In attesa

            # Visualizzazione a schermo
            cv.imshow('Robot Vision - Static Jiggle Test', img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            t.sleep(0.01)

    # Shutdown finale
    service.send("robobot/cmd/ti", "rc 0.00 0.00")
    cv.destroyAllWindows()
    service.terminate()

if __name__ == "__main__":
    try:
        loop()
    except KeyboardInterrupt:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")
        service.terminate()