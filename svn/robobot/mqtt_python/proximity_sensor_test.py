#!/usr/bin/env python3

import time as t
import os
from uservice import service
from sservo import servo

# --- CONFIGURATION ---
TELEMETRY_TOPIC ="ricbot/teensy/nm"
# Definiamo il percorso completo (assoluto)
LOG_DIR = "/svn/robobot/mqtt_python/"
LOG_FILE = os.path.join(LOG_DIR, "proximity_calibration.txt")
LED_TOPIC = "robobot/cmd/T0" 
SAMPLES_REQUIRED = 5

def proximity_sensor_test():
    """
    Raccoglie 5 letture e le salva in una cartella specifica.
    """
    # Verifichiamo se la cartella esiste, altrimenti la creiamo
    if not os.path.exists(LOG_DIR):
        try:
            os.makedirs(LOG_DIR)
            print(f"% Cartella creata: {LOG_DIR}")
        except Exception as e:
            print(f"% Errore nel creare la cartella: {e}")
            return

    print(f"% Inizio test. Il file verrà salvato in: {LOG_FILE}")
    
    servo.servo_change_position(-900)
    t.sleep(1.0)

    front_values = []
    left_values = []

    while len(front_values) < SAMPLES_REQUIRED and not service.stop:
        data = service.get_latest(TELEMETRY_TOPIC)
        
        if data and 'obs' in data:
            distances = data['obs']
            
            if len(distances) >= 2:
                f_val = distances[0]
                l_val = distances[1]
                
                front_values.append(f_val)
                left_values.append(l_val)
                
                print(f"% Campione {len(front_values)}/5 registrato: F={f_val:.3f}m, L={l_val:.3f}m")
                
                # Feedback LED Blu
                service.send(LED_TOPIC, "leds 16 0 0 50")
                t.sleep(0.2) 
                service.send(LED_TOPIC, "leds 16 0 0 0")
                t.sleep(0.8) 
            else:
                print("% Errore: l'array 'obs' non ha abbastanza sensori.")
                t.sleep(1.0)
        else:
            print(f"% In attesa di telemetria su {TELEMETRY_TOPIC}...")
            t.sleep(0.5)

    # --- SCRITTURA NEL FILE ---
    if len(front_values) == SAMPLES_REQUIRED:
        try:
            with open(LOG_FILE, "w") as f:
                f.write("VALORI SENSORE FRONTALE (metri)\n")
                for val in front_values:
                    f.write(f"{val:.4f}\n")
                    
                f.write("\nVALORI SENSORE LATERALE (metri)\n")
                for val in left_values:
                    f.write(f"{val:.4f}\n")

            print("\n" + "="*40)
            print(f" SUCCESSO: File salvato in:\n {LOG_FILE}")
            print("="*40)
            
            # Segnale Verde finale
            service.send(LED_TOPIC, "leds 16 0 50 0")
            t.sleep(1.0)
            service.send(LED_TOPIC, "leds 16 0 0 0")
        except Exception as e:
            print(f"% Errore durante la scrittura del file: {e}")

if __name__ == "__main__":
    service.setup('localhost')
    if service.connected:
        try:
            proximity_sensor_test()
        except KeyboardInterrupt:
            print("\n% Abortito dall'utente.")
            service.send(LED_TOPIC, "leds 16 0 0 0")
    else:
        print("% Errore: Broker MQTT non trovato.")
    service.terminate()