

if __name__ == "__main__":
    print("% mission-run: initializing")
    indicator_stop = threading.Event()
    indicator_thread = None
    service.setup("localhost")
    try:
        if not service.connected:
            print("% mission-run: MQTT not connected")
        elif service.stop or robot.hbtUpdCnt == 0:
            print("% mission-run: startup failed (no robot heartbeat)")
            print("% mission-run: start teensy_interface and try again")
        else:
            indicator_thread = threading.Thread(
                target=flat_indicator_task,
                args=(indicator_stop,),
                daemon=True,
            )
            indicator_thread.start()
            calibrate_before_run()
            loop()
    finally:
        indicator_stop.set()
        if indicator_thread is not None:
            indicator_thread.join(timeout=1.0)
        set_line_leds(0, 0, 0)
        service.terminate()
    print("% mission-run: terminated")