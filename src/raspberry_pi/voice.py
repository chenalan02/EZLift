import threading
import pvporcupine
import pvrhino
from pvrecorder import PvRecorder

class VoiceThread(threading.Thread):
    def __init__(self, command_queue, access_key, wake_model_path, intent_model_path, wake_sens=0.9, intent_sens=0.9):
        super().__init__()
        self.porcupine = pvporcupine.create(keyword_paths=[wake_model_path],
                                            access_key=access_key,
                                            sensitivities=[wake_sens])
        self.rhino = pvrhino.create(context_path = intent_model_path,
                                    access_key=access_key,
                                    sensitivity=intent_sens)
        device_idx = PvRecorder.get_available_devices().index('Built-in Audio Stereo')
        self.recorder = PvRecorder(device_index=device_idx, frame_length=self.porcupine.frame_length)
        self.recorder.start()
        self.command_queue = command_queue
        print("voice_init")

    def run(self):
        
        awoken = False
        while True:
            pcm = self.recorder.read()

            if not awoken:
                if self.porcupine.process(pcm) >= 0:
                    print("Wake word detected!")
                    awoken = True
            else:
                if self.rhino.process(pcm):
                    inference = self.rhino.get_inference()
                    self.command_queue.put(self.rhino.get_inference())
                    print(inference)
                    awoken = False