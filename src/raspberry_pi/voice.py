import threading
import multiprocessing
import pvporcupine
import pvrhino
from pvrecorder import PvRecorder

class voice_thread(multiprocessing.Process):
    def __init__(self, command, access_key, wake_model_path, intent_model_path, device_idx=2, wake_sens=0.7, intent_sens=0.7):
        super().__init__(self)
        self.porcupine = pvporcupine.create(keyword_paths=[wake_model_path],
                                            access_key=access_key,
                                            sensitivities=[wake_sens])
        self.rhino = pvrhino.create(context_path = intent_model_path,
                                    access_key=access_key,
                                    sensitivity=intent_sens)
        self.recorder = PvRecorder(device_index=device_idx, frame_length=self.porcupine.frame_length)
        self.recorder.start()

    def run(self):
        while True:
            pcm = self.recorder.read()
            awoken = False

            if not awoken:
                if self.porcupine.process(pcm) >= 0:
                    print("Wake word detected!")
                    awoken = True
            else:
                if self.rhino.process(pcm):
                    command = self.rhino.get_inference()
                    print(command)
                    awoken = False