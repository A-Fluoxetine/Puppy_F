import os

import time
import wave
import sys
import pyaudio
import numpy as np
import whisper
import pyttsx3
from fuzzywuzzy import fuzz  
MODEL = whisper.load_model("base")
FORMAT = pyaudio.paInt16
CHANNELS = 1
SAMPLE_RATE = 16000
CHUNK_SIZE = 1024
MAX_SILENCE_SECONDS = 5
GLOBAL_TIMEOUT = 10

def init_tts():
    engine = pyttsx3.init()
    engine.setProperty("rate", 150) 
    engine.setProperty("volume", 1.0)  
    return engine

def speak(engine, text):
    print(f"[\u8bed\u97f3\u63d0\u793a] {text}")
    engine.say(text)
    engine.runAndWait()

def sit_down(engine):
    speak(engine, "\u597d\u7684\uff0c\u6211\u5750\u4e0b\u4e86\u3002")

def come_here(engine):
    speak(engine, "\u6211\u9a6c\u4e0a\u8fc7\u6765\u3002")

def fetch_item(engine):
    speak(engine, "\u8bf7\u544a\u8bc9\u6211\u62ff\u4ec0\u4e48\u3002")

def move_forward(engine):
    speak(engine, "\u6b63\u5728\u524d\u8fdb\u3002")

def nod_head(engine):
    speak(engine, "\u55ef\u55ef")

def close_program(engine):
    sys.exit(0)

KEYWORD_ACTIONS = {
    "\u5750\u4e0b": sit_down,
    "\u8fc7\u6765": come_here,
    "\u62ff\u4e1c\u897f": fetch_item,
    "\u524d\u8fdb": move_forward,
    "\u70b9\u5934": nod_head,
    "\u6211\u6ca1\u6709\u522b\u7684\u8981\u6c42\u4e86": close_program,
}
class VoiceRecorder:
    def __init__(self, default_device_index=None):
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.frames = []
        self.recording = False
        self.last_activity = time.time()
        self.threshold = 1500  
        self.input_device = self.select_input_device(default_device_index)
        print(f"\u8bbe\u5907\u521d\u59cb\u5316\u65f6\u95f4: {time.ctime()}")
        print(f"\u5f53\u524d\u9608\u503c: {self.threshold}")
        self.calibrate_threshold()

    def select_input_device(self, default_device_index=None):
        valid_devices = []
        for i in range(self.p.get_device_count()):
            dev = self.p.get_device_info_by_index(i)
            if dev["maxInputChannels"] > 0:
                valid_devices.append(i)

        if default_device_index is not None and default_device_index in valid_devices:
             print(f"\u4f7f\u7528\u9ed8\u8ba4\u8bbe\u5907: {default_device_index}")
             return default_device_index

        print("\n\u53ef\u7528\u97f3\u9891\u8f93\u5165\u8bbe\u5907:")
        for i in valid_devices:
            dev = self.p.get_device_info_by_index(i)
            print(f"[{i}] {dev['name']}")

        while True:
            try:
                choice = int(input("\u8bf7\u9009\u62e9\u8f93\u5165\u8bbe\u5907\u7f16\u53f7: "))
                if choice in valid_devices:
                    return choice
            except:
                pass
            print("\u8f93\u5165\u65e0\u6548\uff0c\u8bf7\u91cd\u65b0\u9009\u62e9")

    def calibrate_threshold(self, duration=3):
        print("\n\u6b63\u5728\u6821\u51c6\u73af\u5883\u566a\u97f3...")
        temp_stream = self.p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=SAMPLE_RATE,
            input=True,
            input_device_index=self.input_device,
            frames_per_buffer=CHUNK_SIZE
        )

        frames = []
        for _ in range(int(SAMPLE_RATE / CHUNK_SIZE * duration)):
            try:
                data = temp_stream.read(CHUNK_SIZE)
                frames.append(np.frombuffer(data, dtype=np.int16))
            except:
                pass

        if frames:
            background = np.concatenate(frames)
            self.threshold = np.abs(background).mean() * 2.5
            print(f"\u81ea\u52a8\u8bbe\u7f6e\u9759\u97f3\u9608\u503c\u4e3a: {self.threshold:.1f}")
        temp_stream.stop_stream()
        temp_stream.close()

    def start_recording(self):
        if self.stream:
            self.stream.close()

        self.stream = self.p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=SAMPLE_RATE,
            input=True,
            input_device_index=self.input_device,
            frames_per_buffer=CHUNK_SIZE,
            stream_callback=self.callback
        )
        self.stream.start_stream()
        self.frames = []
        self.recording = True
        print("* \u5f55\u97f3\u5f00\u59cb")
        print(f"\u5f00\u59cb\u5f55\u97f3\u65f6\u95f4: {time.ctime()}")
        self.last_activity = time.time()


    def stop_recording(self):
        if self.stream:

            self.stream.stop_stream()

            self.stream.close()
        self.recording = False
        print("* \u5f55\u97f3\u7ed3\u675f")

    def callback(self, in_data, frame_count, time_info, status):
        self.frames.append(in_data)
        audio_data = np.frombuffer(in_data, dtype=np.int16)

        if np.abs(audio_data).mean() > self.threshold:

            self.last_activity = time.time()

        return (in_data, pyaudio.paContinue)



    def process_audio(self):

        """\u5904\u7406\u97f3\u9891\u6d41"""

        try:

            if not self.stream or not self.stream.is_active():

                self.start_recording()



            # \u5168\u5c40\u8d85\u65f6\u68c0\u6d4b

            if (time.time() - self.last_activity) > GLOBAL_TIMEOUT:

                print("\n\u60a8\u5df210\u79d2\u672a\u8bf4\u8bdd\uff0c\u7a0b\u5e8f\u5373\u5c06\u9000\u51fa...")

                self.stop_recording()

                sys.exit(1)



            # \u9759\u97f3\u8d85\u65f6\u68c0\u6d4b

            if self.recording and (time.time() - self.last_activity) > MAX_SILENCE_SECONDS:

                if len(self.frames) >= SAMPLE_RATE // CHUNK_SIZE * 1:

                    self.stop_recording()

                    return True

                else:

                    self.frames = []

                    self.recording = False

            return False



        except Exception as e:

            print(f"\u97f3\u9891\u5904\u7406\u9519\u8bef: {str(e)}")

            return False





def speech_to_text(filename, engine):

    try:

        with wave.open(filename, "rb") as wf:

            n_channels = wf.getnchannels()

            sample_width = wf.getsampwidth()

            frame_rate = wf.getframerate()

            n_frames = wf.getnframes()

            frames = wf.readframes(n_frames)

            if sample_width == 2:  

                audio_data = np.frombuffer(frames, dtype=np.int16)

            elif sample_width == 4:  

                audio_data = np.frombuffer(frames, dtype=np.int32)

            else:

                raise ValueError("\u4e0d\u652f\u6301\u7684\u97f3\u9891\u683c\u5f0f")

            if n_channels > 1:

                audio_data = audio_data.reshape(-1, n_channels)

                audio_data = np.mean(audio_data, axis=1)

            audio_data = audio_data.astype(np.float32) / 32768.0 



        # \u4f7f\u7528\u97f3\u9891\u6570\u636e\u8fdb\u884c\u8bc6\u522b

        result = MODEL.transcribe(

            audio_data, 

            language="zh",

            initial_prompt="\u4ee5\u4e0b\u662f\u666e\u901a\u8bdd\u8bed\u97f3\u8f6c\u5199\u3002",

            fp16=False

        )

        simplified_text = result["text"].strip() 

        print(f"\u8bc6\u522b\u7ed3\u679c: {simplified_text}")



        if not simplified_text or len(simplified_text.strip()) == 0:

            print("\u672a\u68c0\u6d4b\u5230\u6709\u6548\u8bed\u97f3")

            speak(engine, "\u672a\u8bc6\u522b\uff0c\u8bf7\u518d\u8bf4\u4e00\u904d\u3002")

            return None



        for keyword, action in KEYWORD_ACTIONS.items():

            similarity = fuzz.partial_ratio(keyword, simplified_text)

            if similarity >= 80:  

                print(f"\u68c0\u6d4b\u5230\u5173\u952e\u8bcd: {keyword}")

                action(engine)  # \u6267\u884c\u5bf9\u5e94\u7684\u7a0b\u5e8f\u8282\u70b9

                return keyword



        speak(engine, "\u672a\u8bc6\u522b\uff0c\u8bf7\u518d\u8bf4\u4e00\u904d\u3002")

        return None

    except Exception as e:

        print(f"\u8bc6\u522b\u5931\u8d25: {str(e)}")

        speak(engine, "\u672a\u8bc6\u522b\uff0c\u8bf7\u518d\u8bf4\u4e00\u904d\u3002")

        return None



def main():

    output_dir = os.path.join(os.path.expanduser("~"), "\u8bed\u97f3\u8bb0\u5f55")

    os.makedirs(output_dir, exist_ok=True)

    engine = init_tts()

    recorder = VoiceRecorder(default_device_index=0)



    try:

        speak(engine, "\u8bf7\u5f00\u59cb\u8bf4\u8bdd\u3002")

        while True:

            if recorder.process_audio():

                filename = os.path.join(

                    output_dir,

                    f"recording_{time.strftime('%Y%m%d-%H%M%S')}.wav"

                )



                # \u4fdd\u5b58\u97f3\u9891\u6587\u4ef6

                with wave.open(filename, "wb") as wf:

                    wf.setnchannels(CHANNELS)

                    wf.setsampwidth(recorder.p.get_sample_size(FORMAT))

                    wf.setframerate(SAMPLE_RATE)

                    wf.writeframes(b"".join(recorder.frames))

                print(f"\u6587\u4ef6\u5df2\u4fdd\u5b58\u81f3: {os.path.abspath(filename)}")



                # \u8bed\u97f3\u8bc6\u522b

                text = speech_to_text(filename, engine)

                if text:

                    print(f"\n\u8bc6\u522b\u7ed3\u679c\uff1a{text}")

                    # \u5355\u53e5\u5f55\u97f3\u7ed3\u675f\u540e\u63d0\u793a\u201c\u6c6a\u201d

                    speak(engine, "\u6c6a")

                else:

                    # \u5982\u679c\u8bc6\u522b\u7ed3\u679c\u65e0\u6548\uff0c\u63d0\u793a\u201c\u672a\u8bc6\u522b\uff0c\u8bf7\u518d\u8bf4\u4e00\u904d\u201d

                    speak(engine, "\u672a\u8bc6\u522b\uff0c\u8bf7\u518d\u8bf4\u4e00\u904d\u3002")



                # \u91cd\u7f6e\u5f55\u97f3\u5668

                recorder = VoiceRecorder(default_device_index=13)



    except KeyboardInterrupt:

        speak(engine, "\u7a0b\u5e8f\u5df2\u624b\u52a8\u7ec8\u6b62\u3002")

        print("\n\u7528\u6237\u4e3b\u52a8\u7ec8\u6b62")

    finally:

        recorder.stop_recording()

        recorder.p.terminate()

        speak(engine, "\u7a0b\u5e8f\u7ed3\u675f")  # \u6574\u4f53\u7a0b\u5e8f\u7ed3\u675f\u524d\u63d0\u793a





if __name__ == "__main__":

    main()