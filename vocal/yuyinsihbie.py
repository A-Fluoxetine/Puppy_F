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
    print(f"[语音提示] {text}")
    engine.say(text)
    engine.runAndWait()

def sit_down(engine):
    speak(engine, "好的，我坐下了。")

def come_here(engine):
    speak(engine, "我马上过来。")

def fetch_item(engine):
    speak(engine, "请告诉我拿什么。")

def move_forward(engine):
    speak(engine, "正在前进。")

def nod_head(engine):
    speak(engine, "嗯嗯")

def close_program(engine):
    sys.exit(0)

KEYWORD_ACTIONS = {
    "坐下": sit_down,
    "过来": come_here,
    "拿东西": fetch_item,
    "前进": move_forward,
    "点头": nod_head,
    "我没有别的要求了": close_program,
}

class VoiceRecorder:
    def __init__(self, default_device_index=None):
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.frames = []
        self.recording = False
        self.last_activity = time.time()
        self.threshold = 1500  # 初始阈值
        self.input_device = self.select_input_device(default_device_index)
        print(f"设备初始化时间: {time.ctime()}")
        print(f"当前阈值: {self.threshold}")
        self.calibrate_threshold()

    def select_input_device(self, default_device_index=None):
        valid_devices = []
        for i in range(self.p.get_device_count()):
            dev = self.p.get_device_info_by_index(i)
            if dev["maxInputChannels"] > 0:
                valid_devices.append(i)

        if default_device_index is not None and default_device_index in valid_devices:
            print(f"使用默认设备: {default_device_index}")
            return default_device_index

        print("\n可用音频输入设备:")
        for i in valid_devices:
            dev = self.p.get_device_info_by_index(i)
            print(f"[{i}] {dev['name']}")

        while True:
            try:
                choice = int(input("请选择输入设备编号: "))
                if choice in valid_devices:
                    return choice
            except:
                pass
            print("输入无效，请重新选择")

    def calibrate_threshold(self, duration=3):
        """自动校准静音阈值"""
        print("\n正在校准环境噪音...")
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
            print(f"自动设置静音阈值为: {self.threshold:.1f}")

        temp_stream.stop_stream()
        temp_stream.close()

    def start_recording(self):
        """启动录音流"""
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
        print("* 录音开始")
        print(f"开始录音时间: {time.ctime()}")
        self.last_activity = time.time()

    def stop_recording(self):
        """停止录音"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.recording = False
        print("* 录音结束")

    def callback(self, in_data, frame_count, time_info, status):
        """音频回调函数"""
        self.frames.append(in_data)
        audio_data = np.frombuffer(in_data, dtype=np.int16)
        if np.abs(audio_data).mean() > self.threshold:
            self.last_activity = time.time()
        return (in_data, pyaudio.paContinue)

    def process_audio(self):
        """处理音频流"""
        try:
            if not self.stream or not self.stream.is_active():
                self.start_recording()

            # 全局超时检测
            if (time.time() - self.last_activity) > GLOBAL_TIMEOUT:
                print("\n您已10秒未说话，程序即将退出...")
                self.stop_recording()
                sys.exit(1)

            # 静音超时检测
            if self.recording and (time.time() - self.last_activity) > MAX_SILENCE_SECONDS:
                if len(self.frames) >= SAMPLE_RATE // CHUNK_SIZE * 1:
                    self.stop_recording()
                    return True
                else:
                    self.frames = []
                    self.recording = False
            return False

        except Exception as e:
            print(f"音频处理错误: {str(e)}")
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
                raise ValueError("不支持的音频格式")
            if n_channels > 1:
                audio_data = audio_data.reshape(-1, n_channels)
                audio_data = np.mean(audio_data, axis=1)
            audio_data = audio_data.astype(np.float32) / 32768.0

        # 使用音频数据进行识别
        result = MODEL.transcribe(
            audio_data,
            language="zh",
            initial_prompt="以下是普通话语音转写。",
            fp16=False
        )
        simplified_text = result["text"].strip()
        print(f"识别结果: {simplified_text}")

        if not simplified_text or len(simplified_text.strip()) == 0:
            print("未检测到有效语音")
            speak(engine, "未识别，请再说一遍。")
            return None

        for keyword, action in KEYWORD_ACTIONS.items():
            similarity = fuzz.partial_ratio(keyword, simplified_text)
            if similarity >= 80:
                print(f"检测到关键词: {keyword}")
                action(engine)  # 执行对应的程序节点
                return keyword

        speak(engine, "未识别，请再说一遍。")
        return None
    except Exception as e:
        print(f"识别失败: {str(e)}")
        speak(engine, "未识别，请再说一遍。")
        return None

def main():
    output_dir = os.path.join(os.path.expanduser("~"), "语音记录")
    os.makedirs(output_dir, exist_ok=True)
    engine = init_tts()
    recorder = VoiceRecorder(default_device_index=13)
#此处想在电脑上用的话记得更改一下默认录音设备。一般改为0就行，这里的13是指树莓派上的录音代号S

    try:
        speak(engine, "请开始说话。")
        while True:
            if recorder.process_audio():
                filename = os.path.join(
                    output_dir,
                    f"recording_{time.strftime('%Y%m%d-%H%M%S')}.wav"
                )

                # 保存音频文件
                with wave.open(filename, "wb") as wf:
                    wf.setnchannels(CHANNELS)
                    wf.setsampwidth(recorder.p.get_sample_size(FORMAT))
                    wf.setframerate(SAMPLE_RATE)
                    wf.writeframes(b"".join(recorder.frames))
                print(f"文件已保存至: {os.path.abspath(filename)}")

                # 语音识别
                text = speech_to_text(filename, engine)
                if text:
                    print(f"\n识别结果：{text}")
                    # 单句录音结束后提示“汪”
                    speak(engine, "汪")
                else:
                    # 如果识别结果无效，提示“未识别，请再说一遍”
                    speak(engine, "未识别，请再说一遍。")

                # 重置录音器
                recorder = VoiceRecorder(default_device_index=0)

    except KeyboardInterrupt:
        speak(engine, "程序已手动终止。")
        print("\n用户主动终止")
    finally:
        recorder.stop_recording()
        recorder.p.terminate()
        speak(engine, "程序结束")  # 整体程序结束前提示


if __name__ == "__main__":
    main()
