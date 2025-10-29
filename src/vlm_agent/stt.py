# stt.py

import os
import sys
import json
import queue
import sounddevice as sd
import vosk
from time import sleep, time

# --- 配置 ---
MODEL_PATH = "vosk-model-cn-0.22"
SAMPLE_RATE = 16000
CHANNELS = 1

class SpeechToText:
    """封装Vosk本地语音识别功能。"""
    def __init__(self, model_path=MODEL_PATH):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"错误：Vosk模型文件夹未找到，请下载并解压到 '{model_path}'。\n"
                f"下载地址: https://alphacephei.com/vosk/models"
            )
        print("[STT] 正在加载本地Vosk模型...")
        self.model = vosk.Model(model_path)
        self.recognizer = vosk.KaldiRecognizer(self.model, SAMPLE_RATE)
        print("[STT] 模型加载成功。")

    def listen(self, duration: int = 5) -> str:
        """
        从麦克风录制指定时长的音频，并进行实时识别。
        返回最终识别出的完整文本。
        """
        q = queue.Queue()

        def audio_callback(indata, frames, time, status):
            if status:
                print(status, file=sys.stderr)
            q.put(bytes(indata))

        print("\n" + "="*40)
        print("准备录音...")
        for i in range(3, 0, -1):
            print(f"{i}...", end="", flush=True)
            sleep(1)
        
        print(f"请开始说话！(将录制 {duration} 秒)")

        try:
            with sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=8000,
                                   dtype='int16', channels=CHANNELS, callback=audio_callback):
                
                start_time = time()
                full_transcript = ""

                while time() - start_time < duration:
                    data = q.get()
                    if self.recognizer.AcceptWaveform(data):
                        result_json = json.loads(self.recognizer.Result())
                        text = result_json.get('text', '')
                        if text:
                            # 实时打印已确认的片段
                            sys.stdout.write('\r' + ' ' * 60 + '\r') # 清除行
                            print(f"已识别片段: '{text}'")
                            full_transcript += text + " "
                    else:
                        partial_result = json.loads(self.recognizer.PartialResult())
                        sys.stdout.write('\r实时识别中: ' + partial_result.get('partial', ''))
                        sys.stdout.flush()

                # 处理最后剩余的音频数据
                final_result_json = json.loads(self.recognizer.FinalResult())
                full_transcript += final_result_json.get('text', '')
                
                sys.stdout.write('\r' + ' ' * 60 + '\r') # 清除最后一行
                print("录音结束。")
                
                final_text = full_transcript.strip()
                # 过滤掉常见的无意义识别结果
                if not final_text or final_text in ["嗯", "啊"]:
                    return ""
                return final_text

        except Exception as e:
            print(f"!!! 语音识别时发生错误: {e}")
            return ""

def listen_for_command(duration: int = 5) -> str:
    """一个简单的包装函数，方便外部调用。"""
    try:
        stt_module = SpeechToText()
        command = stt_module.listen(duration)
        return command
    except FileNotFoundError as e:
        print(e)
        return ""


if __name__ == '__main__':
    # 独立测试 stt.py
    print("--- 语音识别模块独立测试 ---")
    command = listen_for_command(duration=5)
    if command:
        print("\n" + "="*20)
        print(f"最终指令: '{command}'")
        print("="*20)
    else:
        print("\n未能识别到有效指令。")
