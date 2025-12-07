---
sidebar_position: 4
title: Module 4 — Vision-Language-Action
---

# Module 4 — Vision-Language-Action

This module explores the intersection of computer vision, natural language processing, and robotics. These domains converge to enable intuitive and capable robotic systems, allowing robots to understand human commands, interpret their environment, and execute complex actions.

## Whisper → Voice Commands

Voice commands provide a natural interface for human-robot interaction. Automatic Speech Recognition (ASR) enables robots to understand spoken instructions. Whisper is a multilingual, general-purpose speech recognition model that excels even with noise or accents.

### Understanding Whisper

Whisper is trained on large-scale audio–text data and can transcribe or translate speech. Its robustness makes it ideal for robotics, where audio may not be perfect.

### Integrating Whisper for Voice Command Recognition

General workflow:

1. **Audio Capture:** Microphone records the user’s voice.
2. **Speech-to-Text Conversion:** Whisper transcribes the audio.
3. **Natural Language Understanding:** The transcription is processed to extract intent.
4. **Command Execution:** The robot maps intent → actions (e.g., via ROS 2).

### Example: Python Integration with Whisper (Safe Code Block)

```python
import whisper
import pyaudio
import wave

model = whisper.load_model("base")

def record_audio(filename="command.wav", duration=5):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000

    p = pyaudio.PyAudio()
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )

    print(f"* Recording for {duration} seconds...")
    frames = []

    for _ in range(0, int(RATE / CHUNK * duration)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("* Recording finished.")
    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(filename, "wb")
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b"".join(frames))
    wf.close()

    return filename

def transcribe_command(audio_file):
    result = model.transcribe(audio_file)
    return result["text"]

if __name__ == "__main__":
    audio_file = record_audio(duration=3)
    command_text = transcribe_command(audio_file)
    print(f"Transcribed Command: \"{command_text}\"")
