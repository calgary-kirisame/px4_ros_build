#!/usr/bin/env python3
import select
import shutil
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import speech_recognition as sr
    _SPEECH_AVAILABLE = True
except Exception:
    sr = None
    _SPEECH_AVAILABLE = False

try:
    from pocketsphinx import Decoder as PocketSphinxDecoder
    from pocketsphinx import get_model_path as pocketsphinx_model_path
    _POCKETSPHINX_AVAILABLE = True
except Exception:
    PocketSphinxDecoder = None
    pocketsphinx_model_path = None
    _POCKETSPHINX_AVAILABLE = False


START_VOICE_PHRASE = 'jarvis start mission'
END_VOICE_PHRASE = 'jarvis terminate mission'


class StartMissionGate(Node):
    def __init__(self):
        super().__init__('start_mission_gate')
        self.start_publisher = self.create_publisher(Bool, '/px4_single_plan/start_mission', 10)
        self.end_publisher = self.create_publisher(Bool, '/px4_single_plan/end_mission', 10)
        self._start_triggered = threading.Event()
        self._end_triggered = threading.Event()
        self._voice_thread = None
        self._warned_google_unavailable = False

        print('Start gate ready.')
        print('Press ENTER, type "start", or say "Jarvis Start Mission" to begin.')
        print('Type "terminate" or say "Jarvis Terminate Mission" at any time to command landing.')
        if not _SPEECH_AVAILABLE:
            print('Voice recognition unavailable: install python3-speechrecognition and a microphone backend.')
        else:
            self._voice_thread = threading.Thread(target=self._voice_loop, daemon=True)
            self._voice_thread.start()

    def trigger_start(self, source: str):
        if self._start_triggered.is_set():
            return

        self._start_triggered.set()
        self.get_logger().info(f'Start triggered by {source}. Publishing /px4_single_plan/start_mission.')
        msg = Bool()
        msg.data = True
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.start_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0)

    def trigger_end(self, source: str):
        if self._end_triggered.is_set():
            return

        self._end_triggered.set()
        self.get_logger().info(f'End mission triggered by {source}. Publishing /px4_single_plan/end_mission.')
        msg = Bool()
        msg.data = True
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.end_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0)

    def _voice_loop(self):
        recognizer = sr.Recognizer()
        recognizer.dynamic_energy_threshold = True

        try:
            with sr.Microphone() as source:
                print('Calibrating microphone...')
                recognizer.adjust_for_ambient_noise(source, duration=1.0)
                print('Listening for voice mission commands...')

                while rclpy.ok() and not self._end_triggered.is_set():
                    try:
                        audio = recognizer.listen(source, timeout=1.0, phrase_time_limit=4.0)
                    except sr.WaitTimeoutError:
                        continue

                    try:
                        transcript = self._recognize_phrase(recognizer, audio)
                    except Exception as exc:
                        self.get_logger().warn(f'Voice recognition error: {exc}')
                        time.sleep(1.0)
                        continue

                    if transcript:
                        print(f'Heard: {transcript}')
                    if self._matches_end_phrase(transcript):
                        self.trigger_end('voice command')
                        return
                    if self._matches_start_phrase(transcript):
                        self.trigger_start('voice command')
                        continue
                    continue
        except Exception as exc:
            self.get_logger().warn(f'Voice input unavailable: {exc}')

    def _recognize_phrase(self, recognizer, audio) -> str:
        if _POCKETSPHINX_AVAILABLE:
            try:
                return self._recognize_phrase_with_pocketsphinx(audio)
            except Exception as exc:
                if not self._warned_google_unavailable and shutil.which('flac') is None:
                    self._warned_google_unavailable = True
                    self.get_logger().warn(
                        f'Offline Sphinx recognition failed ({exc}). Google fallback disabled because `flac` is not installed. '
                        'Press ENTER to start, or install `flac` if you want the Google fallback path.'
                    )
                    return ''

        if shutil.which('flac') is None:
            if not self._warned_google_unavailable:
                self._warned_google_unavailable = True
                self.get_logger().warn(
                    'Offline Sphinx recognition is unavailable and Google fallback is disabled because `flac` is not installed. '
                    'Press ENTER to start, or install `flac` to enable the Google fallback path.'
                )
            return ''
        return recognizer.recognize_google(audio).strip().lower()

    @staticmethod
    def _recognize_phrase_with_pocketsphinx(audio) -> str:
        model_path = pocketsphinx_model_path()
        config = PocketSphinxDecoder.default_config()
        config.set_string('-hmm', f'{model_path}/en-us')
        config.set_string('-lm', f'{model_path}/en-us.lm.bin')
        config.set_string('-dict', f'{model_path}/cmudict-en-us.dict')
        config.set_string('-logfn', '/dev/null')
        decoder = PocketSphinxDecoder(config)

        raw_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
        decoder.start_utt()
        decoder.process_raw(raw_data, False, True)
        decoder.end_utt()

        hypothesis = decoder.hyp()
        if hypothesis is None:
            return ''
        return hypothesis.hypstr.strip().lower()

    @staticmethod
    def _normalize_command(text: str) -> str:
        return ' '.join(text.lower().split())

    @classmethod
    def _matches_manual_start_command(cls, text: str) -> bool:
        normalized = cls._normalize_command(text)
        return normalized in {'start', 'start mission', START_VOICE_PHRASE}

    @classmethod
    def _matches_manual_end_command(cls, text: str) -> bool:
        normalized = cls._normalize_command(text)
        return normalized in {'terminate', 'terminate mission', END_VOICE_PHRASE}

    @staticmethod
    def _matches_start_phrase(transcript: str) -> bool:
        normalized = StartMissionGate._normalize_command(transcript)
        if normalized == START_VOICE_PHRASE:
            return True
        return all(token in normalized.split() for token in ('jarvis', 'start', 'mission'))

    @staticmethod
    def _matches_end_phrase(transcript: str) -> bool:
        normalized = StartMissionGate._normalize_command(transcript)
        if normalized == END_VOICE_PHRASE:
            return True
        return all(token in normalized.split() for token in ('jarvis', 'terminate', 'mission'))


def main(args=None):
    rclpy.init(args=args)
    node = StartMissionGate()
    try:
        while rclpy.ok() and not node._end_triggered.is_set():
            if select.select([sys.stdin], [], [], 0.2)[0]:
                line = sys.stdin.readline()
                if line == '':
                    break
                command = line.rstrip('\n')
                if command == '':
                    node.trigger_start('keyboard')
                    continue
                if node._matches_manual_start_command(command):
                    node.trigger_start('typed command')
                    continue
                if node._matches_manual_end_command(command):
                    node.trigger_end('typed command')
                    continue
                print('Unknown command. Use ENTER/start to begin or terminate to land.')
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
