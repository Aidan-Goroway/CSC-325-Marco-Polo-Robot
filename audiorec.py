import pyaudio
import math
import struct
import wave
import time
import os
import subprocess

Threshold = 125

SHORT_NORMALIZE = (1.0/32768.0)
chunk = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
swidth = 2

TIMEOUT_LENGTH = 2

f_name_directory = r'/home/gorowaya/tmp'

class Recorder:

    @staticmethod
    def rms(frame):
        count = len(frame) / swidth
        format = "%dh" % (count)
        shorts = struct.unpack(format, frame)

        sum_squares = 0.0
        for sample in shorts:
            n = sample * SHORT_NORMALIZE
            sum_squares += n * n
        rms = math.pow(sum_squares / count, 0.5)

        return rms * 1000

    def __init__(self):
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=FORMAT,
                                  channels=CHANNELS,
                                  rate=RATE,
                                  input=True,
                                  output=True,
                                  frames_per_buffer=chunk)

    def record(self):
        print('Noise detected. Now recording:\n')
        rec = []
        current = time.time()
        end = time.time() + TIMEOUT_LENGTH

        while current <= end:

            data = self.stream.read(chunk)
            if self.rms(data) >= Threshold: end = time.time() + TIMEOUT_LENGTH

            current = time.time()
            rec.append(data)
            # print("still going")
        self.write(b''.join(rec))

    def write(self, recording):

        n_files = len(os.listdir(f_name_directory))

        filename = os.path.join(f_name_directory, 'test.wav'.format(n_files))

        wf = wave.open(filename, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(self.p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(recording)
        wf.close()
        print('Audio Recorded. Written to file: {}'.format(filename))
        # print('Returning to listening')


    def listen(self):
        print('Listening beginning')
        silent = True
        while silent:
            input = self.stream.read(chunk)
            rms_val = self.rms(input)
            if rms_val > Threshold:
                silent = False
                self.record()
                
    # Speech threshold around 125 RMS
    def testThresh(self):
        print('Testing beginning')
        while True:
            input = self.stream.read(chunk)
            rms_val = self.rms(input)
            print("RMS:",rms_val)


def audio2text(audioTest):
    """Wraps the process of recording, wav-converting, and Deepspeech processing into one function.

    
    Args:
        audioTest (bool):   If True, runs testThresh, testing if the microphone has the correct threshold for human voice.
                            Otherwise, runs the function normally.

    Returns:
        string: The speech recognized by the Deepspeech process. Comes pre-stripped.
        string: The error called if the Deepspeech subprocess fails.
        sting: A string that, reasonably, will never actually show up. This is because testThresh is an infinite loop.
    """
    audio = Recorder() # Audio write
    if not audioTest:
        audio.record() # writes wav file as well

        modelVar = os.path.abspath("/opt/deepspeech-0.9.3-models.pbmm")
        scorerVar = os.path.abspath("/opt/deepspeech-0.9.3-models.scorer")

        # Run DeepSpeech as a subprocess
        hotwordList = ["degrees:1", "meters:1", "left:1", "right:1", "forward:1", "backward:1", "thirty:1", "sixty:1", "ninety:1", "one:1", "two:1", "three:1", "four:1", "five:1"]
        command = ["deepspeech", "--model", modelVar, "--scorer", scorerVar, "--audio", "/home/gorowaya/tmp/test.wav"]
        for word in hotwordList:
            command.extend(["--hot_words", word])
        deepAudio = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if deepAudio.returncode == 0:
            print("Recognized words: " + deepAudio.stdout.strip())
            return deepAudio.stdout.strip()
        else:
            print("Error in subprocess: " + deepAudio.stderr)
            return "Error in subprocess: " + deepAudio.stderr
    else:
        audio.testThresh()
        return "Currently in audioTest-mode. You will NEVER see this."

# audio2text(False) # Set param to "True" to activate test-mode.
