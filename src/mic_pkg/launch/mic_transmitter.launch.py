from launch import LaunchDescription
from launch_ros.actions import Node
import json

# Grammar constrains Vosk to this vocabulary — dramatically improves accuracy for
# alphanumeric codes on the small model. Say codes using NATO phonetic alphabet:
# "alpha one" → A1, "bravo charlie three" → BC3.
# Set to "" to disable grammar mode and use full free-speech recognition.
GRAMMAR_EN = json.dumps([
    # NATO phonetic alphabet (A–Z)
    "alpha", "bravo", "charlie", "delta", "echo", "foxtrot", "golf", "hotel",
    "india", "juliet", "kilo", "lima", "mike", "november", "oscar", "papa",
    "quebec", "romeo", "sierra", "tango", "uniform", "victor", "whiskey",
    "x-ray", "yankee", "zulu",
    # Single digits only — users speak each digit individually
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine",
    # Rescue / robot context words
    "sector", "area", "zone", "room", "floor", "building", "point", "location",
    "victim", "code", "hazmat", "number", "detected", "found", "confirmed",
    "the", "a", "at", "in", "is", "on",
    # Out-of-vocabulary fallback (keep — prevents hallucinations on unknown speech)
    "[unk]",
])


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mic_pkg',
            executable='mic_transmitter_node',
            name='mic_transmitter',
            output='screen',
            parameters=[{
                'device': 'auto',
                'usb_vendor_id': '4c4a',
                'usb_product_id': '4155',
                'sample_rate': 48000,
                'channels': 1,
                'frames_per_period': 512,
            }],
        ),
        Node(
            package='mic_pkg',
            executable='mic_receiver_node',
            name='mic_receiver',
            output='screen',
            parameters=[{
                'device': 'default',
                'sample_rate': 48000,
                'channels': 1,
                'frames_per_period': 512,
                'model_path_en': '/home/chumbi/roboticaWS/models/vosk-model-small-en-us-0.15',
                'model_path_es': '/home/chumbi/roboticaWS/models/vosk-model-small-es-0.42',
                'lang': 'en',
                'grammar_en': GRAMMAR_EN,
            }],
        ),
    ])
