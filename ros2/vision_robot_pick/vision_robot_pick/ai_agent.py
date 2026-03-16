#!/usr/bin/env python3
"""
AI Agent Node - Natural Language Robot Commander
Subscribes to /ai_command (String) from Unity
Uses Gemini with function calling to parse intent
Publishes /robot_command (String) and speed to commander

Run:
  ros2 run vision_robot_pick ai_agent

Send command from Unity or terminal:
  ros2 topic pub /ai_command std_msgs/String "data: 'go to phone with speed 20%'" --once
  ros2 topic pub /ai_command std_msgs/String "data: 'move to car slowly'" --once

Install:
  pip install google-generativeai
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
import json
import threading

# ── Config ────────────────────────────────────────────────────────────────────
GEMINI_API_KEY = 'api_key'
GEMINI_MODEL   = 'gemini-2.5-flash'   # fast and cheap

# Known detectable objects (must match YOLO labels)
KNOWN_OBJECTS = ['car', 'cell phone']

# Speed limits
SPEED_MIN =  5
SPEED_MAX = 50
SPEED_DEFAULT = 20


class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # ── Gemini setup ───────────────────────────────────────────────────
        genai.configure(api_key=GEMINI_API_KEY)

        # Define move_to function tool
        self.tools = [
            genai.protos.Tool(
                function_declarations=[
                    genai.protos.FunctionDeclaration(
                        name='move_to',
                        description=(
                            'Move the robot arm above a detected object. '
                            'Use this when the user wants to move to, go to, '
                            'pick up, or reach any object.'
                        ),
                        parameters=genai.protos.Schema(
                            type=genai.protos.Type.OBJECT,
                            properties={
                                'object_name': genai.protos.Schema(
                                    type=genai.protos.Type.STRING,
                                    description=(
                                        f'Name of the object to move to. '
                                        f'Must be one of: {KNOWN_OBJECTS}. '
                                        f'Map common synonyms: '
                                        f'phone/mobile/smartphone → cell phone, '
                                        f'vehicle/automobile/toy car → car.'
                                    )
                                ),
                                'speed': genai.protos.Schema(
                                    type=genai.protos.Type.INTEGER,
                                    description=(
                                        f'Movement speed as percentage {SPEED_MIN}-{SPEED_MAX}. '
                                        f'slow=10, normal=20, fast=40. '
                                        f'Default={SPEED_DEFAULT} if not specified.'
                                    )
                                ),
                            },
                            required=['object_name', 'speed']
                        )
                    )
                ]
            )
        ]

        self.model = genai.GenerativeModel(
            model_name=GEMINI_MODEL,
            tools=self.tools,
            system_instruction=(
                f'You are a robot arm controller. '
                f'When the user gives a movement command, '
                f'always call the move_to function. '
                f'Known objects: {KNOWN_OBJECTS}. '
                f'Speed range: {SPEED_MIN}-{SPEED_MAX}%. '
                f'If speed is not specified use {SPEED_DEFAULT}. '
                f'If object is unclear, pick the closest match from known objects.'
            )
        )

        # ── ROS subscribers/publishers ─────────────────────────────────────
        self.create_subscription(
            String, '/ai_command',
            self.command_cb, 10)

        self.cmd_pub   = self.create_publisher(String, '/robot_command', 10)
        self.speed_pub = self.create_publisher(String, '/robot_speed',   10)
        self.reply_pub = self.create_publisher(String, '/ai_reply',      10)

        self.processing = False
        self.lock       = threading.Lock()

        print('\n' + '='*52)
        print('  AI Agent Node (Gemini)')
        print(f'  Model   : {GEMINI_MODEL}')
        print(f'  Objects : {KNOWN_OBJECTS}')
        print(f'  Speed   : {SPEED_MIN}-{SPEED_MAX}%')
        print('='*52)
        print('\n  Waiting for commands on /ai_command ...')
        print('  Example:')
        print('    ros2 topic pub /ai_command std_msgs/String \\')
        print('      "data: \'go to phone with speed 20%\'" --once\n')

    # ── Command callback ───────────────────────────────────────────────────

    def command_cb(self, msg: String):
        user_input = msg.data.strip()
        if not user_input:
            return

        with self.lock:
            if self.processing:
                self.get_logger().warn('Still processing previous command, skipping.')
                return
            self.processing = True

        print(f'\n  [USER] "{user_input}"')

        # Run Gemini in a thread to avoid blocking ROS spin
        threading.Thread(
            target=self._process_command,
            args=(user_input,),
            daemon=True
        ).start()

    # ── Process with Gemini ────────────────────────────────────────────────

    def _process_command(self, user_input: str):
        try:
            response = self.model.generate_content(user_input)

            # ── Check for function call ────────────────────────────────────
            tool_call = None
            for part in response.candidates[0].content.parts:
                if hasattr(part, 'function_call') and part.function_call:
                    tool_call = part.function_call
                    break

            if tool_call and tool_call.name == 'move_to':
                args        = dict(tool_call.args)
                object_name = str(args.get('object_name', '')).strip().lower()
                speed       = int(args.get('speed', SPEED_DEFAULT))

                # Clamp speed
                speed = max(SPEED_MIN, min(SPEED_MAX, speed))

                # Validate object
                if object_name not in KNOWN_OBJECTS:
                    # Try fuzzy match
                    object_name = self._fuzzy_match(object_name)

                print(f'  [GEMINI] move_to("{object_name}", speed={speed}%)')

                if object_name:
                    # Publish speed first
                    speed_msg      = String()
                    speed_msg.data = str(speed)
                    self.speed_pub.publish(speed_msg)

                    # Publish robot command
                    cmd_msg      = String()
                    cmd_msg.data = object_name
                    self.cmd_pub.publish(cmd_msg)

                    reply = f'Moving to {object_name} at {speed}% speed.'
                else:
                    reply = f'Object not recognized. Known objects: {KNOWN_OBJECTS}'

            else:
                # Gemini replied with text instead of tool call
                reply = response.text if hasattr(response, 'text') else \
                        'Could not understand command.'
                print(f'  [GEMINI] No tool call. Reply: {reply}')

            # ── Publish reply back to Unity ────────────────────────────────
            reply_msg      = String()
            reply_msg.data = reply
            self.reply_pub.publish(reply_msg)
            print(f'  [REPLY] {reply}')

        except Exception as e:
            print(f'  [ERROR] Gemini call failed: {e}')
            reply_msg      = String()
            reply_msg.data = f'Error: {str(e)}'
            self.reply_pub.publish(reply_msg)

        finally:
            with self.lock:
                self.processing = False

    # ── Fuzzy object matching ──────────────────────────────────────────────

    def _fuzzy_match(self, name: str) -> str:
        """Map common synonyms to known object labels."""
        synonyms = {
            'phone'      : 'cell phone',
            'mobile'     : 'cell phone',
            'smartphone' : 'cell phone',
            'iphone'     : 'cell phone',
            'telephone'  : 'cell phone',
            'vehicle'    : 'car',
            'automobile' : 'car',
            'toy'        : 'car',
            'auto'       : 'car',
        }
        # Direct synonym match
        if name in synonyms:
            return synonyms[name]
        # Partial match against known objects
        for obj in KNOWN_OBJECTS:
            if obj in name or name in obj:
                return obj
        return ''


def main():
    rclpy.init()
    node = AIAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
