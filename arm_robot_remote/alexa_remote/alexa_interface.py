#!/usr/bin/env python3

from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractExceptionHandler, AbstractRequestHandler
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arm_robot_msgs.action import ArmRobotTask
import threading


threading.Thread(target=lambda: rclpy.init()).start()
action_client= ActionClient(Node("alexa_interface"), ArmRobotTask, "task_server")
app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, how can I help you today..?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        goal= ArmRobotTask.Goal()
        goal.task_number=0
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    
class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, Pickup task performing"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)
        goal= ArmRobotTask.Goal()
        goal.task_number=1
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response

    
class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, The robot is going to sleep"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(
            True)
        goal= ArmRobotTask.Goal()
        goal.task_number=2
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    
class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm ready to perform the task"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(
            True)
        goal= ArmRobotTask.Goal()
        goal.task_number=0
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response



class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Sorry, I didn't understand. Could you please repeat the request..?!!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response
    

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())
# Register your intent handlers to the skill_builder object

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.1fdde450-a8e0-40a8-9a62-842bd5a826c0", app=app)

# @app.route("/")
# def invoke_skill():
#     return skill_adapter.dispatch_request()


skill_adapter.register(app=app, route="/")


if __name__=="__main__":
    app.run()

