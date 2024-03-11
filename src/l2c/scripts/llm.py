#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import requests
import json
from time import time

import rospy
import actionlib
import franka_gripper.msg
from std_msgs.msg import Empty, String


from config.config import LLMConfig
from l2c.msg import Optimization, Plan

# GPT api key
os.environ["OPENAI_API_KEY"] = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'../keys/openai.key'), 'r').readline().rstrip()

OPEN_GRIPPER_SET = ["open gripper", "open_gripper", "open the gripper"]
CLOSE_GRIPPER_SET = ["close gripper", "close_gripper", "close the gripper"]
	
def close_gripper(width=0.048):
    goal = franka_gripper.msg.GraspActionGoal()
    goal.goal.width = width
    goal.goal.epsilon.inner = 0.01
    goal.goal.epsilon.outer = 0.01
    goal.goal.speed = 0.1
    goal.goal.force = 40.0

    client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
    client.wait_for_server()
    client.send_goal(goal.goal)

    client.wait_for_result()

    return client.get_result()

def open_gripper():
    goal = franka_gripper.msg.MoveActionGoal()

    goal.goal.width = 0.08
    goal.goal.speed = 0.1
        
    client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
    client.wait_for_server()
    client.send_goal(goal.goal)
    client.wait_for_result()

    return client.get_result()

class Message:
  def __init__(self, text, base64_image=None, role="user"):
    self.role = role
    self.text = text
    self.base64_image = base64_image

  def to_dict(self):
    message = [{"type": "text", "text": self.text}]
    if self.base64_image:
      message.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{self.base64_image}", "detail": "high"}})
    return {"role": self.role, "content": message}

class BaseLLM:

  def __init__(self, role, cfg=LLMConfig()) -> None:
    self.cfg = cfg
    self.role = role

    self.messages = [Message(text=self.cfg.prompts_dict[role], role="system")]
    self.headers = {
      "Content-Type": "application/json",
      "Authorization": f"Bearer {os.getenv('OPENAI_API_KEY')}"
    }

  def run(self, user_message:str, base64_image=None, short_history=False) -> dict:
    self.messages.append(Message(text=user_message, role="user", base64_image=base64_image))
    selected_messages = [self.messages[0]] + [m for m in self.messages[-1:] if m.role!="system"] if short_history else self.messages
    payload = {
      "model": self.cfg.model_name,
      "messages": [m.to_dict() for m in selected_messages],
      "response_format": {"type": "json_object"}
    }
    t0 = time()
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=self.headers, json=payload).json()
    solve_time = time() - t0
    try:
      AI_response = response['choices'][0]['message']['content']
      self.messages.append(Message(text=AI_response, role="assistant"))
      AI_response = json.loads(AI_response)
    except Exception as e:
      print(f"Error: {e}")
      AI_response = {"instruction": response['error']['message']}

    AI_response["solve_time"] = solve_time
    return AI_response

class LLM:
	def __init__(self) -> None:
		rospy.init_node('llm', anonymous=True)
		self.substask_sub = rospy.Subscriber('/task', String, self.task_callback)
		self.plan_sub = rospy.Subscriber('/user_message', String, self.user_message_callback)
		self.optimization_trigger_sub = rospy.Subscriber('/optimization_trigger', Empty, self.optimization_trigger_callback)
		self.optimization_trigger_sub = rospy.Subscriber('/manual_optimization_trigger', Empty, self.optimization_trigger_callback)

		self.plan_pub = rospy.Publisher('/plan', Plan, queue_size=1)
		self.optimization_pub = rospy.Publisher('/optimization', Optimization, queue_size=1)
		self.gui_plan_pub = rospy.Publisher('/gui_plan', String, queue_size=1)
		self.gui_optimization_pub = rospy.Publisher('/gui_optimization', String, queue_size=1)
		self.tp_solvetime_pub = rospy.Publisher('/tp_solvetime', String, queue_size=10)
		self.od_solvetime_pub = rospy.Publisher('/od_solvetime', String, queue_size=10)

		self.TP = BaseLLM(role="TP")
		self.OD = BaseLLM(role="OD")

		self.plan = None
		self.task_id = 0
	
	def pub_solvetime_tp(self, solve_time):
		msg = String()
		msg.data = str(solve_time)
		self.tp_solvetime_pub.publish(msg)

	def pub_solvetime_od(self, solve_time):
		msg = String()
		msg.data = str(solve_time)
		self.od_solvetime_pub.publish(msg)

	def user_message_callback(self, user_message:String):
		user_message = user_message.data
		assert user_message[:2] in ["TP", "OD"], f"user_message doesnt specify if it's for TP or OD" 
		message = f"objects = {['blue_cube', 'green_cube', 'yellow_cube', 'red_cube']}\n"
		message += f"# Query: {user_message[3:]}"
		print(f"message: {message}")
		if user_message[:2] == "TP":
			self.plan_callback(message)
		else:  
			self.task_callback(message)
		

	def plan_callback(self, user_message):
		st = time()
		plan = self.TP.run(user_message)
		et = time()
		self.plan = plan
		self.task_id = 0
		plan_msg = Plan()
		plan_msg.tasks = plan["tasks"]
		self.plan_pub.publish(plan_msg)
		self.gui_plan_pub.publish(self.pretty_print_tp(plan))
		self.optimization_trigger_callback(Empty())
		self.pub_solvetime_tp(et-st)


	def task_callback(self, subtask:String):
		task_str = subtask
		if any([g in task_str.lower() for g in OPEN_GRIPPER_SET]):
			open_gripper()
			response_msg = String()
			response_msg.data = "open_gripper()"
			self.gui_optimization_pub.publish(response_msg)
			return
		elif any([g in task_str.lower() for g in CLOSE_GRIPPER_SET]):
			close_gripper()
			response_msg = String()
			response_msg.data = "close_gripper()"
			self.gui_optimization_pub.publish(response_msg)
			return
		st = time()
		optimization = self.OD.run(task_str)
		et = time()
		print(f"optimization: {optimization}")
		optimization_msg = Optimization()
		optimization_msg.objective = optimization["objective"]
		optimization_msg.equality_constraints = optimization["equality_constraints"]
		optimization_msg.inequality_constraints = optimization["inequality_constraints"]
		self.optimization_pub.publish(optimization_msg)
		self.gui_optimization_pub.publish(self.pretty_print_od(optimization))
		self.pub_solvetime_od(et-st)

	def optimization_trigger_callback(self, data:Empty):
		self.task_callback(self.plan["tasks"][self.task_id])
		self.task_id += 1
	
	def pretty_print_od(self, optimization):
		pretty_msg = "Applying the following MPC fomulation:\n```\n"
		pretty_msg += f"min {optimization['objective']}\n"
		pretty_msg += f"s.t.\n"
		for c in optimization["equality_constraints"]:
			pretty_msg += f"\t {c} = 0\n"
		for c in optimization["inequality_constraints"]:
			pretty_msg += f"\t {c} <= 0\n"
		return pretty_msg+"\n```\n"
	
	def pretty_print_tp(self, plan):
		pretty_msg = "Tasks:\n"
		for i, task in enumerate(plan["tasks"]):
			pretty_msg += f"{i+1}. {task}\n"
		return pretty_msg+'\n'


if __name__ == "__main__":
	llm = LLM()
	rospy.spin()