#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


import rospy
#from streamlit import empty, session_state
from typing import List
from pydantic import BaseModel, Field
from langchain_openai import ChatOpenAI
from langchain.schema import HumanMessage, AIMessage
from langchain.prompts.chat import SystemMessagePromptTemplate
from langchain.output_parsers import PydanticOutputParser

from config.config import LLMConfig

# GPT api key
os.environ["OPENAI_API_KEY"] = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'../keys/openai.key'), 'r').readline().rstrip()


class Plan(BaseModel):
	tasks: List[str] = Field(description="list of all tasks to be carried out")
	
	def pretty_print(cls):
		pretty_msg = "Tasks:\n"
		for i, task in enumerate(cls.tasks):
			pretty_msg += f"{i+1}. {task}\n"
		return pretty_msg+'\n'

class Optimization(BaseModel):
	objective: str = Field(description="objective function to be applied to MPC")
	equality_constraints: List[str] = Field(description="equality constraints to be applied to MPC")
	inequality_constraints: List[str] = Field(description="inequality constraints to be applied to MPC")

	def pretty_print(cls):
		pretty_msg = "Applying the following MPC fomulation:\n```\n"
		pretty_msg += f"min {cls.objective}\n"
		pretty_msg += f"s.t.\n"
		for c in cls.equality_constraints:
			pretty_msg += f"\t {c} = 0\n"
		for c in cls.inequality_constraints:
			pretty_msg += f"\t {c} <= 0\n"
		return pretty_msg+"\n```\n"

class LLM():

	def __init__(self, output:str=None, cfg=LLMConfig()) -> None:
		assert output in ["plan", "optimization", None], f"{output} is neither plan nor optimization"
		self.cfg =cfg
		self.output = output
		# init parser
		self.parser = PydanticOutputParser(pydantic_object=Plan if output=="plan" else Optimization)
		# init model
		self.model = ChatOpenAI(
			model_name=self.cfg.model_name, 
			temperature=self.cfg.temperature,
			streaming=self.cfg.streaming,
			callbacks=None
		)
		# init prompt
		system_prompt = SystemMessagePromptTemplate.from_template(self.cfg.prompt)
		self.messages = [system_prompt.format(format_instructions=self.parser.get_format_instructions())]    
		

	def step(self, user_message:str) -> str:
		self.messages.append(HumanMessage(content=user_message))
		if self.cfg.mock_task is None:
			model_message = self.model(self.messages)
		else:
			# TODO: mock
			pass
		self.messages.append(model_message)
		print(f"\33[92m {model_message.content} \033[0m \n")
		#return self.parser.parse(model_message.content)
		return model_message.content if self.output is None else self.parser.parse(model_message.content)
	

	def run(self):
		while not rospy.is_shutdown():
			user_msg = input("Ask me anything:")
			respone = self.step(user_msg)
			print(respone)


if __name__ == "__main__":
	llm = LLM()
	llm.run()