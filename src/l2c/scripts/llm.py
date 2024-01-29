#!/usr/bin/env python3
from typing import List
from pydantic import BaseModel, Field


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