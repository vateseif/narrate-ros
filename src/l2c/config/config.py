
class LLMConfig:
    mock_task = None#"OD_move_table"
    prompt: str = "" # TODO: this is bad. Only works for NMPC now
    avatar: str = "OD"
    parsing: str = "optimization"
    model_name: str = "gpt-3.5-turbo"
    streaming: bool = False # whether you want to stream text to the streamlit interface
    temperature: float = 0.6

class ControllerConfig:
    nx: int = 3
    nu: int = 3 
    T: int = 15
    dt: float = 0.05
    lu: float = -0.2 # lower bound on u
    hu: float = 0.2  # higher bound on u
    model_type: str = "discrete"
    penalty_term_cons: float = 1e7