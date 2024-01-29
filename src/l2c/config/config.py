
class ControllerConfig:
    nx: int = 3
    nu: int = 3 
    T: int = 15
    dt: float = 0.05
    lu: float = -0.2 # lower bound on u
    hu: float = 0.2  # higher bound on u
    model_type: str = "discrete"
    penalty_term_cons: float = 1e7