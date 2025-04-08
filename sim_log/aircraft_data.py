import pandas as pd
from dataclasses import dataclass, field

@dataclass
class AircraftData:
    data_frame: pd.DataFrame = field(default_factory=lambda: pd.DataFrame(columns=[
        'time', 'x', 'y', 'z', 'u', 'v', 'w', 'phi', 'theta', 'psi',
        'p', 'q', 'r', 'Fx', 'Fy', 'Fz', 'l', 'm', 'n', 'motor1',
        'motor2', 'motor3', 'motor4', 'motor5', 'aileron', 'elevator',
        'rudder', 'mode'
    ]))

    def upload_data(self, data):
        """Append new data to the data frame."""
        self.data_frame = pd.concat([self.data_frame, pd.DataFrame([data])], ignore_index=True)

    def get_data(self):
        """Retrieve the entire data frame."""
        return self.data_frame
