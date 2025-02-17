# sensor_feedback.py
# Integration of Tactile and Vision Sensors
class SensorFeedback:
    def __init__(self):
        self.vision_data = None
        self.tactile_data = None

    def update_vision(self, data):
        self.vision_data = data

    def update_tactile(self, data):
        self.tactile_data = data
