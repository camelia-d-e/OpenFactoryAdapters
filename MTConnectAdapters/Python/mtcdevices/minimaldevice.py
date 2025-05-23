

class MinimalDevice():

    @property
    def name(self):
        return "Minimal-Device"

    def health_check(self):
        return True

    def manufacturer(self):
        return "Demo adapter"

    def serialNumber(self):
        return "SIM12345"

    def read_data(self):
        return {
            "mode": "AUTOMATIC",
            "execution": "INACTIVE",
        }

    def on_connect(self):
        return {
            "avail": "AVAILABLE",
        }