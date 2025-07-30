from adapters.mtcadapter import MTCAdapter
from mtcdevices.dust_trak.dust_trak import DustTrak

class DustTrakAdapter(MTCAdapter):
    device_class = DustTrak
    adapter_port = 7880

    def __init__(self):
        super().__init__()
        self.device = self.device_class(readings_average_num=10)
    
    def run(self):
        """Start MTConnect adapter"""
        print("Starting DustTrak MTConnect adapter...")
        print(f"Listening on port {self.adapter_port}")
        try:
            super().run()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.device.stop_capture()

def main():
    adapter = DustTrakAdapter()
    try:
        adapter.run()
    except KeyboardInterrupt:
        print("\nShutting down adapter...")
    except Exception as e:
        print(f"Error running adapter: {e}")
    finally:
        if hasattr(adapter, 'device'):
            adapter.device.stop_capture()

if __name__ == "__main__":
    main()