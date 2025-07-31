from adapters.mtcadapter import MTCAdapter
from mtcdevices.witmotion.wtvb01 import WTVB01


class MyWTVB01(WTVB01):
    port = "COM6"
    temperature_id = None

class WTVB01Adapter(MTCAdapter):
    device_class = WTVB01
    adapter_port = 7881

def main():
    adapter = WTVB01Adapter()
    try:
        adapter.run()
    except KeyboardInterrupt:
        print("\nShutting down adapter...")


if __name__ == "__main__":
    main()