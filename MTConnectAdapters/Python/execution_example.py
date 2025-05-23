from adapters.mtcadapter import MTCAdapter
from mtcdevices.minimaldevice import MinimalDevice


class MyAdapter(MTCAdapter):
    adapter_port = 7878
    device_class = MinimalDevice


def main():
    myadapter = MyAdapter()
    myadapter.run()


if __name__ == "__main__":
    main()
