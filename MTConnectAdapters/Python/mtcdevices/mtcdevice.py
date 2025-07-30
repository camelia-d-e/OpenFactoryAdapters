from abc import ABC, abstractmethod


class MTCDevice(ABC):
    """
    Base class for MTCDevices
    """

    @abstractmethod
    def manufacturer(self):
        """
        Return device manufacturer
        """
        return None

    @abstractmethod
    def serialNumber(self):
        """
        Return device serial number
        """
        return None

    @abstractmethod
    def on_connect(self):
        """
        Send initial data when MTConnect Agent connects
        """
        return {}

    @abstractmethod
    def health_check(self):
        """
        Return health status of device connection
        """
        return True

    @abstractmethod
    def read_data(self):
        """
        Read and return device data
        """
        return {}