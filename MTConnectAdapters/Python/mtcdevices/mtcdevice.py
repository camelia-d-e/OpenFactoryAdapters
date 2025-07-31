from abc import ABC, abstractmethod
from typing import Optional


class MTCDevice(ABC):
    """
    Base class for MTCDevices
    """

    @abstractmethod
    def manufacturer(self)-> Optional[str]:
        """
        Return device manufacturer
        """
        return None

    @abstractmethod
    def serialNumber(self)-> Optional[str]:
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
    def health_check(self)-> Optional[bool]:
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