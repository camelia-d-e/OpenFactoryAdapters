import sys
import socket
import socketserver
import getpass

class AgentRequestHandler(socketserver.BaseRequestHandler):
    """
    Handles requests from the MTConnect Agent
    Reads data from a MTCDevice and sends them in SHDR format to the connected agent
    Defintion of the Adapter Agent protocol : https://www.mtcup.org/Protocol
    """

    HEARTBEAT_TIMEOUT = 10000
    DEBUG = False

    # TCP Keepalive configuration
    TCP_KEEPIDLE = 60         # Idle time before sending keepalive probes
    TCP_KEEPINTVL = 10        # Interval between keepalive probes
    TCP_KEEPCNT = 3           # Number of failed probes before closing

    __data_old__ = {}

    def __init__(self, request, client_address, server):
        """
        Constructor
        """
        self.device = server.device
        socketserver.BaseRequestHandler.__init__(self, request, client_address, server)

    def send_shdr(self, data):
        """
        Send SHDR data to agent
        """
        for id in data:
            if id not in self.__data_old__:
                self.__data_old__[id] = ''
            if data[id] != self.__data_old__[id]:
                self.request.sendall((f"|{id}|{data[id]}\n").encode())
                if self.DEBUG:
                    print(f"|{id}|{data[id]}")
                if id != 'avail':
                    self.__data_old__[id] = data[id]

    def handle(self):
        """
        Handle connection from MTConnect Agent
        """
        print(f"Connection from {self.client_address[0]}")

        # enable TCP keepalive
        self.request.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

        # configure keepalive options
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, self.TCP_KEEPIDLE)
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, self.TCP_KEEPINTVL)
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, self.TCP_KEEPCNT)

        # send avail status
        if self.device.health_check():
            self.request.sendall(("|avail|AVAILABLE\n").encode())
        else:
            self.request.sendall(("|avail|UNAVAILABLE\n").encode())
            return
        self.request.settimeout(0.01)

        # send initial SHDR data
        self.request.sendall(("|operator|" + getpass.getuser() + "\n").encode())
        self.request.sendall(("* shdrVersion: 2.0\n").encode())
        self.request.sendall((f"* adapterVersion: {2.0}\n").encode())
        manufacturer = self.device.manufacturer()
        serialNumber = self.device.serialNumber()
        if manufacturer:
            self.request.sendall((f"* manufacturer: {manufacturer}\n").encode())
        if serialNumber:
            self.request.sendall((f"* serialNumber: {serialNumber}\n").encode())
        self.send_shdr(self.device.read_data())
        self.send_shdr(self.device.on_connect())

        while 1:
            try:
                data = self.request.recv(1024)
            except socket.timeout:
                device_data = self.device.read_data()
                self.send_shdr(device_data)
                continue
            except (ConnectionResetError, BrokenPipeError):
                print("Connection from Agent closed")
                break

            if not data:
                print("Connection from Agent closed")
                break

            if self.DEBUG:
                print("Agent sent:", data.decode())

            if "* PING" in data.decode():
                self.request.sendall(f"* PONG {str(self.HEARTBEAT_TIMEOUT)}\n".encode())
                if self.DEBUG:
                    print(f"* PONG {str(self.HEARTBEAT_TIMEOUT)}")


class MTCAdapter(socketserver.TCPServer):
    """
    Implements a MTConnect Adapter as a TCP server
    Handles requests from the Agent with agentRequestHandler_class
    """

    adapter_port = 7880
    device_class = None
    agentRequestHandler_class = AgentRequestHandler

    def __init__(self):
        """
        Constructor
        """
        # Configuration validations
        if self.device_class is None:
            raise ImproperlyConfigured("MTCAdapter requires the attribute 'device_class' to be defined")

        print("Setting up connection to device")
        self.device = self.device_class()

        print("Creating Adapter")
        socketserver.TCPServer.__init__(self, (self.get_ip(), self.adapter_port), self.agentRequestHandler_class)
        print(f'Running on : {self.server_address[0]}')

    def get_ip(self)-> str:
        """ Get the current IP address of the machine
        Returns:
            str: IP address of the machine
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(('10.255.255.255', 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip

    def run(self):
        """
        Run the adapter
        """
        print(f"Starting Adapter on port {self.adapter_port}")
        try:
            self.serve_forever()
        except KeyboardInterrupt:
            sys.exit(0)

class ImproperlyConfigured(Exception):
    """ Something is improperly configured """
    pass