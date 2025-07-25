import threading
import asyncio
from typing import Dict, List, Optional
import pyshark

class DustTrak:
    """DustTrak device"""

    def __init__(self, device_ip: str='169.254.66.117'):
        self.device_ip = device_ip
        self.latest_data = {
            'pm1_concentration': 0.0,
            'pm2_5_concentration': 0.0,
            'pm4_concentration': 0.0,
            'pm10_concentration': 0.0,
            'avail': 'UNAVAILABLE'
        }
        self.capture_thread = None
        self.running = False

    @property
    def name(self):
        """Device name"""
        return "DustTrak"

    def health_check(self):
        """Health check"""
        return True

    def manufacturer(self):
        """Device manufacturer"""
        return "TSI"

    def serialNumber(self):
        """Device serial number."""
        return "8543210412"

    def on_connect(self):
        """Data sent on first connection."""
        self.start_capture()
        return {
            "avail": "AVAILABLE",
        }

    def start_capture(self):
        """Start capturing data in a background thread"""
        if not self.running:
            self.running = True
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            print("Started data capture thread")

    def stop_capture(self):
        """Stop capturing data"""
        self.running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2)
            print("Stopped data capture thread")

    def _capture_loop(self):
        """Continuous capture loop running in background thread"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            print(f"Starting packet capture on Ethernet 4 from {self.device_ip}...")

            capture = pyshark.LiveCapture(
                interface='Ethernet 4',
                display_filter=f'ip.src=={self.device_ip}'
            )

            for packet in capture.sniff_continuously():
                if not self.running:
                    break

                if hasattr(packet, 'tcp') and hasattr(packet.tcp, 'payload'):
                    packet_len = int(packet.length)

                    if packet_len == 91:
                        parsed_data = self.parse_hex_data(packet.tcp.payload)
                        if parsed_data:
                            converted_data = self.convert_to_percent(parsed_data)

                            self.latest_data = {
                                'pm1_concentration': converted_data[0],
                                'pm2_5_concentration': converted_data[1],
                                'pm4_concentration': converted_data[2],
                                'pm10_concentration': converted_data[3],
                                'avail': 'AVAILABLE'
                            }
                            print(f"Updated data: {self.latest_data}")
        except Exception as e:
            print(f"Error in capture loop: {e}")
            self.latest_data['avail'] = 'UNAVAILABLE'
        finally:
            try:
                loop.close()
            except Exception as e:
                print(f"Error closing event loop: {e}")

    def read_data(self) -> Dict[str, float]:
        """Return the latest captured data"""
        return self.latest_data.copy()

    def parse_hex_data(self, raw_data: str) -> Optional[List[str]]:
        """Decodes and parses raw hex data from TCP messages"""
        try:
            bytes_obj = bytes.fromhex(raw_data.replace(":", ""))
            decoded_str = bytes_obj.decode('utf-8')

            parsed_data = decoded_str.split(',')
            data_values = parsed_data[1:-1]
            print(f"Parsed data: {data_values}")
            return data_values
        except (ValueError, UnicodeDecodeError) as e:
            print(f"Error parsing hex data: {e}")
            return None

    def convert_to_percent(self, concentrations: List[str]) -> List[float]:
        """Convert concentration values to percentage"""
        percentages = []
        for concentration in concentrations:
            try:
                value = (float(concentration) / 1225000) * 100
                percentages.append(value)
            except (ValueError, ZeroDivisionError) as e:
                print(f"Error converting {concentration}: {e}")
                percentages.append(0.0)
        return percentages
