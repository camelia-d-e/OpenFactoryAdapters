from datetime import datetime
from typing import Dict, List
import threading
import os
import time
import csv
import asyncio
import pyshark
import pyautogui


class DustTrak:
    """DustTrak device"""

    def __init__(self, device_ip: str='169.254.66.117', readings_average_num: int=1):
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
        self.data_updates_timer = 0
        self.no_packet_received_timer = 0
        self.readings_average_num = readings_average_num 
        self.launch_dust_trak_monitoring()

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
    
    def launch_dust_trak_monitoring(self):
        """Launch DustTrak monitoring"""
        try:
            pyautogui.hotkey('win', 'd')
            pyautogui.sleep(2)
        except Exception as e:
            print(f"Could not show desktop: {e}")

        current_screenshot_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates', 'current_desktop.png')
        pyautogui.screenshot(current_screenshot_path)
        print(f"Current desktop screenshot saved to: {current_screenshot_path}")

        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(current_dir, 'templates', 'power_automate_shortcut.png')
            if not os.path.exists(file_path):
                print(f"Template image not found at {file_path}")
                return
            
            shortcut_location = pyautogui.locateOnScreen(file_path, confidence=0.9)
            if shortcut_location:
                pyautogui.click(shortcut_location, button='left', clicks=2)
                pyautogui.sleep(10) # Wait for the application to open
                pyautogui.press('enter')
                pyautogui.press('tab', presses=4, interval=0.5) # Navigate to the readings input
                pyautogui.press('up', presses=self.readings_average_num-1, interval=0.5) # Set the number of readings
                pyautogui.press('enter')
            else:
                print("Shortcut not found on screen")
        except pyautogui.ImageNotFoundException:
            print("Could not find the shortcut image")

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
                    parsed_data: List[str] = self.parse_hex_data(packet.tcp.payload)
                    if not self.is_empty_data(parsed_data) and len(parsed_data) >= 4:
                        if parsed_data:
                            self.write_to_csv({
                                'pm1_concentration': parsed_data[0],
                                'pm2_5_concentration': parsed_data[1],
                                'pm4_concentration': parsed_data[2],
                                'pm10_concentration': parsed_data[3],
                            })

                            converted_data = self.convert_to_percent(parsed_data)

                            if not self.is_data_updated(converted_data) and self.data_updates_timer == 0:
                                self.data_updates_timer = time.time()
                                time.sleep(1)
                            elif not self.is_data_updated(converted_data) and time.time() - self.data_updates_timer > 30:
                                print("No new data received for 20 seconds, restarting monitoring...")
                                self.launch_dust_trak_monitoring()
                                self.data_updates_timer = 0
                            elif self.is_data_updated(converted_data):
                                self.data_updates_timer = 0

                                self.latest_data = {
                                    'pm1_concentration': converted_data[0],
                                    'pm2_5_concentration': converted_data[1],
                                    'pm4_concentration': converted_data[2],
                                    'pm10_concentration': converted_data[3],
                                    'avail': 'AVAILABLE'
                                }


            if self.no_packet_received_timer == 0:
                self.no_packet_received_timer = time.time()
            elif time.time() - self.no_packet_received_timer > 30:
                print("No packets received for 30 seconds, restarting monitoring...")
                self.launch_dust_trak_monitoring()
                self.no_packet_received_timer = 0
        except Exception as e:
            print(f"Error in capture loop: {e}")
            self.latest_data['avail'] = 'UNAVAILABLE'
        finally:
            try:
                loop.close()
            except Exception as e:
                print(f"Error closing event loop: {e}")
                
    def is_data_updated(self, new_data: List[float]) -> bool:
        """Check if the new data is different from the latest data"""   
        return not (self.latest_data['pm1_concentration'] == new_data[0] and
                self.latest_data['pm2_5_concentration'] == new_data[1] and
                self.latest_data['pm4_concentration'] == new_data[2] and
                self.latest_data['pm10_concentration'] == new_data[3])

    def read_data(self) -> Dict[str, float]:
        """Return the latest captured data"""
        return self.latest_data.copy()

    def parse_hex_data(self, raw_data: str) -> List[str]:
        """Decodes and parses raw hex data from TCP messages"""
        try:
            bytes_obj = bytes.fromhex(raw_data.replace(":", ""))
            decoded_str = bytes_obj.decode('utf-8')

            parsed_data = decoded_str.split(',')
            data_values = parsed_data[1:-1]
            return data_values
        except (ValueError, UnicodeDecodeError) as e:
            print(f"Error parsing hex data: {e}")
            return ['']

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
    
    def write_to_csv(self, data: Dict[str, str]) -> None:
        """Write data to CSV file"""
        csv_data = data.copy()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(current_dir, 'logs', f'{datetime.today().strftime('%Y-%m-%d')}.csv')

        csv_data['timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        with open(file=file_path, mode='a', newline='', encoding='utf-8') as csvfile:
            fieldnames = []
            for key in csv_data.keys():
                fieldnames.append(key)

            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            csvfile.seek(0, 2)
            if csvfile.tell() == 0:
                writer.writeheader()

            writer.writerow(csv_data)
    
    def is_empty_data(self, parsed_data) -> bool:
        """Check if message contains useful data"""
        is_empty = True
        for data_point in parsed_data:
            if (float(data_point) != 0.0 and float(data_point) != 91.0):
                is_empty = False
        return is_empty
