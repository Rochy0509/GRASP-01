import requests
import time
import subprocess

# Replace with your ESP32's IP address
esp32_ip = '192.168.4.1'
url = f'http://{esp32_ip}/command'

def connect_to_esp32_ap():
    ssid = 'ESP32_AP'
    password = '12345678'
    # Connect to the ESP32 AP
    try:
        subprocess.run(['nmcli', 'd', 'wifi', 'connect', ssid, 'password', password], check=True)
        print(f"Connected to {ssid}")
    except subprocess.CalledProcessError:
        print(f"Failed to connect to {ssid}")
        return False
    return True

def send_command():
    try:
        response = requests.get(url)
        if response.status_code == 200:
            print(f"Received: {response.text}")
        else:
            print(f"Failed to get response, status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")

def main():
    if connect_to_esp32_ap():
        # Wait a moment to ensure connection is established
        time.sleep(5)
        send_command()

if __name__ == "__main__":
    main()
