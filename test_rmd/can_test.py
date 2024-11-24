import can

def main():
    # Create a CAN bus instance
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    print("CAN Receiver")

    try:
        while True:
            # Receive a message
            msg = bus.recv(timeout=1.0)  # Timeout in seconds

            if msg is None:
                continue

            print(f"Received {'extended ' if msg.is_extended_id else ''}"
                  f"{'RTR ' if msg.is_remote_frame else ''}packet with id 0x{msg.arbitration_id:X}")

            if msg.is_remote_frame:
                print(f" and requested length {msg.dlc}")
            else:
                print(f" and length {msg.dlc}")
                print(f"Data: {msg.data.hex()}")
            print()

    except KeyboardInterrupt:
        print("Program interrupted by user")

if __name__ == "__main__":
    main()
