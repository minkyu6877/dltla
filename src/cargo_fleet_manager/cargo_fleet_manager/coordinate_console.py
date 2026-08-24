"""Simple terminal input for a delivery destination in the simulator."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = Node('coordinate_console')
    publisher = node.create_publisher(String, '/fleet/mission', 10)
    print('Destination input: x y cargo_type  (example: 4.0 3.0 small)')
    print('cargo_type: small | long.  Type q to quit.')
    try:
        while rclpy.ok():
            raw = input('destination> ').strip()
            if raw.lower() in ('q', 'quit', 'exit'):
                break
            try:
                x, y, cargo_type = raw.split()
                command = {
                    'command': 'DELIVERY', 'dest_x': float(x),
                    'dest_y': float(y), 'cargo_type': cargo_type.lower(),
                }
            except ValueError:
                print('Use exactly: x y cargo_type')
                continue
            message = String()
            message.data = json.dumps(command)
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
            print(f'Sent: {message.data}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
