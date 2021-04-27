#!/usr/bin/env python
from hippolink_ros.transceiver import TransceiverNode


def main():
    node = TransceiverNode("transceiver")
    node.run()


if __name__ == "__main__":
    main()
