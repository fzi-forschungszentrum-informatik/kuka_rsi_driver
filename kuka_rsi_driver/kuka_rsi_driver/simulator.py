# Copyright 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
import threading
import argparse
import rclpy
import rclpy.utilities
import socket
import rclpy.executors
from ament_index_python.packages import get_package_share_directory
from kuka_rsi_driver.config import Config
from kuka_rsi_driver.rsi import RsiCommand


class Simulator:
    DEFAULT_VALUES = {"BOOL": False, "DOUBLE": 0.0, "LONG": 0}

    def __init__(self, config, node, log):
        self.__config = config

        self.__node = node
        self.__log = log

        self.__state = config.send_signals.create_default_state(self.DEFAULT_VALUES)
        if "OvPro" in self.__state.values:
            self.__state.values["OvPro"]["R"] = 100.0
        if "ProgStatus" in self.__state.values:
            self.__state.values["ProgStatus"]["R"] = 3

        self.__socket = self.__create_socket()

        self.__ipoc = 0

        self.__log.info(
            f"Started simulation node {self.__node.get_name()} for sentype '{config.sentype}' at {config.host[0]}:{config.host[1]}"
        )
        self.__log.info(f"Initial state: {self.__state}")

    def tick(self):
        try:
            state_str = self.__state.xml(self.__config.sentype, self.__ipoc)
            self.__socket.sendto(state_str, self.__config.host)

            cmd_str, addr = self.__socket.recvfrom(1024)
            cmd = RsiCommand.parse_xml(cmd_str)

            if "AK" in cmd.values:
                self.__state.values["AIPos"] = cmd.values["AK"]

            self.__ipoc += 1

        except socket.timeout:
            self.__log.warn("Receive timeout")

    def __create_socket(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1)

            return s
        except OSError as ex:
            raise RuntimeError(f"Could not create socket: {ex}")


def main():
    rclpy.init()

    args = parse_args()
    config = Config.parse_config(args.config)

    if args.host_ip is not None:
        config.host = (args.host_ip, config.host[1])
    if args.host_port is not None:
        config.host = (config.host[0], args.host_port)

    node = rclpy.create_node("kuka_rsi_simulator")
    log = node.get_logger()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    sim = Simulator(config, node, log)
    rate = node.create_rate(args.rate)
    while rclpy.ok():
        sim.tick()
        rate.sleep()

    executor.shutdown()
    executor_thread.join()
    rclpy.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description="RSI simulator")

    parser.add_argument(
        "--rate", type=float, default=250, help="Frequency at which to run simulation"
    )

    default_config = os.path.join(
        get_package_share_directory("kuka_rsi_driver"),
        "krl",
        "Common",
        "ros_rsi_ethernet.xml",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=default_config,
        help="Path of RSI ethernet configuration",
    )

    parser.add_argument("--host-ip", type=str, help="Overwrite the host ip address")
    parser.add_argument("--host-port", type=int, help="Overwrite the host port")

    args = rclpy.utilities.remove_ros_args()[1:]
    return parser.parse_args(args)
