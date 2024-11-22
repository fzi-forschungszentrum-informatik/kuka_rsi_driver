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
import xml.etree.ElementTree as ET
from kuka_rsi_driver.rsi import RsiState


def parse_elements(root_node):
    node_elements = root_node.find("ELEMENTS")

    signals = {}

    for node in node_elements.findall("ELEMENT"):
        data_type = node.attrib["TYPE"]

        tag_parts = node.attrib["TAG"].split(".", 1)
        if len(tag_parts) == 1:
            resolved_name, resolved_signals = resolve_builtins(tag_parts[0], data_type)
            signals[resolved_name] = resolved_signals
        else:
            if tag_parts[0] not in signals:
                signals[tag_parts[0]] = []

            signals[tag_parts[0]].append(Signal(tag_parts[1], data_type))

    return signals


def resolve_builtins(name, data_type):
    names_cart = ["X", "Y", "Z", "A", "B", "C"]
    names_joint = [f"A{i+1}" for i in range(6)]

    builtins = {
        "DEF_RIst": [Signal(a, data_type) for a in names_cart],
        "DEF_RSol": [Signal(a, data_type) for a in names_cart],
        "DEF_AIPos": [Signal(a, data_type) for a in names_joint],
        "DEF_ASPos": [Signal(a, data_type) for a in names_joint],
        "DEF_Delay": [Signal("D", data_type)],
    }

    if name in builtins:
        return (name[4:], builtins[name])

    return (name, Signal(name, data_type))


class SignalSet:
    def __init__(self, signals):
        self.signals = signals

    def create_default_state(self, default_values):
        result = {}

        for name, signal in self.signals.items():
            if isinstance(signal, list):
                result[name] = {s.name: default_values[s.data_type] for s in signal}

            else:
                result[name] = default_values[signal.data_type]

        return RsiState(result)

    def __str__(self):
        def element_str(name, el):
            if isinstance(el, list):
                attr_strs = [f"{s}" for s in el]
                return f"{name}<{','.join(attr_strs)}>"
            else:
                return f"{el}"

        element_strs = [element_str(name, el) for name, el in self.signals.items()]
        return ",".join(element_strs)


class Config:
    def __init__(self, host, sentype, send_signals, receive_signals):
        self.host = host
        self.sentype = sentype

        self.send_signals = send_signals
        self.receive_signals = receive_signals

    @staticmethod
    def parse_config(path):
        tree = ET.parse(path)
        root = tree.getroot()
        node_config = root.find("CONFIG")

        host_ip = node_config.find("IP_NUMBER").text
        host_port = int(node_config.find("PORT").text)

        sentype = node_config.find("SENTYPE").text

        send_signals = SignalSet(parse_elements(root.find("SEND")))
        receive_signals = SignalSet(parse_elements(root.find("RECEIVE")))

        return Config((host_ip, host_port), sentype, send_signals, receive_signals)


class Signal:
    def __init__(self, name, data_type):
        self.name = name
        self.data_type = data_type

    def __str__(self):
        return f"{self.name}[{self.data_type[0]}]"
