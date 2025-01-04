# Copyright 2025 FZI Forschungszentrum Informatik
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
import rclpy


def wait_for_service(srv_name, srv_type, node, timeout=10, log=None):
    if log is None:
        log = node.get_logger()

    client = node.create_client(srv_type, srv_name)

    log.info(f"Waiting for service '{srv_name}' (timeout: {timeout:.2f}s)...")
    if not client.wait_for_service(timeout):
        log.error(
            f"  Could not reach service '{srv_name}' within timeout {timeout:.2f}s"
        )
        return None

    log.info(f"  Successfully waited for service '{srv_name}'")
    return client


def call_service(client, request, node, log=None):
    if log is None:
        log = node.get_logger()

    log.info(f"Calling service '{client.srv_name}' with request '{request}'")
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is None:
        log.error(f"  Could not call service '{client.srv_name}': {future.exception()}")
        return None

    log.info(f"  Received result: {future.result()}")
    return future.result()
