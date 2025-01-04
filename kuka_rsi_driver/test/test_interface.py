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
from rclpy.action import ActionClient

from controller_manager_msgs.srv import ListControllers, SwitchController
from action_msgs.msg import GoalStatus


class ControllerManagerInterface:
    def __init__(self, node):
        self._node = node
        self._log = node.get_logger().get_child("controller_manager_interface")

        self._list_controllers_client = wait_for_service(
            "controller_manager/list_controllers", ListControllers, node, 10, self._log
        )
        self._switch_controllers_client = wait_for_service(
            "controller_manager/switch_controller", SwitchController, node, 3, self._log
        )

    def list_controllers(self):
        res = call_service(
            self._list_controllers_client,
            ListControllers.Request(),
            self._node,
            self._log,
        )

        if len(res.controller) > 0:
            self._log.info("Controllers:")
            for controller in res.controller:
                self._log.info(
                    f"  - {controller.name} ({controller.type}) - {controller.state}"
                )
        else:
            self._log.info("Controllers: []")

        return res.controller

    def switch_controller(self, activate=[], deactivate=[], strict=True):
        request = SwitchController.Request(
            activate_controllers=activate,
            deactivate_controllers=deactivate,
            strictness=(
                SwitchController.Request.STRICT
                if strict
                else SwitchController.Request.BEST_EFFORT
            ),
        )
        res = call_service(
            self._switch_controllers_client, request, self._node, self._log
        )

        if not res.ok:
            error_msg = "Could not switch controllers"
            self._log.error(error_msg)
            raise RuntimeError(error_msg)

        self._log.info(
            f"Switched controllers ({'strict' if strict else 'best_effort'}):"
        )
        self._log.info(f"  - Activate: {activate}")
        self._log.info(f"  - Deactivate: {deactivate}")


class ActionInterface:
    def __init__(self, action_name, action_type, node, timeout=10):
        self._node = node
        self._log = node.get_logger().get_child(action_name.split("/")[-1])

        self._client = wait_for_action(
            action_name, action_type, self._node, timeout, self._log
        )

    def send_goal(self, goal, timeout=10):
        # Send goal
        self._log.info(f"Sending goal to action server: {goal}")
        goal_future = self._client.send_goal_async(goal)

        self._log.info("  Waiting for goal acceptance")
        rclpy.spin_until_future_complete(self._node, goal_future)

        # Get goal handle
        if goal_future.result() is None:
            error_msg = (
                f"Could not send goal to action server: {goal_future.exception()}"
            )
            self._log.error(f"  {error_msg}")
            raise RuntimeError(error_msg)

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            error_msg = "  Goal was not accepted"
            self._log.error(f"  {error_msg}")
            raise RuntimeError(error_msg)

        uuid_str = "".join([f"{v:02x}" for v in goal_handle.goal_id.uuid])
        self._log.info(f"  Goal {uuid_str} accepted")

        # Wait for result
        result_future = goal_handle.get_result_async()

        self._log.info(f"  Waiting {timeout:.2f}s for result")
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)

        # Check result
        response = result_future.result()
        if response is None:
            error_msg = (
                f"  Could not get result from action: {result_future.exception()}"
            )
        elif response.status != GoalStatus.STATUS_SUCCEEDED:
            error_msg = (
                f"  Goal did not succeed (status={response.status}): {response.result}"
            )
        else:
            self._log.info(f"  Received result: {response.result}")
            return response.result

        self._log.error(f"  {error_msg}")
        raise RuntimeError(error_msg)


def wait_for_service(srv_name, srv_type, node, timeout=10, log=None):
    if log is None:
        log = node.get_logger()

    client = node.create_client(srv_type, srv_name)

    log.info(f"Waiting for service '{srv_name}' (timeout: {timeout:.2f}s)...")
    if not client.wait_for_service(timeout):
        error_msg = (
            f"Could not reach service '{srv_name}' within timeout {timeout:.2f}s"
        )
        log.error(f"  {error_msg}")
        raise RuntimeError(error_msg)

    log.info(f"  Successfully waited for service '{srv_name}'")
    return client


def call_service(client, request, node, log=None):
    if log is None:
        log = node.get_logger()

    log.info(f"Calling service '{client.srv_name}' with request '{request}'")
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is None:
        error_msg = f"Could not call service '{client.srv_name}': {future.exception()}"
        log.error(f"  {error_msg}")
        raise RuntimeError(error_msg)

    log.info(f"  Received result: {future.result()}")
    return future.result()


def wait_for_action(action_name, action_type, node, timeout=10, log=None):
    if log is None:
        log = node.get_logger()

    client = ActionClient(node, action_type, action_name)

    log.info(f"Waiting for action '{action_name}' (timeout: {timeout:.2f}s)...")
    if not client.wait_for_server(timeout):
        error_msg = f"Could not reach action server '{action_name}' within timeout {timeout:.2f}s"
        log.error(f"  {error_msg}")
        raise RuntimeError(error_msg)

    log.info(f"  Successfully waited for action server '{action_name}'")
    return client
