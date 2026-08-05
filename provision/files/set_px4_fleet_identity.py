#!/usr/bin/env python3
"""Set one disarmed PX4's persistent DDS domain and namespace index."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from px4_msgs.msg import ParameterRequest, ParameterResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def topic(namespace: str, suffix: str) -> str:
    prefix = f"/{namespace.strip('/')}" if namespace.strip("/") else ""
    return f"{prefix}/fmu/{suffix}"


def encoded_name(name: str) -> list[int]:
    raw = name.encode("ascii")
    return list(raw + bytes(17 - len(raw)))


class ParameterClient(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("set_px4_fleet_identity")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=8,
        )
        self.publisher = self.create_publisher(
            ParameterRequest, topic(namespace, "in/parameter_request"), qos
        )
        self.subscription = self.create_subscription(
            ParameterResponse,
            topic(namespace, "out/parameter_response"),
            self._on_response,
            qos,
        )
        self._request_id = time.monotonic_ns() & 0xFFFFFFFF
        self._wanted_id: int | None = None
        self._response: ParameterResponse | None = None

    def _on_response(self, response: ParameterResponse) -> None:
        if response.request_id == self._wanted_id:
            self._response = response

    def exchange(
        self,
        name: str,
        operation: int,
        *,
        value: int = 0,
        timeout_s: float,
    ) -> ParameterResponse:
        self._request_id = (self._request_id + 1) & 0xFFFFFFFF
        request = ParameterRequest()
        request.timestamp = self.get_clock().now().nanoseconds // 1000
        request.request_id = self._request_id
        request.operation = operation
        request.parameter_type = (
            ParameterRequest.VALUE_TYPE_INT32
            if operation == ParameterRequest.OPERATION_WRITE
            else ParameterRequest.VALUE_TYPE_UNKNOWN
        )
        request.name = encoded_name(name)
        request.int_value = value

        self._wanted_id = request.request_id
        self._response = None
        deadline = time.monotonic() + timeout_s
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publisher.publish(request)
                next_publish = now + 0.25
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - now)))
            if self._response is not None:
                return self._response
        raise TimeoutError(name)


def checked(response: ParameterResponse, name: str) -> ParameterResponse:
    if response.result != ParameterResponse.RESULT_SUCCESS:
        raise RuntimeError(f"PX4 rejected {name} with result {response.result}")
    if response.parameter_type != ParameterResponse.VALUE_TYPE_INT32:
        raise RuntimeError(f"{name} is not an INT32 parameter")
    return response


def connect(namespaces: list[str], timeout_s: float) -> tuple[str, ParameterClient]:
    for namespace in namespaces:
        client = ParameterClient(namespace)
        try:
            response = client.exchange(
                "UXRCE_DDS_DOM_ID",
                ParameterRequest.OPERATION_READ,
                timeout_s=timeout_s,
            )
            checked(response, "UXRCE_DDS_DOM_ID")
            return namespace, client
        except TimeoutError:
            client.destroy_node()
    tried = ", ".join(f"/{value}" if value else "<bare>" for value in namespaces)
    raise RuntimeError(f"no PX4 parameter response in namespaces: {tried}")


def set_if_needed(
    client: ParameterClient, name: str, value: int, timeout_s: float
) -> bool:
    current = checked(
        client.exchange(name, ParameterRequest.OPERATION_READ, timeout_s=timeout_s),
        name,
    )
    if current.int_value == value:
        return False
    written = checked(
        client.exchange(
            name,
            ParameterRequest.OPERATION_WRITE,
            value=value,
            timeout_s=timeout_s,
        ),
        name,
    )
    if written.int_value != value:
        raise RuntimeError(f"{name} read back as {written.int_value}, expected {value}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=4.0)
    args = parser.parse_args()
    if not 0 <= args.index <= 9999:
        parser.error("--index must be between 0 and 9999")
    if not 0 <= args.domain <= 232:
        parser.error("--domain must be between 0 and 232")

    desired_namespace = f"px4_{args.index}"
    rclpy.init()
    client: ParameterClient | None = None
    try:
        namespace, client = connect([desired_namespace, ""], args.timeout)
        changed = set_if_needed(client, "UXRCE_DDS_DOM_ID", args.domain, args.timeout)
        changed |= set_if_needed(client, "UXRCE_DDS_NS_IDX", args.index, args.timeout)
        state = "CHANGED" if changed else "UNCHANGED"
        source = f"/{namespace}" if namespace else "bare namespace"
        print(
            f"{state}: PX4 reached through {source}; domain={args.domain}, "
            f"namespace={desired_namespace}"
        )
        if changed:
            print("Power-cycle the flight controller before namespace verification.")
        return 0
    finally:
        if client is not None:
            client.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
