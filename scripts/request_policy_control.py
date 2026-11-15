#!/usr/bin/env python3
"""Return robot ownership to the policy before starting the motion bridge."""

import sys


def main() -> int:
    import rclpy
    from std_srvs.srv import SetBool

    rclpy.init()
    node = rclpy.create_node("towel_request_policy_control")
    try:
        service = "/dexbotCtl/vr_override"
        client = node.create_client(SetBool, service)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{service} unavailable; start make robot and wait for initialization")
        request = SetBool.Request()
        request.data = False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"{service} response timed out; motion bridge was not started")
        response = future.result()
        if response is None or not response.success:
            reason = response.message if response is not None else "empty service response"
            raise RuntimeError(f"Policy ownership rejected: {reason}")
        print(f"Policy control granted: {response.message}", flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
