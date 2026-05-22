from __future__ import annotations

import threading
from typing import Any

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


def _make_parameter(name: str, value: Any) -> Parameter:
    """Wrap a Python value in an rcl_interfaces/msg/Parameter."""
    p = Parameter()
    p.name = name
    pv = ParameterValue()
    if isinstance(value, bool):  # must precede int; bool is an int subclass
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = value
    elif isinstance(value, int):
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)
    elif isinstance(value, float):
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
    elif isinstance(value, str):
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = value
    elif isinstance(value, (list, tuple)):
        items = list(value)
        if items and all(isinstance(v, bool) for v in items):
            pv.type = ParameterType.PARAMETER_BOOL_ARRAY
            pv.bool_array_value = [bool(v) for v in items]
        elif items and all(isinstance(v, str) for v in items):
            pv.type = ParameterType.PARAMETER_STRING_ARRAY
            pv.string_array_value = [str(v) for v in items]
        elif items and all(isinstance(v, int) and not isinstance(v, bool) for v in items):
            pv.type = ParameterType.PARAMETER_INTEGER_ARRAY
            pv.integer_array_value = [int(v) for v in items]
        else:
            # Default numeric/empty/mixed-numeric lists to double array.
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(v) for v in items]
    else:
        raise ValueError(f"unsupported parameter value type for {name}: {type(value).__name__}")
    p.value = pv
    return p


def _parameter_value_to_python(value: ParameterValue) -> Any:
    if value.type == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return int(value.integer_value)
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_STRING:
        return str(value.string_value)
    if value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        return [bool(v) for v in value.bool_array_value]
    if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return [int(v) for v in value.integer_array_value]
    if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return [float(v) for v in value.double_array_value]
    if value.type == ParameterType.PARAMETER_STRING_ARRAY:
        return [str(v) for v in value.string_array_value]
    return None


class ParamsGateway:
    def __init__(self, node: Any) -> None:
        self._node = node

    def set(self, node_name: str, params: dict[str, Any]) -> tuple[bool, str]:
        """Set parameters on another node via its /<node>/set_parameters service."""
        if not node_name:
            return False, "node name required"
        service_name = f"/{node_name.strip('/')}/set_parameters"
        client = self._node.create_client(SetParameters, service_name)
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return False, f"service {service_name} not available"

            request = SetParameters.Request()
            for name, value in params.items():
                try:
                    request.parameters.append(_make_parameter(name, value))
                except ValueError as exc:
                    return False, str(exc)

            done = threading.Event()
            outcome: dict[str, Any] = {"ok": False, "message": "set_parameters timed out"}
            future = client.call_async(request)

            def _finished(_future: Any) -> None:
                try:
                    response = _future.result()
                    failures = [
                        f"{p.name}: {r.reason or 'rejected'}"
                        for p, r in zip(request.parameters, response.results)
                        if not r.successful
                    ]
                    if failures:
                        outcome["message"] = "; ".join(failures)
                    else:
                        outcome["ok"] = True
                        outcome["message"] = (
                            f"set {len(request.parameters)} parameter(s) on {node_name}"
                        )
                except Exception as exc:  # noqa: BLE001
                    outcome["message"] = f"set_parameters failed: {exc}"
                finally:
                    done.set()

            future.add_done_callback(_finished)
            done.wait(timeout=3.0)
            return bool(outcome["ok"]), str(outcome["message"])
        finally:
            # Do not leak service clients across many tuning calls.
            self._node.destroy_client(client)

    def get(self, node_name: str, names: list[str]) -> tuple[bool, dict[str, Any], str]:
        """Read parameters from another node via its /<node>/get_parameters service."""
        if not node_name:
            return False, {}, "node name required"
        clean_names = [str(name).strip() for name in names if str(name).strip()]
        if not clean_names:
            return False, {}, "parameter names required"

        service_name = f"/{node_name.strip('/')}/get_parameters"
        client = self._node.create_client(GetParameters, service_name)
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return False, {}, f"service {service_name} not available"

            request = GetParameters.Request()
            request.names = clean_names
            done = threading.Event()
            outcome: dict[str, Any] = {
                "ok": False,
                "params": {},
                "message": "get_parameters timed out",
            }
            future = client.call_async(request)

            def _finished(_future: Any) -> None:
                try:
                    response = _future.result()
                    outcome["params"] = {
                        name: _parameter_value_to_python(value)
                        for name, value in zip(clean_names, response.values)
                    }
                    outcome["ok"] = True
                    outcome["message"] = (
                        f"read {len(outcome['params'])} parameter(s) from {node_name}"
                    )
                except Exception as exc:  # noqa: BLE001
                    outcome["message"] = f"get_parameters failed: {exc}"
                finally:
                    done.set()

            future.add_done_callback(_finished)
            done.wait(timeout=3.0)
            return (
                bool(outcome["ok"]),
                dict(outcome["params"]),
                str(outcome["message"]),
            )
        finally:
            self._node.destroy_client(client)
