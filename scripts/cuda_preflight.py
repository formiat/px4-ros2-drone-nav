#!/usr/bin/env python3
"""Fail a simulation before PX4/Gazebo if the CUDA runtime cannot allocate."""

from __future__ import annotations

import ctypes
import ctypes.util
import sys


def error_text(runtime: ctypes.CDLL, code: int) -> str:
    runtime.cudaGetErrorString.argtypes = [ctypes.c_int]
    runtime.cudaGetErrorString.restype = ctypes.c_char_p
    text = runtime.cudaGetErrorString(code)
    return text.decode("utf-8", errors="replace") if text else "unknown CUDA error"


def check(runtime: ctypes.CDLL, operation: str, code: int) -> None:
    if code != 0:
        raise RuntimeError(f"{operation} failed: {error_text(runtime, code)} ({code})")


def main() -> int:
    library = ctypes.util.find_library("cudart") or "libcudart.so"
    try:
        runtime = ctypes.CDLL(library)
    except OSError as error:
        print(f"CUDA_PREFLIGHT status=failed stage=load_runtime detail={error}", file=sys.stderr)
        return 2

    runtime.cudaDriverGetVersion.argtypes = [ctypes.POINTER(ctypes.c_int)]
    runtime.cudaRuntimeGetVersion.argtypes = [ctypes.POINTER(ctypes.c_int)]
    runtime.cudaGetDeviceCount.argtypes = [ctypes.POINTER(ctypes.c_int)]
    driver = ctypes.c_int()
    runtime_version = ctypes.c_int()
    devices = ctypes.c_int()
    try:
        check(runtime, "cudaDriverGetVersion", runtime.cudaDriverGetVersion(ctypes.byref(driver)))
        check(runtime, "cudaRuntimeGetVersion", runtime.cudaRuntimeGetVersion(ctypes.byref(runtime_version)))
        check(runtime, "cudaGetDeviceCount", runtime.cudaGetDeviceCount(ctypes.byref(devices)))
        if devices.value < 1:
            raise RuntimeError("cudaGetDeviceCount returned no devices")
        allocation = ctypes.c_void_p()
        runtime.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        runtime.cudaFree.argtypes = [ctypes.c_void_p]
        check(runtime, "cudaMalloc", runtime.cudaMalloc(ctypes.byref(allocation), 4))
        try:
            check(runtime, "cudaFree", runtime.cudaFree(allocation))
            allocation = ctypes.c_void_p()
        finally:
            if allocation.value:
                runtime.cudaFree(allocation)
    except RuntimeError as error:
        print(f"CUDA_PREFLIGHT status=failed stage=runtime detail={error}", file=sys.stderr)
        return 2

    print(
        "CUDA_PREFLIGHT status=ready "
        f"driver_version={driver.value} runtime_version={runtime_version.value} devices={devices.value}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
